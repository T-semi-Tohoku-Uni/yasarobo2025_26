#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <inrof2025_ros_type/srv/waypoint.hpp>
#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace path {
    class PathGenerator: public rclcpp::Node {
        public:
            explicit PathGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
                : Node("path_generator", options), waypoint_array_{} {
                // TODO: get from launch file
                this->declare_parameter<std::float_t>("initial_x", 0.25);
                this->declare_parameter<std::float_t>("initial_y", 0.25);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);
                this->declare_parameter<std::float_t>("sample_parameter", 15.0);

                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();
                sample_parameter_ = this->get_parameter("sample_parameter").as_double();

                this->curOdom_.x = initial_x;
                this->curOdom_.y = initial_y;
                this->curOdom_.theta = initial_theta;

                this->mapResolution_ = 0.01;
                this->mapWidth_ = 182;
                this->mapHeight_ = 232;
                this->mapDir_ = "src/yasarobo2025_26/map/";

                readMap();

                // initialize publisher
                rclcpp::QoS pathQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliable()
                                  .transient_local();
                rclcpp::QoS test_pathQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliable()
                                  .transient_local();
                pubPath_ = create_publisher<nav_msgs::msg::Path>("route", pathQos);
                pubSamplePath_ = create_publisher<nav_msgs::msg::Path>("test_route", test_pathQos);

                rclcpp::QoS markerQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .reliable()
                                  .transient_local();
                pubMarker_ = create_publisher<visualization_msgs::msg::MarkerArray>("path_orientations", markerQos);


                // initialize subscriber
                rclcpp::QoS sOdomQos(rclcpp::KeepLast(10));
                subOdom_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                    "pose", sOdomQos, std::bind(&PathGenerator::odomCallback, this, std::placeholders::_1)
                );

                // initialize service server
                srvOdom_= this->create_service<inrof2025_ros_type::srv::GenRoute>(
                    "generate_route", std::bind(&PathGenerator::poseCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
                srvWaypoint_ = this->create_service<inrof2025_ros_type::srv::Waypoint>(
                    "waypoint", std::bind(&PathGenerator::waypointCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
                
                RCLCPP_INFO(this->get_logger(), "Success initialze");
            }
        private:
        struct mapNode {
            int r, c;
            double width;
            double cost;
        };

        void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {
            *u = (std::int32_t)(x / mapResolution_);
            *v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
        };

        struct Cell {
            int u, v;
            double cost;

            bool operator>(const Cell& other) const {
                return cost > other.cost;
            }
        };

        void odomCallback(geometry_msgs::msg::Pose2D msgs) {
            // TODO lock
            curOdom_.x = msgs.x;
            curOdom_.y = msgs.y;
            curOdom_.theta = msgs.theta; // null ok
        }

        // void poseCallback(inrof2025_ros_type::srv::GenRoute srvs) {
        //     RCLCPP_INFO(this->get_logger(), "jfoejrifojerifjiorejfoierjfierjfojer");
        //     // TODO lock
        //     goalOdom_.x = srvs->x;
        //     goalOdom_.y = srvs->y;
        //     goalOdom_.theta = 0.0; // null ok

        //     generator();
        // }

        void waypointCallback(
            const std::shared_ptr<inrof2025_ros_type::srv::Waypoint::Request> request,
            const std::shared_ptr<inrof2025_ros_type::srv::Waypoint::Response> response
        ) {
            // Add array
            waypoint_array_.push_back(std::make_pair(request->x, request->y));
        }

        void poseCallback(
            const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Request> request,
            const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Response> response
        ) {
            RCLCPP_INFO(this->get_logger(), "%.4f %.4f", request->x, request->y);

            // Add goal odom
            waypoint_array_.push_back(std::make_pair(request->x, request->y));

            std::vector<std::pair<double, double>> path;
            std::pair<double, double> start_point = std::make_pair(curOdom_.x, curOdom_.y); 
            for (size_t i=0; i<waypoint_array_.size(); i++ ) {
                std::vector<std::pair<double, double>> partial_pass = generator(start_point, waypoint_array_[i]);
                path.insert(path.end(), partial_pass.begin(), partial_pass.end());
                start_point = waypoint_array_[i];
            }

            path = splineSmoothEigen(path);

            // create path message
            nav_msgs::msg::Path pathMsg;
            pathMsg.header.frame_id = "map";
            pathMsg.header.stamp    = this->now();
            for (size_t i=0; i<path.size(); i++) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = pathMsg.header;

                pose.pose.position.x = path[i].first;
                pose.pose.position.y = path[i].second;
                pose.pose.position.z = 0.0;

                tf2::Quaternion q;
                if (i+30 > path.size()) {
                    q.setRPY(0, 0, request->theta);
                } else {
                    q.setRPY(0, 0, curOdom_.theta);
                }
                pose.pose.orientation = tf2::toMsg(q);
                pathMsg.poses.push_back(pose);
            }
            pubPath_->publish(pathMsg);

            // create arrow message

            visualization_msgs::msg::MarkerArray markerArray;
            // delete old marker
            visualization_msgs::msg::Marker del;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            markerArray.markers.push_back(del);
            // add marker
            for (size_t i=0; i<pathMsg.poses.size(); i+=10) {
                visualization_msgs::msg::Marker arrow;
                arrow.header = pathMsg.header;
                arrow.ns = "path_orientations";
                arrow.id = static_cast<int>(i);
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose = pathMsg.poses[i].pose;
                arrow.scale.x = 0.05;
                arrow.scale.y = 0.01;
                arrow.scale.z = 0.01;

                arrow.color.r = 1.0f;
                arrow.color.g = 0.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 1.0f;
                markerArray.markers.push_back(arrow);
            }
            pubMarker_->publish(markerArray);

            // clear waypoint array
            waypoint_array_.clear();
        }
        
        std::vector<std::pair<double, double>> splineSmoothEigen(const std::vector<std::pair<double, double>> &original_path) {
            using Spline2d = Eigen::Spline<double, 2>;
            using Vec2 = Eigen::Matrix<double, 2, 1>;

            // sampling point from original path
            std::vector<std::pair<double, double>> sampled_path;
            for (size_t i=0; i<original_path.size(); i+=sample_parameter_ ) {
                sampled_path.push_back(original_path[i]);
            }
            sampled_path.push_back(original_path.back());

            const int degree = 3;
            if (sampled_path.size() < 4) return original_path;
            // sampleする点はdegree+1以上
            if (sampled_path.size() <= degree) return original_path;

            Eigen::Matrix<double, 2, Eigen::Dynamic> points(2, sampled_path.size());
            for (size_t i = 0; i < sampled_path.size(); i++) {
                points(0, i) = sampled_path[i].first;
                points(1, i) = sampled_path[i].second;
            }

            Eigen::RowVectorXd u(sampled_path.size());
            for (int i = 0; i < sampled_path.size(); ++i) {
               u(i) = static_cast<double>(i) / double(sampled_path.size() - 1);
            }
 
            // const int degree = 3; // 3次元のspline
            // // 注意: 点数 N は degree+1 以上であること
            // if (N <= degree) return original_path;

            Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points, degree, u);
            std::vector<std::pair<double, double>> smoothed_path;
            int dense = sampled_path.size() * sample_parameter_;
            for (int i = 0; i <= dense; i++) {
                
                double t = static_cast<double>(i) / dense; // 0..1
                
                Eigen::Vector2d pv = spline(t); // p(t)

                geometry_msgs::msg::PoseStamped pose;
                smoothed_path.push_back(std::make_pair(pv.x(), pv.y()));
            }

            return smoothed_path;
        }

        std::vector<std::pair<double, double>> generator(std::pair<double, double> start_point, std::pair<double, double> goal_point) {
            double sx = start_point.first;
            double sy = start_point.second;
            double gx = goal_point.first;
            double gy = goal_point.second;

            std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> q;
            std::vector<std::vector<double>> distances(
                this->mapHeight_, std::vector<double>(this->mapWidth_, std::numeric_limits<double>::infinity())
            );
            std::vector<std::vector<std::pair<int, int>>> previous(
                this->mapHeight_, std::vector<std::pair<int, int>>(this->mapWidth_, {-1, -1})
            );
            

            int su, sv, gu, gv;
            xy2uv(sx, sy, &su, &sv);
            xy2uv(gx, gy, &gu, &gv);

            // RCLCPP_INFO(this->get_logger(), "start %d %d", su, sv);

            distances[sv][su] = 0;
            q.push({su, sv, 0});

            const int du[4] = {-1, 1, 0, 0};
            const int dv[4] = {0, 0, -1, 1};

            while(!q.empty()) { 
                Cell cur = q.top(); q.pop();
                if (cur.u == gu && cur.v == gv) break;

                for (int dir = 0; dir < 4; dir++ ) {
                    int nu = cur.u + du[dir];
                    int nv = cur.v + dv[dir];

                    if (nu >= 0 && nu < this->mapWidth_ && nv >= 0 && nv < this->mapHeight_) {
                        double cost = cur.cost + distField_.at<double>(nv, nu);
                        if (cost < distances[nv][nu]) {
                            distances[nv][nu] = cost;
                            previous[nv][nu] = {cur.u, cur.v};
                            q.push({nu, nv, cost});
                        }
                    }
                }
            }

            // 経路再構築
            std::vector<std::pair<int, int>> path_g;
            for (int u = gu, v = gv; u != -1 && v != -1; ) {
                path_g.push_back({u, v});
                std::tie(u, v) = previous[v][u];
            }

            std::reverse(path_g.begin(), path_g.end());

            // convert grid -> field
            std::vector<std::pair<double, double>> path_f;
            for (std::pair<int, int> &p: path_g) {
                path_f.push_back(
                    std::make_pair(
                        (p.first + 0.5) * mapResolution_,
                        (static_cast<double>(mapHeight_ - p.second - 1) + 0.5) * mapResolution_
                    )
                );
            }

            return path_f;
        }

        void readMap() {
            try {
                YAML::Node lconf = YAML::LoadFile(this->mapDir_ + "map.yaml");
                mapResolution_ = lconf["resolution"].as<std::double_t>();
                mapOrigin_ = lconf["origin"].as<std::vector<std::double_t>>();

                std::string imgFile = mapDir_ + "map.pgm";
                mapImg_ = cv::imread(imgFile, 0);
                mapWidth_ = mapImg_.cols;
                mapHeight_ = mapImg_.rows;

                cv::Mat mapImg = mapImg_.clone();
                for (int v = 0; v < mapHeight_; v++ ) {
                    for (int u = 0; u < mapWidth_; u++ ) {
                        uchar val = mapImg.at<uchar>(v, u);
                        if (val == 0) {
                            mapImg.at<uchar>(v, u) = 0;
                        } else {
                            mapImg.at<uchar>(v, u) = 1;
                        }
                    }
                }

                cv::Mat distFieldF(mapHeight_, mapWidth_, CV_32FC1);
                cv::Mat distFieldD(mapHeight_, mapWidth_, CV_64FC1);
                cv::distanceTransform(mapImg, distFieldF, cv::DIST_L2, 5);
                
                // 原点    : 左上
                // first  : 縦軸
                // second : 横軸
                
                for (int v = 0; v < mapHeight_; v++ ) {
                    for (int u = 0; u < mapWidth_; u++ ) {
                        std::float_t d = distFieldF.at<std::float_t>(v, u);
                        distFieldD.at<std::double_t>(v, u) = (std::double_t)d * mapResolution_;
                    }
                }

                double max_val;
                cv::minMaxLoc(distFieldD, nullptr, &max_val, nullptr, nullptr);
                cv::Mat diff = distFieldD - max_val;   // 要素ごとに引き算
                cv::Mat absDiff = cv::abs(diff);      // 要素ごとの絶対値


                // // 1) 距離場 distFieldD（CV_64F）を 0–255 に正規化して 8bit 化
                // cv::Mat normDist;
                // cv::normalize(distFieldD, normDist, 0.0, 255.0, cv::NORM_MINMAX);
                // cv::Mat dist8U;
                // normDist.convertTo(dist8U, CV_8U);

                // // 2) グレースケール→BGR に変換
                // cv::Mat colorImg;
                // cv::cvtColor(dist8U, colorImg, cv::COLOR_GRAY2BGR);

                // // 3) 特定ピクセルをマーク (row=50, col=11 を赤に)
                // //    .at は (y,x) = (row,col) の順番なので注意
                // colorImg.at<cv::Vec3b>(11, 50) = cv::Vec3b(0, 0, 255);

                // （任意）円マークを描く場合
                // cv::circle(colorImg, cv::Point(11, 50), /*半径*/ 3, cv::Scalar(0,255,0), /*塗りつぶし*/ -1);

                // 4) 画像を保存
                // cv::imwrite("distField_highlight.png", colorImg);

                distField_ = absDiff.clone();
            } catch (const YAML::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s\n", e.what());
            }
        }

        std::array<std::pair<int,int>, 8> directions8_ {{
            {-1,  0},   // 上        (north)
            { 1,  0},   // 下        (south)
            { 0, -1},   // 左        (west)
            { 0,  1},   // 右        (east)
            {-1, -1},   // 左上      (north-west)
            {-1,  1},   // 右上      (north-east)
            { 1, -1},   // 左下      (south-west)
            { 1,  1}    // 右下      (south-east)
        }};
        double sample_parameter_;
        std::string mapDir_;
        std::double_t mapResolution_;
        std::int32_t mapWidth_, mapHeight_;
        std::vector<std::double_t> mapOrigin_;
        cv::Mat mapImg_;
        cv::Mat distField_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubSamplePath_;        
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubMarker_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subOdom_;
        geometry_msgs::msg::Pose2D curOdom_;
        std::vector<std::pair<double, double>> waypoint_array_;

        // connect to behaivorTree
        rclcpp::Service<inrof2025_ros_type::srv::GenRoute>::SharedPtr srvOdom_;
        rclcpp::Service<inrof2025_ros_type::srv::Waypoint>::SharedPtr srvWaypoint_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path::PathGenerator>());
    rclcpp::shutdown();
    return 0;
}