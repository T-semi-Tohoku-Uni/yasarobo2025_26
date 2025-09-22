#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>

namespace path {
    class PathGenerator: public rclcpp::Node {
        public:
            explicit PathGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("path_generator", options) {
                // TODO: get from launch file
                this->declare_parameter<std::float_t>("initial_x", 0.25);
                this->declare_parameter<std::float_t>("initial_y", 0.25);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);

                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();

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
                pubPath_ = create_publisher<nav_msgs::msg::Path>("route", pathQos);


                // initialize subscriber
                rclcpp::QoS sOdomQos(rclcpp::KeepLast(10));
                subOdom_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                    "pose", sOdomQos, std::bind(&PathGenerator::odomCallback, this, std::placeholders::_1)
                );

                // initialize service server
                srvOdom_= this->create_service<inrof2025_ros_type::srv::GenRoute>(
                    "generate_route", std::bind(&PathGenerator::poseCallback, this, std::placeholders::_1, std::placeholders::_2)
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
            curOdom_.theta = 0.0; // null ok
        }

        // void poseCallback(inrof2025_ros_type::srv::GenRoute srvs) {
        //     RCLCPP_INFO(this->get_logger(), "jfoejrifojerifjiorejfoierjfierjfojer");
        //     // TODO lock
        //     goalOdom_.x = srvs->x;
        //     goalOdom_.y = srvs->y;
        //     goalOdom_.theta = 0.0; // null ok

        //     generator();
        // }

        void poseCallback(
            const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Request> request,
            const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Response> response
        ) {
            RCLCPP_INFO(this->get_logger(), "%.4f %.4f", request->x, request->y);
            goalOdom_.x = request->x;
            goalOdom_.y = request->y;
            goalOdom_.theta = 0.0;

            generator();
        }

        void generator() {
            double sx = curOdom_.x;
            double sy = curOdom_.y;
            double gx = goalOdom_.x;
            double gy = goalOdom_.y;

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
            std::vector<std::pair<int, int>> path;
            for (int u = gu, v = gv; u != -1 && v != -1; ) {
                path.push_back({u, v});
                std::tie(u, v) = previous[v][u];
            }

            std::reverse(path.begin(), path.end());

            nav_msgs::msg::Path pathMsg;
            pathMsg.header.frame_id = "map";
            pathMsg.header.stamp    = this->now();

            for (auto [gr, gc] : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = pathMsg.header;

                // OccupancyGrid のセル → ワールド座標（セル中心にオフセット）
                pose.pose.position.x = (gr + 0.5) * mapResolution_;
                pose.pose.position.y = (static_cast<double>(mapHeight_ - gc - 1) + 0.5) * mapResolution_;
                pose.pose.position.z = 0.0;

                // 進行方向の yaw を持たせても良いが，ここでは単位クォータニオン
                pose.pose.orientation.w = 1.0;


                // RCLCPP_INFO(this->get_logger(), "%.4f %.4f", pose.pose.position.x, pose.pose.position.y);
                
                pathMsg.poses.push_back(std::move(pose));
            }
            
            pubPath_->publish(pathMsg);
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
        std::string mapDir_;
        std::double_t mapResolution_;
        std::int32_t mapWidth_, mapHeight_;
        std::vector<std::double_t> mapOrigin_;
        cv::Mat mapImg_;
        cv::Mat distField_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subOdom_;
        geometry_msgs::msg::Pose2D curOdom_;
        geometry_msgs::msg::Pose2D goalOdom_;

        // connect to behaivorTree
        rclcpp::Service<inrof2025_ros_type::srv::GenRoute>::SharedPtr srvOdom_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path::PathGenerator>());
    rclcpp::shutdown();
    return 0;
}