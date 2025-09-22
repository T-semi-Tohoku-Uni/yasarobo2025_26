#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <cmath>
#include <cstdlib>
#include <inrof2025_ros_type/srv/ball_pose.hpp>

using namespace std::chrono_literals; 

namespace mcl {
    class Particle {
        public:
            const std::double_t& getX() const& { return pose_.x; }
            const std::double_t& getY() const& { return pose_.y; }
            const std::double_t& getTheta() const& { return pose_.theta; }
            const geometry_msgs::msg::Pose2D& getPose() const& { return pose_; }
            const std::double_t& getW() const& { return w_; }
            void setPose(std::double_t x, std::double_t y, std::double_t theta) {
                pose_.set__x(x);
                pose_.set__y(y);
                pose_.set__theta(theta);
            }
            void setW(std::double_t w) {
                w_ = w;
            }
        private:
            geometry_msgs::msg::Pose2D pose_;
            std::double_t w_;
    };

    // TODO: iterを実装してforで回してあげたい
    class probability {
        public:
            float p[1000];
            int num_scan;
    };

    enum class MeasurementModel { LikelihoodFieldModel };

    class MCL: public rclcpp::Node {
        public:
            explicit MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
                this->declare_parameter<std::int32_t>("particleNum", 100);
                this->declare_parameter<std::float_t>("initial_x", 0.25);
                this->declare_parameter<std::float_t>("initial_y", 0.25);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);
                this->declare_parameter<std::float_t>("resampleThreshold", 0.5);
                this->declare_parameter<std::float_t>("odomNoise1", 1.0);
                this->declare_parameter<std::float_t>("odomNoise2", 1.0);
                this->declare_parameter<std::float_t>("odomNoise3", 1.0);
                this->declare_parameter<std::float_t>("odomNoise4", 1.0);
                
                particleNum_ = this->get_parameter("particleNum").as_int();
                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();
                this->resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();
                this->odomNoise1_ = this->get_parameter("odomNoise1").as_double();
                this->odomNoise2_ = this->get_parameter("odomNoise2").as_double();
                this->odomNoise3_ = this->get_parameter("odomNoise3").as_double();
                this->odomNoise4_ = this->get_parameter("odomNoise4").as_double();
                particles_.resize(particleNum_);
                pro_.resize(particleNum_);

                measurementLikelihoods_.resize(particleNum_);
            
                // init robot pos
                geometry_msgs::msg::Pose2D pose;
                // TODO: get parameter from user
                pose.set__x(initial_x);
                pose.set__y(initial_y);
                pose.set__theta(initial_theta);
                // initalize mclPose
                setMCLPose(pose);
                velOdom_.set__x(initial_x);
                velOdom_.set__y(initial_y);
                velOdom_.set__theta(initial_theta);
                
                // initialize particle
                geometry_msgs::msg::Pose2D initialNoise;
                auto cloud_qos = rclcpp::SensorDataQoS();
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", cloud_qos);
                initialNoise.set__x(0.07); // var of x
                initialNoise.set__y(0.07); // var of y
                initialNoise.set__theta(M_PI/180.0); // var of theta
                resetParticlesDistribution(initialNoise);
                printParticlesMakerOnRviz2();
                
                // set mesurementModel
                measurementModel_ = MeasurementModel::LikelihoodFieldModel;

                // TODO: move parameter settings to yaml file
                this->mapResolution_ = 0.01;
                this->mapWidth_ = 182;
                this->mapHeight_ = 232;
                this->mapDir_ = "src/yasarobo2025_26/map/";
                this->scanStep_ = 50;
                this->lfmSigma_ = 0.03;
                this->zHit_ = 1.0;
                this->zMax_ = 0.0;
                this->zRand_ = 1.0;

                MCL::readMap();
                
                last_timestamp_ = this->get_clock()->now();
                // setup subscriper
                rclcpp::QoS cmdVelQos(rclcpp::KeepLast(10));
                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel_feedback", cmdVelQos, std::bind(&MCL::cmdVelCallback, this, std::placeholders::_1)
                );

                // TODO: ポーリングするなにかをつくりたいな（いじっているときに値が変更する可能性があるおがキモい）
                // rclcpp::QoS laserScanQos(rclcpp::KeepLast(10));
                auto laserScanQos = rclcpp::SensorDataQoS();
                subLayerScan_ = create_subscription<sensor_msgs::msg::LaserScan>(
                    "/ldlidar_node/scan", laserScanQos, std::bind(&MCL::laserScanCallback, this, std::placeholders::_1)
                );
                // rclcpp::QoS callbackQos(rclcpp::KeepLast(10));
                // subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
                //     "/odom", callbackQos, std::bind(&MCL::odomCallback, this, std::placeholders::_1)
                // );

                tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

                pubPath_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
                path_.header.frame_id = "map";
                pubPose_ = create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);
                srvBallPose_ = this->create_service<inrof2025_ros_type::srv::BallPose>(
                    "ball_pose", std::bind(&MCL::poseCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
                
                // s_odom_ = create_subscription<nav_msgs::msg::Odometry>(
                //     "/odom", tmp_qos, std::bind(&MCL::odomCallback, this, std::placeholders::_1)
                // );

                const char *sim = std::getenv("WITH_SIM");
                // RCLCPP_INFO(this->get_logger(), "freofkprekfore");
                if (!sim || std::string(sim) != "1") {
                    is_sim_ = false;
                } else {
                    is_sim_ = true;
                }

                // setup publisher
                iter_=0;
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MCL::loop, this));
                RCLCPP_INFO(this->get_logger(), "Success initialize");

                // TODO: deleteb
            } 

        private:
            void setMCLPose(geometry_msgs::msg::Pose2D pose) { mclPose_=pose; }
            std::double_t randNormal(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }
            
            // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            //     auto &q = msg->pose.pose.orientation;
            //     // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            //     double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            //     double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            //     double yaw = std::atan2(siny_cosp, cosy_cosp);
                
            //     // if (!cmdVel_) {
            //     //     return;
            //     // }
            //     yaw_ = yaw;
            //     odom_twist_ = msg->twist.twist.angular.z;
            //     // RCLCPP_INFO(this->get_logger(), "%.3f", yaw_);
                // RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.3f %.3f %.3f", yaw, msg->twist.twist.angular.z, particles_[0].getTheta());
            // }

            void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
                scan_ = scan;
            }
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_=msg;
            }
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_=msg;
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
                    RCLCPP_INFO(this->get_logger(), "(11, 50) = %lf", distFieldF.at<std::float_t>(11, 50));

                    // 1) 距離場 distFieldD（CV_64F）を 0–255 に正規化して 8bit 化
                    cv::Mat normDist;
                    cv::normalize(distFieldD, normDist, 0.0, 255.0, cv::NORM_MINMAX);
                    cv::Mat dist8U;
                    normDist.convertTo(dist8U, CV_8U);

                    // 2) グレースケール→BGR に変換
                    cv::Mat colorImg;
                    cv::cvtColor(dist8U, colorImg, cv::COLOR_GRAY2BGR);

                    // 3) 特定ピクセルをマーク (row=50, col=11 を赤に)
                    //    .at は (y,x) = (row,col) の順番なので注意
                    colorImg.at<cv::Vec3b>(11, 50) = cv::Vec3b(0, 0, 255);

                    // （任意）円マークを描く場合
                    cv::circle(colorImg, cv::Point(11, 50), /*半径*/ 3, cv::Scalar(0,255,0), /*塗りつぶし*/ -1);

                    // 4) 画像を保存
                    cv::imwrite("distField_highlight.png", colorImg);

                    distField_ = distFieldD.clone();
                } catch (const YAML::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "%s\n", e.what());
                }
            }

            void loop() {
                // 並進速度・回転速度を取得
                if (!cmdVel_) {
                    return;
                }
                // RCLCPP_INFO(this->get_logger(), "fkerpofkpoerkfopkarpofjaer");
                
                if (!scan_) {
                    return;
                }          
                
                
                // ロボットから見た座標系
                std::double_t vx_ = cmdVel_->linear.x;
                std::double_t vy_ = cmdVel_->linear.y;
                std::double_t omega_ = cmdVel_->angular.z;
                
                geometry_msgs::msg::Twist delta_;
                delta_.linear.x = vx_*0.1;
                delta_.linear.y = vy_*0.1;
                delta_.angular.z = omega_*0.1;


                updateParticles(delta_);
                printParticlesMakerOnRviz2();
                caculateMeasurementModel(*scan_); // TODO: ポーリングする（この方法でもあたいが変更されることはなさそうだけど）
                estimatePose();
                resampleParticles();
                printTrajectoryOnRviz2();
                // publishScanEndpoints();
                // TODO: printTrajectory
            }

            void getRobotInitialPos() {
                // TODO: split into robot_manager
            }

            void resetParticlesDistribution(geometry_msgs::msg::Pose2D noise) {
                std::double_t wo = 1.0 / (std::double_t)particles_.size();
                for (std::size_t i=0; i<particles_.size(); i++ ) {
                    // TODO: フィールドの中に入っていない場合はリサンプリングする
                    std::double_t x = mclPose_.x + randNormal(noise.x);
                    std::double_t y = mclPose_.y + randNormal(noise.y);
                    // std::double_t x = mclPose_.x;
                    // std::double_t y = mclPose_.y;
                    std::double_t theta = mclPose_.theta;
                    particles_[i].setPose(x, y, theta);
                    particles_[i].setW(wo);
                }
            }

            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y + delta.linear.y;
                std::double_t dy2 = delta.angular.z * delta.angular.z;
                // std::double_t dd2 = 0;
                // std::double_t dy2 = 0;
                // RCLCPP_INFO(this->get_logger(), "odomNoise1=%lf", odomNoise1_);
                for (size_t i = 0; i < this->particles_.size(); i++ ) {
                    std::double_t dx = delta.linear.x + randNormal(
                        odomNoise1_*dd2 + odomNoise2_*dy2
                    );
                    std::double_t dy = delta.linear.y + randNormal(
                        odomNoise1_*dd2 + odomNoise2_*dy2
                    );
                    std::double_t dtheta = delta.angular.z + randNormal(
                        odomNoise3_*dd2 + odomNoise4_*dy2
                    );

                    geometry_msgs::msg::Pose2D pose_ = this->particles_[i].getPose();
                    std::double_t theta_ = pose_.theta;
                    std::double_t x_ = pose_.x + std::cos(theta_)*dx - std::sin(theta_)*dy;
                    std::double_t y_ = pose_.y + std::sin(theta_)*dx + std::cos(theta_)*dy;
                    theta_ += dtheta;
                    particles_[i].setPose(x_, y_, theta_);
                }
            }

            // void updateParticles(std::double_t deltaDist, std::double_t deltaTheta) {
            //     std::double_t dd2 = deltaDist * deltaDist;
            //     std::double_t dtheta2 = deltaTheta * deltaTheta;
            //     std::normal_distribution<std::double_t> distd_(0, odomNoise1_*dd2 + odomNoise2_*dtheta2);
            //     std::normal_distribution<std::double_t> distt_(0, odomNoise3_*dd2 + odomNoise4_*dtheta2);
            //     for (size_t particle_idx_=0; particle_idx_ < particles_.size(); particle_idx_++) {
            //         std::double_t dd = deltaDist + distd_(gen_);
            //         std::double_t dtheta = deltaTheta + distt_(gen_);
            //         std::double_t theta = particles_[particle_idx_].getTheta();
            //         std::double_t x = particles_[particle_idx_].getX() + dd*std::cos(theta);
            //         std::double_t y = particles_[particle_idx_].getY() + dd*std::sin(theta);
            //         theta += dtheta;
            //         particles_[particle_idx_].setPose(x, y, theta);
            //     }
            // }

            sensor_msgs::msg::PointCloud2 layerScan2PointCloud(sensor_msgs::msg::LaserScan scan) {
                // TODO : ノード分ける, 色変えたい
                // sensor_msgs::msg::LaserScan out = scan;
                // out.ranges.clear();
                // out.intensities.clear();
                // out.angle_increment = scan.angle_increment * std::float_t(this->scanStep_);

                // for (std::size_t i=0; i<scan.ranges.size(); i+=scanStep_){
                //     out.ranges.push_back(scan.ranges[i]);
                //     if (scan.intensities.empty()) {
                //         RCLCPP_ERROR(this->get_logger(), "index out of range.");
                //     }
                //     out.intensities.push_back(scan.intensities[i]);
                // }
                
                sensor_msgs::msg::PointCloud2 cloud_in, cloud_out;
                projector_.projectLaser(scan, cloud_in);

                // RCLCPP_INFO(this->get_logger(), "%s", scan.header.frame_id.c_str());

                geometry_msgs::msg::TransformStamped tf = tf_buffer_.lookupTransform(
                    "ldlidar_base",
                    scan.header.frame_id,
                    tf2::TimePointZero  // 最新の transform を使う
                );
                tf2::doTransform(cloud_in, cloud_out, tf);

                return cloud_out;
            }

            void caculateMeasurementModel(sensor_msgs::msg::LaserScan scan) {
                totalLikelihood_ = 0.0;
                std::double_t maxLikelihood = 0.0;

                std::vector<std::vector<double>> likelihood_table;
                likelihood_table.reserve(particleNum_);
                
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t likelihood = 0.0;
                    // 尤度場モデル
                    if (measurementModel_ == MeasurementModel::LikelihoodFieldModel) {
                        likelihood_table.push_back(std::move(caculateLikelihoodFieldModel(particles_[i].getPose(), scan)));
                    }
                    if (i == 0) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = 0;
                    } else if (maxLikelihood < likelihood) {
                        maxLikelihood = likelihood;
                        maxLikelihoodParticleIdx_ = i;
                    }
                    // RCLCPP_INFO(this->get_logger(), "%lf", maxLikelihood);
                }
                // RCLCPP_INFO(this->get_logger(), "%lf", maxLikelihood);
                std::double_t w_sum = 0;
                for(std::size_t i=0; i<likelihood_table.size(); i++ ) {
                    std::double_t w = 0;
                    for (std::size_t j=0; j<likelihood_table.size(); j++ ) {
                        std::double_t loglikefood_sum=0;
                        for (std::size_t k=0; k<likelihood_table[i].size(); k++ ) {
                            // if (std::isnan(likelihood_table[j][k]) || std::isnan(likelihood_table[i][k])) continue;
                            // if (likelihood_table[j][k]<1e-12 || likelihood_table[i][k]<1e-12) continue;
                            loglikefood_sum += std::log(likelihood_table[j][k]/likelihood_table[i][k]);
                            // RCLCPP_INFO(this->get_logger(), "j=%d k=%d %.4f", j, k, likelihood_table[j][k]);
                        }
                        w += std::exp(loglikefood_sum);
                    }
                    w = 1/w;
                    particles_[i].setW(w);
                    w_sum += w*w;
                }
                effectiveSampleSize_ = 1.0 / w_sum;
                
                // // normalize likelihood and caculate valid sample num
                // std::double_t sum = 0.0;
                // for(std::size_t i = 0; i < particles_.size(); i++ ) {
                //     std::double_t w = measurementLikelihoods_[i] / totalLikelihood_;
                //     particles_[i].setW(w);
                //     sum += w*w;
                // }
                // effectiveSampleSize_ = 1.0 / sum;
            }

            std::vector<std::double_t> caculateLikelihoodFieldModel (geometry_msgs::msg::Pose2D pose, sensor_msgs::msg::LaserScan scan) {

                //
                scan_endpoints_.clear();
                //

                std::double_t var = lfmSigma_*lfmSigma_;
                std::double_t normConst = 1.0 / (sqrt(2.0*M_PI*var));
                std::double_t pMax = 1.0 / mapResolution_; // <- mapResolution_で割る必要なくない？
                std::double_t pRand = 1.0 / scan.range_max * mapResolution_;
                std::double_t w = 0.0;

                // sensor_msgs::msg::PointCloud2 pointCloud = layerScan2PointCloud(scan);
                // sensor_msgs::PointCloud2ConstIterator<float> it_x(pointCloud, "x");
                // sensor_msgs::PointCloud2ConstIterator<float> it_y(pointCloud, "y");
                // sensor_msgs::PointCloud2ConstIterator<float> it_z(pointCloud, "z");


                std::vector<double> p_vector;

                for (std::size_t i = 0; i < scan.ranges.size(); i+=scanStep_) {
                    std::double_t r = scan.ranges[i];
                    if (std::isnan(r) || r < scan.range_min || scan.range_max < r) {
                        // p_vector.push_back(zRand_*pRand); // TODO: add pMax
                        p_vector.push_back(zRand_*pRand);
                    }

                    std::double_t theta_lidar;
                    if (is_sim_) {
                        theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment;
                    } else {
                        theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment - 3.0*M_PI/2.0;
                    }

                    double x_odom, y_odom;
                    int u, v;
                    lidarpose2uv(r, theta_lidar, pose, &x_odom, &y_odom, &u, &v);

                    // TODO: isInMap
                    if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_) {
                        // TODO: 尤度場モデルからdをもってくる
                        std::double_t d = (std::double_t)distField_.at<std::double_t>(v, u);
                        std::double_t pHit = normConst * exp(-(d*d)/(2.0*var))*mapResolution_; // 確率密度 <=> 確率の変換は要注意
                        std::double_t p = zHit_*pHit + zRand_*pRand;

                        // RCLCPP_INFO(this->get_logger(), "%.4f %.4f", d, pHit);

                        if (p > 1.0) p = 1.0;
                        p_vector.push_back(p);

                        // RCLCPP_INFO(this->get_logger(), "%d %d %lf %lf", u, v, d, log(p));
                        
                        //
                        // scan_endpoints_.push_back(pt);
                        // particleMarker_->publish(cloud_tf);
                        //
                    } else {
                        // RCLCPP_INFO(this->get_logger(), "fpeafreafkoera");
                        p_vector.push_back(zRand_*pRand);
                        // w += log(zRand_ * pRand); // TODO
                    }
                    // RCLCPP_INFO(this->get_logger(), "##############################");
                    // シミュレーション動かしながら逐次実装していくよ
                }
                // RCLCPP_INFO(this->get_logger(), "####################################");
                // publishScanEndpoints();
                return p_vector;
            }

            void lidarpose2uv(double range, double theta, geometry_msgs::msg::Pose2D pose, double *x_odom, double *y_odom, int *u, int *v) {
                std::double_t x_lidar = range*cos(theta) + 0.033 + 0.005;
                std::double_t y_lidar = range*sin(theta) + 0.013 - 0.013;
                std::double_t x = x_lidar*cos(pose.theta) - y_lidar*sin(pose.theta) + pose.x;
                std::double_t y = x_lidar*sin(pose.theta) + y_lidar*cos(pose.theta) + pose.y;

                *x_odom = x;
                *y_odom = y;

                xy2uv(x, y, u, v);
            }

            void poseCallback (
                const std::shared_ptr<inrof2025_ros_type::srv::BallPose::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::BallPose::Response> response
            ) {
                double ball_min_r = INFINITY;
                double x;
                double y;

                for (std::size_t i = 0; i < scan_->ranges.size(); i+=scanStep_) {
                    std::double_t r = scan_->ranges[i];
                    if (std::isnan(r) || r < scan_->range_min || scan_->range_max < r) continue;

                    std::double_t theta_lidar;
                    if (is_sim_) {
                        theta_lidar = scan_->angle_min + ((std::double_t)(i))*scan_->angle_increment;
                    } else {
                        theta_lidar = scan_->angle_min + ((std::double_t)(i))*scan_->angle_increment - 3.0*M_PI/2.0;
                    }

                    int u, v;
                    double x_odom, y_odom;
                    lidarpose2uv(r, theta_lidar, mclPose_, &x_odom, &y_odom, &u, &v);

                    std::double_t d = (std::double_t)distField_.at<std::double_t>(v, u);
                    if (d > 0.10 && r < ball_min_r) {
                        ball_min_r = d;

                        x = x_odom;
                        y = y_odom;
                    }
                }

                response->x = x;
                response->y = y;
            }

            void publishScanEndpoints()
            {
                sensor_msgs::msg::PointCloud2 cloud;
                cloud.header.stamp    = this->get_clock()->now();
                cloud.header.frame_id = "map";
                cloud.height          = 1;
                cloud.width           = scan_endpoints_.size();
                cloud.is_dense        = false;
                cloud.is_bigendian    = false;

                // xyz フィールドだけを使う
                sensor_msgs::PointCloud2Modifier mod(cloud);
                mod.setPointCloud2FieldsByString(1, "xyz");
                mod.resize(scan_endpoints_.size());

                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

                for (const auto &pt : scan_endpoints_) {
                    *iter_x = pt.x;
                    *iter_y = pt.y;
                    *iter_z = pt.z;  // 0 で OK
                    ++iter_x; ++iter_y; ++iter_z;
                }

                particleMarker_->publish(cloud);
            }

            void estimatePose() {
                std::double_t tmpTheta = mclPose_.theta;
                std::double_t x = 0.0, y = 0.0, theta = 0.0;
                for (size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t w = particles_[i].getW();
                    x += particles_[i].getX() * w;
                    y += particles_[i].getY() * w;
                    std::double_t dTheta = tmpTheta - particles_[i].getTheta();
                    // RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", w, particles_[i].getX(), particles_[i].getY());
                    while (dTheta < -M_PI) dTheta += 2.0*M_PI;
                    while (dTheta > M_PI) dTheta -= 2.0*M_PI;
                    theta += dTheta * w;
                }
                theta = tmpTheta - theta;
                mclPose_.set__x(x);
                mclPose_.set__y(y);
                mclPose_.set__theta(theta);
                pubPose_->publish(mclPose_);

                // TODO: publish odom
                if (!is_sim_) {
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header.stamp = this->get_clock()->now();
                    tf_msg.header.frame_id = "odom";
                    tf_msg.child_frame_id = "base_footprint";

                    tf_msg.transform.translation.x = x;
                    tf_msg.transform.translation.y = y;
                    tf_msg.transform.translation.z = 0.3;

                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, theta);
                    tf_msg.transform.rotation = tf2::toMsg(q);

                    tf_broadcaster_->sendTransform(tf_msg);
                }

                // RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", x, y, theta);
            }

            void resampleParticles(void) {
                double threshold = ((double)particles_.size()) * resampleThreshold_;
                if (effectiveSampleSize_ > threshold) return;

                std::vector<double> wBuffer((int)particles_.size());
                wBuffer[0] = particles_[0].getW();
                for (size_t i=1; i<particles_.size(); i++ ) {
                    wBuffer[i] = particles_[i].getW() + wBuffer[i-1];
                }

                std::vector<Particle> tmpParticles = particles_;
                double wo = 1.0 / (double)particles_.size();
                for (size_t i = 0; i < particles_.size(); i++ ) {
                    double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                    for (size_t j=0; j<particles_.size(); j++ ) {
                        if (darts < wBuffer[j]) {
                            geometry_msgs::msg::Pose2D tmpPos = tmpParticles[j].getPose();
                            particles_[i].setPose(tmpPos.x, tmpPos.y, tmpPos.theta);
                            particles_[i].setW(wo);
                            break;
                        }
                    }
                }
            }

            void printParticlesMakerOnRviz2() {
                sensor_msgs::msg::PointCloud2 cloud_;
                cloud_.header.stamp = this->get_clock()->now();
                cloud_.header.frame_id = "map";
                cloud_.height = 1;
                cloud_.width = particleNum_;
                cloud_.is_dense = false;
                cloud_.is_bigendian = false;

                sensor_msgs::PointCloud2Modifier modifier(cloud_);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(particleNum_);

                sensor_msgs::PointCloud2Iterator<std::float_t> iter_x(cloud_, "x");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_y(cloud_, "y");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_z(cloud_, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_r(cloud_, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_g(cloud_, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_b(cloud_, "b");

                for (const Particle &p: particles_) {
                    *iter_x = p.getX();
                    *iter_y = p.getY();
                    *iter_z = 0;

                    *iter_r = 0;
                    *iter_g = 0;
                    *iter_b = int(p.getW()*255);

                    ++iter_x, ++iter_y, ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                }

                particleMarker_->publish(cloud_);
            }

            void printTrajectoryOnRviz2() {
                geometry_msgs::msg::PoseStamped stamped;
                stamped.header.stamp = this->now();
                stamped.header.frame_id = path_.header.frame_id;
                stamped.pose.position.x = mclPose_.x;
                stamped.pose.position.y = mclPose_.y;
                stamped.pose.position.z = 0.0;
                // RCLCPP_INFO(this->get_logger(), "%lf %lf", mclPose_.x, mclPose_.y);

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, mclPose_.theta);
                stamped.pose.orientation = tf2::toMsg(q);

                path_.poses.push_back(stamped);
                path_.header.stamp = stamped.header.stamp;

                pubPath_->publish(path_);
            }

            // void vel2OdomCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
            //     rclcpp::Time now = this->get_clock()->now();

            //     double dt = (now - last_timestamp_).seconds();
            //     if (dt <= 0.0) {
            //         return;
            //     }
            //     last_timestamp_ = now;
                
            //     std::double_t theta = velOdom_.theta;
            //     std::double_t dx = (msg->linear.x) * dt;
            //     std::double_t dy = (msg->linear.y) * dt;
            //     velOdom_.x += std::cos(theta)*dx - std::sin(theta)*dy;
            //     velOdom_.y += std::sin(theta)*dx + std::cos(theta)*dy;
            //     velOdom_.theta += msg->angular.z*dt;

            //     RCLCPP_INFO(this->get_logger(), "Yaw (rad): %.5f %.5f %.5f %.5f %.5f", yaw_, odom_twist_, msg->angular.z, velOdom_.theta, now.seconds());
            // }
            
            // TODO
            // グリッドマップ上の点へ変換する
            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {
                *u = (std::int32_t)(x / mapResolution_);
                *v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
            }

            // parameter for map
            std::string mapDir_;
            std::double_t mapResolution_;
            std::int32_t mapWidth_, mapHeight_;
            std::vector<std::double_t> mapOrigin_;
            cv::Mat mapImg_;
            cv::Mat distField_;

            // particle settings
            int particleNum_;
            // 絶対座標
            std::vector<Particle> particles_;
            geometry_msgs::msg::Pose2D mclPose_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;

            // likelihood
            int maxLikelihoodParticleIdx_;
            std::double_t totalLikelihood_;
            std::double_t averageLikelihood_;
            std::vector<std::double_t> measurementLikelihoods_;
            std::vector<probability> pro_;

            std::double_t effectiveSampleSize_;
            std::double_t resampleThreshold_;

            // model for mesurement
            mcl::MeasurementModel measurementModel_;
            std::int32_t scanStep_;

            // parameter for measurement model
            std::double_t zHit_, zShort_, zMax_, zRand_;
            std::double_t lfmSigma_;

            // parameter for lidar relitive position
            // lidarの絶対座標はtfが変換してpublishしてくれている可能性があるので，もしかしたらいらないかも
            std::double_t x_lidar_, y_lidar_, theta_lidar_;

            // noize when updateing particle
            std::double_t odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_;
            std::mt19937 gen_;

            std::string base_footprint_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            
            geometry_msgs::msg::Twist::SharedPtr cmdVel_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subCmdVel_;

            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLayerScan_;

            bool is_sim_;

            // print trajectory on rviz
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
            nav_msgs::msg::Path path_;
            
            rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pubPose_;

            // cmd_velのみから現在のodometryを計算する
            // last_time_に前回差分を取得したときの時刻
            // last_odom_に前回のodometryを保存
            rclcpp::Time last_timestamp_;
            geometry_msgs::msg::Pose2D velOdom_;
            geometry_msgs::msg::Pose2D last_odom_;

            // ball
            rclcpp::Service<inrof2025_ros_type::srv::BallPose>::SharedPtr srvBallPose_;

            // TODO: delete
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
            std::float_t iter_;
            std::double_t yaw_;
            std::double_t odom_twist_;
            std::vector<geometry_msgs::msg::Point> scan_endpoints_;
            tf2_ros::TransformListener tf_listener_;
            tf2_ros::Buffer tf_buffer_;
            laser_geometry::LaserProjection projector_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
            nav_msgs::msg::Odometry::SharedPtr odom_;

            // TODO: delete
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            void timer_callback() {
                RCLCPP_INFO(this->get_logger(), "In timer loop");
            }
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mcl::MCL>());
    rclcpp::shutdown();
    return 0;
}