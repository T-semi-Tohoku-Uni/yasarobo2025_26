#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>
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
    class Particle{
        public:
            //保持しているデータを外部から持ってくる関数群
            const std::double_t& getX() const& {return pose_.x;}
            const std::double_t& getY() const& {return pose_.y;}
            const std::double_t& getTheta() const&{return pose_.theta;}
            const geometry_msgs::msg::Pose2D& getPose() const&{return pose_;}
            const std::double_t& getW() const& {return w_;}
            //保持しているデータを外部から書き換える関数群
            void setPose(std::double_t x, std::double_t y, std::double_t theta) {
                pose_.set__x(x);
                pose_.set__y(y);
                pose_.set__theta(theta);
            }
            void setW(std::double_t w) {
                w_ = w;
            }
        private:
            //実際にデータを保持している部分
            geometry_msgs::msg::Pose2D pose_;//ROS2の標準メッセージ型
            std::double_t w_;//尤度
    };

    
    class Obstaclespointclowd : public sensor_msgs::msg::LaserScan{
        public:
            const std::double_t& getobstaclesW() const& {return obstaclesw_;}

            void setobstacles(std::double_t w){
                obstaclesw_ = w;
            }
        private:
        std::double_t obstaclesw_;
    };



    enum class MeasurementModel { 
        //使う観測モデルの選択　多分拡張性を持たせている
        LikelihoodFieldModel,
        ClassConditionalMeasurementModel 
    };
    
    class MCL: public rclcpp::Node {
        public:
            //:（コロン）から {（波括弧）までの部分がメンバ初期化子リスト。
            explicit MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("mcl_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
                //nodeの名前と型の定義デフォルトの値
                this->declare_parameter<std::int32_t>("particleNum", 100);
                this->declare_parameter<std::float_t>("initial_x", 0.25);
                this->declare_parameter<std::float_t>("initial_y", 0.25);
                this->declare_parameter<std::float_t>("initial_theta", M_PI/2);
                this->declare_parameter<std::float_t>("resampleThreshold", 0.5);
                this->declare_parameter<std::float_t>("odomNoise1", 1.0);
                this->declare_parameter<std::float_t>("odomNoise2", 1.0);
                this->declare_parameter<std::float_t>("odomNoise3", 1.0);
                this->declare_parameter<std::float_t>("odomNoise4", 1.0);
                this->declare_parameter<std::float_t>("mapResolution", 0.01);
                this->declare_parameter<std::string>("mapDir", "src/yasarobo2025_26/map/");
                this->declare_parameter<std::int32_t>("scanStep", 50);
                this->declare_parameter<std::double_t>("lfmSigma", 0.03);
                this->declare_parameter<std::double_t>("zHit", 1.0);
                this->declare_parameter<std::double_t>("zMax", 0.0);
                this->declare_parameter<std::double_t>("zRand", 1.0);
                this->declare_parameter<std::double_t>("unknownLambda",0.1);
                this->declare_parameter<std::double_t>("obstacleThreshold", 0.8); // デフォルトの閾値を 0.8 に設定

                particleNum_ = this->get_parameter("particleNum").as_int();
                double initial_x = this->get_parameter("initial_x").as_double();
                double initial_y = this->get_parameter("initial_y").as_double();
                double initial_theta = this->get_parameter("initial_theta").as_double();
                this->resampleThreshold_ = this->get_parameter("resampleThreshold").as_double();
                this->odomNoise1_ = this->get_parameter("odomNoise1").as_double();
                this->odomNoise2_ = this->get_parameter("odomNoise2").as_double();
                this->odomNoise3_ = this->get_parameter("odomNoise3").as_double();
                this->odomNoise4_ = this->get_parameter("odomNoise4").as_double();
                this->mapResolution_ = this->get_parameter("mapResolution").as_double();
                this->mapDir_ = this->get_parameter("mapDir").as_string();
                this->scanStep_ = this->get_parameter("scanStep").as_int();
                this->lfmSigma_ = this->get_parameter("lfmSigma").as_double();
                this->zHit_ = this->get_parameter("zHit").as_double();
                this->zMax_ = this->get_parameter("zMax").as_double();
                this->zRand_ = this->get_parameter("zRand").as_double();
                this->unknownLambda_ = this->get_parameter("unknownLambda").as_double();
                this->obstacleThreshold_ = this->get_parameter("obstacleThreshold").as_double();

                particles_.resize(particleNum_);

                measurementLikelihoods_.resize(particleNum_);

                //ロボットの初期位置を設定
                geometry_msgs::msg::Pose2D pose;
                pose.set__x(initial_x);//もっと別のところでやりたいね
                pose.set__y(initial_y);
                pose.set__theta(initial_theta);
                //初期位置をMcl全体がもつ変数に格納している
                setMCLPose(pose);
                velOdom_.set__x(initial_x);
                velOdom_.set__y(initial_y);
                velOdom_.set__theta(initial_theta);

                //パーティクルの初期化
                geometry_msgs::msg::Pose2D initialNoise;
                auto cloud_qos = rclcpp::SensorDataQoS();
                particleMarker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", cloud_qos);
                obstaclesParticleMarker_= this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstaclescloud", cloud_qos);
                initialNoise.set__x(0.07); // var of x
                initialNoise.set__y(0.07); // var of y
                initialNoise.set__theta(M_PI/180.0); // var of theta
                resetParticlesDistribution(initialNoise);
                printParticlesMakerOnRviz2();

                //観測モデルの選択
                measurementModel_ = MeasurementModel::ClassConditionalMeasurementModel;
                //measurementModel_ = MeasurementModel::LikelihoodFieldModel;
                
                MCL::readMap();
                
                last_timestamp_ = this->get_clock()->now();

                //何件のデータを保持しておくか
                rclcpp::QoS cmdVelQos(rclcpp::KeepLast(10));
                //データの更新
                subCmdVel_ = create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel_feedback", cmdVelQos, std::bind(&MCL::cmdVelCallback, this, std::placeholders::_1)
                );
                auto laserScanQos = rclcpp::SensorDataQoS();
                filteredScanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", laserScanQos);
                subLayerScan_ = create_subscription<sensor_msgs::msg::LaserScan>(
                    "/ldlidar_node/scan", laserScanQos, std::bind(&MCL::laserScanCallback, this, std::placeholders::_1)
                );

                //情報発信の設定
                pubPath_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
                path_.header.frame_id = "map";
                pubPose_ = create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);

                //なにこれ
                const char *sim = std::getenv("WITH_SIM");
                // RCLCPP_INFO(this->get_logger(), "freofkprekfore");
                if (!sim || std::string(sim) != "1") {
                    is_sim_ = false;
                } else {
                    is_sim_ = true;
                }

                //コントラクタの終了
                iter_=0;
                timer_ = rclcpp::create_timer(
                    this,
                    this->get_clock(),
                    100ms,
                    std::bind(&MCL::loop, this)
                  );
                RCLCPP_INFO(this->get_logger(), "Success initialize");
            }
        private:
            //位置の更新
            void setMCLPose(geometry_msgs::msg::Pose2D pose) { mclPose_=pose; }
            //ガウス分布のノイズ用
            std::double_t randNormal(double n) { return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); }

            //データの更新
            void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
                scan_ = scan;
            }
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
                cmdVel_=msg;
            }
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_=msg;
            }

            //mapの読み込み　変えよう
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


                updateParticles(delta_);//移動量に応じてロボットの自己位置が更新される
                printParticlesMakerOnRviz2();//パーティクルをRviz上に表示
                calculateMeasurementModel(*scan_); // lidarからの情報を受け取りパーティクルの尤度を計算
                estimatePose();//全パーティクルの重みつき計算をして最終的な自己位置
                resampleParticles();//パーティクルの偏りを見る
                printTrajectoryOnRviz2();
                printObstaclesParticlesOnRviz2(*scan_);
                publishFilteredScan(*scan_);
            }

            //パーティクルを再度配置する
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

            //移動量からノイズを含む自己位置を推定する
            void updateParticles(geometry_msgs::msg::Twist delta) {
                std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y;
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

            //各パーティクルの尤度を計算する
            void calculateMeasurementModel(sensor_msgs::msg::LaserScan scan) {
                //最大尤度となるパーティクルのインデックスを取得
                totalLikelihood_ = 0.0;
                std::double_t maxLikelihood = 0.0;

                std::vector<std::vector<double>> likelihood_table;
                likelihood_table.reserve(particleNum_);
                
                //各パーティクルの尤度を計算する、最大尤度のインデックスを探す
                for (std::size_t i = 0; i < particles_.size(); i++ ) {
                    std::double_t likelihood = 0.0;
                    // 尤度場モデル
                    if (measurementModel_ == MeasurementModel::LikelihoodFieldModel) {
                        likelihood_table.push_back(std::move(caculateLikelihoodFieldModel(particles_[i].getPose(), scan)));
                    }else if(measurementModel_ == MeasurementModel::ClassConditionalMeasurementModel){
                        likelihood_table.push_back(std::move(calculateClassConditionalMeasurementModel(particles_[i].getPose(), scan)));
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

                if (measurementModel_ == MeasurementModel::ClassConditionalMeasurementModel)calculateUnknownScanProbs(particles_[maxLikelihoodParticleIdx_].getPose(),scan);


                //各パーティクルの最終的な重みを決定
                std::double_t w_sum = 0;
                for(std::size_t i=0; i<likelihood_table.size(); i++ ) {
                    std::double_t w = 0;
                    for (std::size_t j=0; j<likelihood_table.size(); j++ ) {
                        std::double_t loglikefood_sum=0;
                        for (std::size_t k=0; k<likelihood_table[i].size(); k++ ) {
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
                
            }

            //尤度場モデルによる観測モデルの実装
            std::vector<std::double_t> caculateLikelihoodFieldModel(geometry_msgs::msg::Pose2D pose,sensor_msgs::msg::LaserScan scan){
                scan_endpoints_.clear();

                std::double_t var = lfmSigma_*lfmSigma_;
                std::double_t normConst = 1.0 / (sqrt(2.0*M_PI*var));
                std::double_t pMax = 1.0 / mapResolution_; // <- mapResolution_で割る必要なくない？
                std::double_t pRand = 1.0 / scan.range_max * mapResolution_;
                std::double_t w = 0.0;

                std::vector<double> p_vector;

                for (std::size_t i = 0; i < scan.ranges.size(); i+=scanStep_){
                    std::double_t r = scan.ranges[i];
                    //無効なスキャンの処理
                    if (std::isnan(r) || r < scan.range_min || scan.range_max < r) {
                        //ランダムノイズを入れる
                        p_vector.push_back(zRand_*pRand);
                    }
                    //実機かシミュか
                    std::double_t theta_lidar;
                    if (is_sim_) {
                        theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment;
                    } else {
                        theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment - 3.0*M_PI/2.0;
                    }

                    double x_odom, y_odom;
                    int u, v;
                    lidarpose2uv(r, theta_lidar, pose, &x_odom, &y_odom, &u, &v);

                    if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_) {
                        // TODO: 尤度場モデルからdをもってくる
                        std::double_t d = (std::double_t)distField_.at<std::double_t>(v, u);
                        std::double_t pHit = normConst * exp(-(d*d)/(2.0*var))*mapResolution_; // 確率密度 <=> 確率の変換は要注意
                        std::double_t p = zHit_*pHit + zRand_*pRand;

                        if (p > 1.0) p = 1.0;
                        p_vector.push_back(p);
                    } else {
                        p_vector.push_back(zRand_*pRand);
                    }

                }

                
                return p_vector;
            }

            //クラス条件付き観測モデルを用いた
            std::vector<std::double_t> calculateClassConditionalMeasurementModel(geometry_msgs::msg::Pose2D pose, sensor_msgs::msg::LaserScan scan){
                scan_endpoints_.clear();

                std::double_t var = lfmSigma_*lfmSigma_;
                std::double_t normConst = 1.0 /(sqrt(2.0 * M_PI*var));
                std::double_t range_max = scan.range_max;
                std::double_t unknownConst = 1.0/(1.0 - exp(-unknownLambda_ * range_max));
                std::double_t pMax = 1.0/mapResolution_;
                std::double_t pRand = 1.0/ range_max * mapResolution_;
                std::double_t pKnownPrior = 0.5;//存在する確率
                std::double_t pUnknownPrior = 1.0 - pKnownPrior;
                std::double_t w = 0.0;
                std::vector<double_t> p_vector;

                for(std::size_t i = 0; i < scan.ranges.size(); i+=scanStep_){
                    // スキャン点が地図に存在するかしないかの確率
                    // この和がパーティクルの尤度となる

                    std::double_t pKnown, pUnknown;
                    std::double_t r = scan.ranges[i];
                    if (std::isnan(r) || r < scan.range_min || scan.range_max < r) {
                        pKnown = (zMax_ * pMax + zRand_ * pRand) * pKnownPrior;
                        pUnknown = (unknownConst * unknownLambda_*exp(-unknownLambda_ * scan.range_max) * mapResolution_) * pUnknownPrior;
                    }else {
                        std::double_t a = scan.angle_min + (std::double_t)i * scan.angle_increment + pose.theta;
                        std::double_t x = r * cos(a) + pose.x;
                        std::double_t y = r * sin(a) + pose.y;

                        std::int32_t u,v;
                        xy2uv(x,y,&u,&v);
                        if(0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_){
                            std::double_t d = (std::double_t)distField_.at<std::double_t>(v,u);
                            std::double_t pHit = normConst * exp(-(d * d) / (2.0 * var)) * mapResolution_;
                            pKnown = (zHit_ * pHit + zRand_ * pRand) * pKnownPrior;
                        }else {
                            pKnown = (zRand_ * pRand) * pKnownPrior;
                        }
                        pUnknown = (unknownConst * unknownLambda_ * exp(-unknownLambda_ * r) * mapResolution_) * pUnknownPrior;


                    }
                    std::double_t p = pKnown + pUnknown;
                    if(p > 1.0)p = 1.0;
                    //RCLCPP_INFO(this->get_logger(),"%.4f %.4f",pUnknown,pKnown);
                    //RCLCPP_INFO(this->get_logger(),"%.4f",pUnknown);
                    p_vector.push_back(p);
                     

                }
            //RCLCPP_INFO(this->get_logger(),"ookokokkokokoookok"); 
              
            return p_vector;

            }

            void calculateUnknownScanProbs(geometry_msgs::msg::Pose2D pose,sensor_msgs::msg::LaserScan scan){
                //unknownScanProbs_.resize(scan.ranges.size(), 0.0);
                obstacles_.resize(scan.ranges.size());
                std::double_t var = lfmSigma_*lfmSigma_;
                std::double_t normConst = 1.0 /(sqrt(2.0 * M_PI*var));
                std::double_t range_max = scan.range_max;
                std::double_t unknownConst = 1.0/(1.0 - exp(-unknownLambda_ * range_max));
                std::double_t pMax = 1.0/mapResolution_;
                std::double_t pRand = 1.0/ range_max * mapResolution_;
                std::double_t pKnownPrior = 0.5;//存在する確率
                std::double_t pUnknownPrior = 1.0 - pKnownPrior;
                for (std::size_t i = 0; i < scan.ranges.size(); i++){
                    std::double_t pKnown, pUnknown;
                    std::double_t r = scan.ranges[i];
                    if (r < scan.range_min || scan.range_max < r) {
                        pKnown = (zMax_ * pMax + zRand_ * pRand) * pKnownPrior;
                        pUnknown = (unknownConst * unknownLambda_*exp(-unknownLambda_ * scan.range_max) * mapResolution_) * pUnknownPrior;
                    }else {
                        std::double_t a = scan.angle_min + (std::double_t)i * scan.angle_increment + pose.theta;
                        std::double_t x = r * cos(a) + pose.x;
                        std::double_t y = r * sin(a) + pose.y;

                        std::int32_t u,v;
                        xy2uv(x,y,&u,&v);
                        if(0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_){
                            std::double_t d = (std::double_t)distField_.at<std::double_t>(v,u);
                            std::double_t pHit = normConst * exp(-(d * d) / (2.0 * var)) * mapResolution_;
                            pKnown = (zHit_ * pHit + zRand_ * pRand) * pKnownPrior;
                        }else {
                            pKnown = (zRand_ * pRand) * pKnownPrior;
                        }
                        pUnknown = (unknownConst * unknownLambda_ * exp(-unknownLambda_ * r) * mapResolution_) * pUnknownPrior;
                    }
                    std::double_t p = pUnknown / (pKnown + pUnknown);
                    //RCLCPP_INFO(this->get_logger(),"%.4f",p);
                    obstacles_[i].setobstacles(pUnknown / (pKnown + pUnknown));
                }

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

            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {
                *u = (std::int32_t)(x / mapResolution_);
                *v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
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

                //RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", x, y, theta);
            }

            //リサンプリング
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

            void printObstaclesParticlesOnRviz2(sensor_msgs::msg::LaserScan scan){
                sensor_msgs::msg::PointCloud2 obstaclesCloud_;
                obstaclesparticleNum_ = scan.ranges.size();
                obstaclesCloud_.header.stamp = this->get_clock()->now();
                obstaclesCloud_.header.frame_id = "map";
                obstaclesCloud_.height = 1;
                obstaclesCloud_.width = obstaclesparticleNum_;
                obstaclesCloud_.is_dense = false;
                obstaclesCloud_.is_bigendian  = false;

                sensor_msgs::PointCloud2Modifier modifier(obstaclesCloud_);
                modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
                modifier.resize(obstaclesparticleNum_);

                sensor_msgs::PointCloud2Iterator<std::float_t> iter_x(obstaclesCloud_, "x");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_y(obstaclesCloud_, "y");
                sensor_msgs::PointCloud2Iterator<std::float_t> iter_z(obstaclesCloud_, "z");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_r(obstaclesCloud_, "r");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_g(obstaclesCloud_, "g");
                sensor_msgs::PointCloud2Iterator<uint8_t>  iter_b(obstaclesCloud_, "b");

                int i = 0;
                
                // 基準となるロボットのポーズ (最終推定値)
                geometry_msgs::msg::Pose2D estimated_pose = mclPose_;

                for (const Obstaclespointclowd &o: obstacles_) {
                    std::double_t range = scan.ranges[i];

                    // 無効なスキャン値のチェック
                    if (std::isnan(range) || range < scan.range_min || range > scan.range_max) {
                        *iter_x = 0; // 無効な点は原点にでも置く
                        *iter_y = 0;
                        *iter_z = 0;
                    } else {
                        // 1. LIDARセンサー座標系での角度を計算
                        std::double_t theta_lidar;
                        if (is_sim_) {
                            theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment;
                        } else {
                            // caculateLikelihoodFieldModelと同じ補正を適用
                            theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment - 3.0*M_PI/2.0;
                        }

                        // 2. lidarpose2uv を使って、LIDARオフセットとロボットポーズを考慮した
                        //    map座標系での X, Y を計算する
                        double x_world, y_world;
                        int u_map, v_map; // ピクセル座標 (ここでは使わない)
                        
                        lidarpose2uv(range, theta_lidar, estimated_pose, &x_world, &y_world, &u_map, &v_map);

                        // 3. 計算された正しいmap座標をポイントクラウドに設定
                        *iter_x = x_world;
                        *iter_y = y_world;
                        *iter_z = 0; // Zは0
                    }

                    // 障害物らしさ (Weight) に基づいて色を決定
                    *iter_r = 0;
                    *iter_g = int(o.getobstaclesW()*255);
                    *iter_b = 0;

                    ++iter_x, ++iter_y, ++iter_z;
                    ++iter_r; ++iter_g; ++iter_b;
                    ++i;
                    
                    // ↓ このログはデバッグ中にコンソールを埋め尽くす可能性があるので注意
                    // std::uint8_t w = int(o.getobstaclesW()*255);
                    // RCLCPP_INFO(this->get_logger(),"%d",w);
                }

                obstaclesParticleMarker_->publish(obstaclesCloud_);

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

            //フィルタリングした後のデータを送る
            void publishFilteredScan(sensor_msgs::msg::LaserScan scan) {
                
                // 1. 新しいLaserScanメッセージ(のユニークポインタ)を作成
                auto filtered_scan = std::make_unique<sensor_msgs::msg::LaserScan>();

                // 2. 元スキャンのメタデータをすべてコピー
                // (header, 角度, 時間, 範囲の最小/最大)
                filtered_scan->header = scan.header;
                filtered_scan->angle_min = scan.angle_min;
                filtered_scan->angle_max = scan.angle_max;
                filtered_scan->angle_increment = scan.angle_increment;
                filtered_scan->time_increment = scan.time_increment;
                filtered_scan->scan_time = scan.scan_time;
                filtered_scan->range_min = scan.range_min;
                filtered_scan->range_max = scan.range_max;

                // 3. ranges 配列のサイズを元スキャンに合わせる
                filtered_scan->ranges.resize(scan.ranges.size());

                // 4. 全ての障害物確率をチェック
                for (std::size_t i = 0; i < obstacles_.size(); ++i) {
                    
                    const std::double_t original_range = scan.ranges[i];

                    // チェック1: 元のレンジが有効か？
                    bool is_valid_range = !std::isnan(original_range) && 
                                           original_range >= scan.range_min && 
                                           original_range <= scan.range_max;
                    
                    // チェック2: 障害物としての閾値を超えているか？
                    bool is_obstacle = obstacles_[i].getobstaclesW() > obstacleThreshold_;

                    // 有効なレンジで、かつ障害物閾値を超えている点のみ、元の距離をコピー
                    if (is_valid_range && is_obstacle) {
                        filtered_scan->ranges[i] = original_range;
                    } 
                    // それ以外の点（障害物でない、または元々無効な点）は無効値にする
                    else {
                        filtered_scan->ranges[i] = std::numeric_limits<double>::infinity();
                    }
                }

                // 5. フィルタリングされたLaserScanメッセージをパブリッシュ
                filteredScanPub_->publish(std::move(filtered_scan));
            }






            //map用のパラメータ
            std::string mapDir_;
            std::double_t mapResolution_;
            std::int32_t mapWidth_, mapHeight_;
            std::vector<std::double_t> mapOrigin_;
            cv::Mat mapImg_;
            cv::Mat distField_;

            // パーティクルの数
            int particleNum_;//パーティクルの総数
            int obstaclesparticleNum_;
            // 絶対座標
            std::vector<Particle> particles_;

            geometry_msgs::msg::Pose2D mclPose_;//最終的な自己位置
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particleMarker_;//ポイントクラウド

            std::vector<Obstaclespointclowd> obstacles_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstaclesParticleMarker_;
            std::double_t unknownLambda_;

            // フィルタリングした後のlidardata
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filteredScanPub_;
            std::double_t obstacleThreshold_;


            // likelihood
            int maxLikelihoodParticleIdx_;//最も高い尤度をもつパーティクルを保持
            std::double_t totalLikelihood_;//全パーティクルの尤度の合計
            std::double_t averageLikelihood_;//平均尤度を保持するための変数
            std::vector<std::double_t> measurementLikelihoods_;//各パーティクルの尤度
            //std::vector<probability> pro_;//使っていない

            // 各スキャン点が未知障害物である確率
            // 動的障害物の棄却使用時，またはクラス条件付き観測モデル使用時のみ利用可能
            std::vector<double> unknownScanProbs_;

            std::double_t effectiveSampleSize_;//パーティクルの偏り具合
            std::double_t resampleThreshold_;//リサピリングの閾値

            // model for mesurement
            mcl::MeasurementModel measurementModel_;
            std::int32_t scanStep_;//

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