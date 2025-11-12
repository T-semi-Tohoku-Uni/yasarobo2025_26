#include "yasarobo2025_26/mcl_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <optional>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals; 

namespace yasarobo2025_26 {
    void Pose::setYaw(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        this->pose.orientation = tf2::toMsg(q);
    }

    double Pose::getYaw() {
        tf2::Quaternion q;
        tf2::fromMsg(this->pose.orientation, q);
        return tf2::getYaw(q);
    }

    geometry_msgs::msg::PoseStamped Pose::toPoseStamped() {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = this->header;
        ps.pose = this->pose;
        return ps;
    }

    geometry_msgs::msg::Pose2D Pose::toPose2D() {
        geometry_msgs::msg::Pose2D pose_2d;
        pose_2d.x = this->pose.position.x;
        pose_2d.y = this->pose.position.y;
        pose_2d.theta = getYaw();

        return pose_2d;
    }
}

namespace yasarobo2025_26 {
    Field::Field(std::string map_dir) {
        YAML::Node lconf = YAML::LoadFile(map_dir + "map.yaml");
        mapResolution_ = lconf["resolution"].as<std::double_t>();

        std::string imgFile = map_dir + "map.pgm";
        mapImg_ = cv::imread(imgFile, 0);
        mapWidth_ = mapImg_.cols;
        mapHeight_ = mapImg_.rows;

        for (int v = 0; v < mapHeight_; v++ ) {
            for (int u = 0; u < mapWidth_; u++ ) {
                uchar val = mapImg_.at<uchar>(v, u);
                if (val >= 239) {
                    mapImg_.at<uchar>(v, u) = 1;
                } else {
                    mapImg_.at<uchar>(v, u) = 0;
                }
            }
        }

        cv::Mat distFieldF(mapHeight_, mapWidth_, CV_32FC1);
        cv::Mat distFieldD(mapHeight_, mapWidth_, CV_64FC1);
        cv::distanceTransform(mapImg_, distFieldF, cv::DIST_L2, 5);

        for (int v = 0; v < mapHeight_; v++ ) {
            for (int u = 0; u < mapWidth_; u++ ) {
                std::float_t d = distFieldF.at<std::float_t>(v, u);
                distFieldD.at<std::double_t>(v, u) = (std::double_t)d * mapResolution_;
            }
        }

        distance_field_ = distFieldD.clone();

        win_u_min = 60;
        win_u_max = 60 + 28 + 1;
        win_v_min = 90;
        win_v_max = 90 + 90;
    }

    double Field::getMapResolution() {
        return this->mapResolution_;
    }

    void Field::xy2uv(double x, double y, int& u, int& v) {
        u = (std::int32_t)(x / mapResolution_);
        v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
    }

    bool Field::isOnField(int u, int v) {
        if (u < 0 || v < 0 || u >= mapWidth_ || v >= mapHeight_) return false;
        if (mapImg_.at<uchar>(v, u) == 0) return false;
        // return true;
        return true;
    }

    int Field::computeOutCode(int u, int v) {
        int code = 0;
        if (v > win_v_max) code |= 8;
        if (v < win_v_min) code |= 4;
        if (u > win_u_max) code |= 2;
        if (u < win_u_min) code |= 1;
        return code;
    }

    double Field::getWallDistance(int u_pose, int v_pose, int u_layser, int v_layser) {
        int code1 = Field::computeOutCode(u_pose, v_pose);
        int code2 = Field::computeOutCode(u_layser, v_layser);

        if ((code1 | code2) == 0) return static_cast<double>(distance_field_.at<double>(v_layser, u_layser)); // 内部
        if (code1 & code2) return static_cast<double>(distance_field_.at<double>(v_layser, u_layser));  // 交わっていない

        double x_pose = static_cast<double>(u_pose);
        double y_pose = static_cast<double>(v_pose);
        double x_layser = static_cast<double>(u_layser);
        double y_layser = static_cast<double>(v_layser);

        double x_min = static_cast<double>(win_u_min);
        double x_max = static_cast<double>(win_u_max);
        double y_min = static_cast<double>(win_v_min);
        double y_max = static_cast<double>(win_v_max);
        
        // 交わっている場合
        if (code1 & 0b1000) { // x top
            x_pose = x_pose + ((x_layser-x_pose)/(y_layser-y_pose))*(y_max-y_pose);
            y_pose = y_max;
        } else if (code1 & 0b0100) { // x bottom
            x_pose = x_pose + ((x_layser-x_pose)/(y_layser-y_pose))*(y_min-y_pose);
            y_pose = y_min;
        } else if (code1 & 0b0010) { // y top
            y_pose = y_pose + ((y_layser-y_pose)/(x_layser-x_pose))*(x_max-x_pose);
            x_pose = x_max;
        } else if (code1 & 0b0001) { // y bottom
            y_pose = y_pose + ((y_layser-y_pose)/(x_layser-x_pose))*(x_min-x_pose);
            x_pose = x_min;
        }

        return std::hypot(x_pose-x_layser, y_pose-y_layser)*mapResolution_;
    }
}

namespace yasarobo2025_26 {
    MCL::MCL(const rclcpp::NodeOptions & options): 
        Node("mcl", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) 
    {
        this->declare_parameter<std::int32_t>("particleNum", 100);
        this->declare_parameter<std::float_t>("initial_x", 0.25);
        this->declare_parameter<std::float_t>("initial_y", 0.25);
        this->declare_parameter<std::float_t>("initial_theta", M_PI/2);
        this->declare_parameter<std::float_t>("odomNoise1", 1.0);
        this->declare_parameter<std::float_t>("odomNoise2", 1.0);
        this->declare_parameter<std::float_t>("odomNoise3", 1.0);
        this->declare_parameter<std::float_t>("odomNoise4", 1.0);
        this->declare_parameter<std::double_t>("lfmSigma", 0.03);
        this->declare_parameter<std::string>("mapDir", "src/yasarobo2025_26/map/");
        this->declare_parameter<std::int32_t>("scanStep", 50);
        this->declare_parameter<std::double_t>("zRand", 1.0);
        this->declare_parameter<std::double_t>("zHit", 1.0);
        this->declare_parameter<std::float_t>("resampleThreshold", 0.5);

        int particle_num = this->get_parameter("particleNum").as_int();
        double initial_x = this->get_parameter("initial_x").as_double();
        double initial_y = this->get_parameter("initial_y").as_double();
        double initial_theta = this->get_parameter("initial_theta").as_double();
        std::string map_dir = this->get_parameter("mapDir").as_string();

        this->odomNoise1_ = this->get_parameter("odomNoise1").as_double();
        this->odomNoise2_ = this->get_parameter("odomNoise2").as_double();
        this->odomNoise3_ = this->get_parameter("odomNoise3").as_double();
        this->odomNoise4_ = this->get_parameter("odomNoise4").as_double();
        this->lfmSigma_ = this->get_parameter("lfmSigma").as_double();
        this->scan_step_ = this->get_parameter("scanStep").as_int();
        this->z_rand_ = this->get_parameter("zRand").as_double();
        this->z_hit_ = this->get_parameter("zHit").as_double();
        this->resample_threshold_ = this->get_parameter("resampleThreshold").as_double();

        // initialize publisher
        rclcpp::SensorDataQoS cloud_qos = rclcpp::SensorDataQoS();
        pub_particle_marker_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", cloud_qos);

        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("trajectory", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);

        // initialize subscriber
        rclcpp::QoS cmd_vel_qos(rclcpp::KeepLast(10));
        // rclcpp::SensorDataQoS cmd_vel_qos = rclcpp::SensorDataQoS();
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_feedback", cmd_vel_qos, std::bind(&MCL::cmdVelCallback, this, std::placeholders::_1)
        );
        rclcpp::SensorDataQoS laserScanQos = rclcpp::SensorDataQoS();
        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/ldlidar_node/scan", laserScanQos, std::bind(&MCL::laserScanCallback, this, std::placeholders::_1)
        );

        // initialize tf
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // initialize field
        field_ = yasarobo2025_26::Field(map_dir);

        // initialize particle
        particles_.resize(particle_num);

        // initialize mcl_pose_
        mcl_pose_.pose.position.x = initial_x;
        mcl_pose_.pose.position.y = initial_y;
        mcl_pose_.setYaw(initial_theta);

        resetParticlesDistribution(0.07, 0.07, M_PI/180.0); // TODO: set parameter
        printParticle();

        // TODO measurement model definition

        // initialize env
        const char *sim = std::getenv("WITH_SIM");
        if (!sim || std::string(sim) != "1") is_sim_ = false;
        else is_sim_ = true;

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            100ms,
            std::bind(&MCL::loop, this)
        );
    };

    void MCL::loop() {
        if (!cmd_vel_) {
            RCLCPP_WARN(this->get_logger(), "cmd_vel is None");
            return;
        } 
        if (!scan_) {
            RCLCPP_WARN(this->get_logger(), "scan is None");
            return;
        }


        updateParticles();
        printParticle();
        caculateMeasurementModel();
        estimatePose();
        resampleParticles();
        printPath();
    }

    void MCL::estimatePose() {
        double cur_theta = mcl_pose_.getYaw();
        double x=0.0, y=0.0, theta=0.0;
        for (size_t i=0; i<particles_.size(); i++ ) {
            double w = particles_[i].w;
            x += particles_[i].pose.position.x * w;
            y += particles_[i].pose.position.y * w;
            double dtheta = cur_theta - particles_[i].getYaw();
            // TODO 必要ない
            while (dtheta < -M_PI) dtheta += 2.0*M_PI;
            while (dtheta > M_PI) dtheta -= 2.0*M_PI;
            theta += dtheta * w;
        }

        theta = cur_theta - theta;
        // normalize theta
        while (theta < -M_PI) theta += 2.0*M_PI;
        while (theta > M_PI) theta -= 2.0*M_PI;
        mcl_pose_.pose.position.x = x;
        mcl_pose_.pose.position.y = y;
        mcl_pose_.setYaw(theta);
        pub_pose_->publish(mcl_pose_.toPose2D());

        // publish base_footprint -> odom when real world
        if (!is_sim_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_footprint";

            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0.3;

            tf_msg.transform.rotation = mcl_pose_.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }

    }

    void MCL::resampleParticles() {
        double th = static_cast<double>(particles_.size()) * resample_threshold_;
        if (effective_sample_size_ > th) return;

        std::vector<double> w_buffer(static_cast<int>(particles_.size()));
        w_buffer[0] = particles_[0].w;
        for (size_t i=1; i<particles_.size(); i++ ) {
            w_buffer[i] = particles_[i].w + w_buffer[i-1];
        }

        std::vector<yasarobo2025_26::Particle> tmp_particles = particles_;
        double wo = 1.0 / static_cast<double>(particles_.size());
        for (size_t i=0; i<particles_.size(); i++ ) {
            double darts = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX) + 1.0);
            for (size_t j=0; j<particles_.size(); j++ ) {
                if (darts < w_buffer[j]) {
                    particles_[i].pose.position.x = tmp_particles[j].pose.position.x;
                    particles_[i].pose.position.y = tmp_particles[j].pose.position.y;
                    particles_[i].setYaw(tmp_particles[j].getYaw());
                    particles_[i].w = wo;
                    break;
                }
            }
        }
    }

    void MCL::printPath() {
        path_.header.frame_id = "map";
        path_.header.stamp = this->get_clock()->now();
        path_.poses.push_back(mcl_pose_);
        pub_path_->publish(path_);
    }

    std::optional<std::vector<double>> MCL::caculateLikelihoodFieldModel(yasarobo2025_26::Particle particle_pose) {
        double var = lfmSigma_*lfmSigma_;
        double normConst = 1.0 / (sqrt(2.0*M_PI*var));
        double p_rand = 1.0 / scan_->range_max * field_.getMapResolution();

        std::vector<double> p_vector;
        for (size_t i=0; i<scan_->ranges.size(); i+=scan_step_) {
            double r = scan_->ranges[i];
            if (std::isnan(r) || r < scan_->range_min || scan_->range_max < r) {
                p_vector.push_back(z_rand_*p_rand);
                continue;
            }

            double theta_lidar;
            if (is_sim_) {
                theta_lidar = scan_->angle_min + (static_cast<double>(i))*scan_->angle_increment;
            } else {
                theta_lidar = scan_->angle_min + (static_cast<double>(i))*scan_->angle_increment - 3.0*M_PI/2.0;
            }

            // TODO: tf
            double x_lidar = r*cos(theta_lidar) + 0.033 + 0.005;
            double y_lidar = r*sin(theta_lidar) + 0.013 - 0.013;
            double robot_theta = particle_pose.getYaw();
            double x = x_lidar*cos(robot_theta) - y_lidar*sin(robot_theta) + particle_pose.pose.position.x;
            double y = x_lidar*sin(robot_theta) + y_lidar*cos(robot_theta) + particle_pose.pose.position.y;

            int u_layser, v_layser;
            field_.xy2uv(x, y, u_layser, v_layser);

            int u_pose, v_pose;
            field_.xy2uv(particle_pose.pose.position.x, particle_pose.pose.position.y, u_pose, v_pose);

            if (field_.isOnField(u_layser, v_layser)) {
                double d = field_.getWallDistance(u_pose, v_pose, u_layser, v_layser);
                double p_hit = normConst * exp(-(d*d)/(2.0*var))*field_.mapResolution_;
                double p = z_hit_*p_hit + z_rand_*p_rand;
                if (p > 1.0) p = 1.0;
                p_vector.push_back(p);
            } else {
                p_vector.push_back(z_rand_*p_rand);
            }
        }

        return p_vector;
    }

    void MCL::caculateMeasurementModel() {
        std::vector<std::vector<double>> likelihood_table;
        likelihood_table.reserve(particles_.size());

        for (size_t i=0; i<particles_.size(); i++ ) {
            // TODO change mesurement model
            std::optional<std::vector<double>> p_vector = std::move(MCL::caculateLikelihoodFieldModel(particles_[i]));
            if (!p_vector) return;
            likelihood_table.push_back(p_vector.value());
        }

        double w = 0.0;
        double w_sum = 0.0;
        double loglikefood_sum = 0.0;
        for (size_t i=0; i<likelihood_table.size(); i++ ) {
            w = 0.0;
            for (size_t j=0; j<likelihood_table.size(); j++ ) {
                loglikefood_sum = 0.0;
                for (size_t k=0; k<likelihood_table[i].size(); k++ ) {
                    loglikefood_sum += std::log(likelihood_table[j][k]/likelihood_table[i][k]);
                }
                w += std::exp(loglikefood_sum);
            } 
            w = 1/w;
            particles_[i].w = w;
            w_sum += w*w;
        }
        effective_sample_size_ = 1.0 / w_sum;
    }

    void MCL::updateParticles() {
        // TODO: 0.1->var
        double delta_x = cmd_vel_->linear.x*0.1;
        double delta_y = cmd_vel_->linear.y*0.1;
        double delta_theta = cmd_vel_->angular.z*0.1;

        double dd2 = delta_x*delta_x + delta_y*delta_y;
        double dy2 = delta_theta*delta_theta;

        for (size_t i=0; i<this->particles_.size(); i++ ) {
            double dx = delta_x + randNormal(
                odomNoise1_*dd2 + odomNoise2_*dy2
            );
            double dy = delta_y + randNormal(
                odomNoise1_*dd2 + odomNoise2_*dy2
            );
            double dtheta = delta_theta + randNormal(
                odomNoise3_*dd2 + odomNoise4_*dy2
            );

            double theta = particles_[i].getYaw();
            particles_[i].pose.position.x = particles_[i].pose.position.x + cos(theta)*dx - sin(theta)*dy;
            particles_[i].pose.position.y = particles_[i].pose.position.y + sin(theta)*dx + cos(theta)*dy;
            particles_[i].setYaw(theta+dtheta);
        }
    }

    void MCL::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_ = msg;
    }

    void MCL::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = msg;
    }

    void MCL::resetParticlesDistribution(double noise_x, double noise_y, double noise_theta) {
        double wo = 1.0 / static_cast<double >(particles_.size());
        for (size_t i=0; i<particles_.size(); i++ ) {
            double x = mcl_pose_.pose.position.x + randNormal(noise_x);
            double y = mcl_pose_.pose.position.y + randNormal(noise_y);
            double theta = mcl_pose_.getYaw() + randNormal(noise_theta);

            particles_[i].pose.position.x = x;
            particles_[i].pose.position.y = y;
            particles_[i].setYaw(theta);
            particles_[i].w = wo;
        }
    };

    double MCL::randNormal(double n) { 
        return (n * sqrt(-2.0 * log(static_cast<double>(rand()+1.0)/(RAND_MAX+2.0))) * cos(2.0 * M_PI * static_cast<double>(rand()+1.0)/(RAND_MAX+2.0))); 
    }

    void MCL::printParticle() {
        sensor_msgs::msg::PointCloud2 cloud_;
        cloud_.header.stamp = this->get_clock()->now();
        cloud_.header.frame_id = "map";
        cloud_.height = 1;
        cloud_.width = particles_.size();
        cloud_.is_dense = false;
        cloud_.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(particles_.size());

        sensor_msgs::PointCloud2Iterator<std::float_t> iter_x(cloud_, "x");
        sensor_msgs::PointCloud2Iterator<std::float_t> iter_y(cloud_, "y");
        sensor_msgs::PointCloud2Iterator<std::float_t> iter_z(cloud_, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>  iter_r(cloud_, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t>  iter_g(cloud_, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t>  iter_b(cloud_, "b");

        for (const Particle &p: particles_) {
            *iter_x = p.pose.position.x;
            *iter_y = p.pose.position.y;
            *iter_z = 0;

            *iter_r = 0;
            *iter_g = 0;
            *iter_b = 255;

            ++iter_x, ++iter_y, ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }

        this->pub_particle_marker_->publish(cloud_);
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<yasarobo2025_26::MCL>());
    rclcpp::shutdown();
    return 0;
}