#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace yasarobo2025_26 {
    class Pose: public geometry_msgs::msg::PoseStamped {
        public:
            void setYaw(double yaw);
            double getYaw();
            geometry_msgs::msg::PoseStamped toPoseStamped();
            geometry_msgs::msg::Pose2D toPose2D();
    };

    class Particle: public Pose {
        public:
            double w;
    };
}

namespace yasarobo2025_26 {
    class Field {
        public:
            Field() = default;
            Field(std::string map_dir);
            double getMapResolution();
            void xy2uv(double x, double y, int& u, int& v);
            double getWallDistance(int u_pose, int v_pose, int u_layser, int v_layser);
            bool isOnField(int u, int v);
            double mapHeight_, mapWidth_;
            double mapResolution_;
        private:
            cv::Mat mapImg_;
            cv::Mat distance_field_;

            int win_u_min, win_u_max, win_v_min, win_v_max;
            // const int INSIDE = 0b0000;
            // const int LEFT   = 0b0001;
            // const int RIGHT  = 0b0010;
            // const int BOTTOM = 0b0100;
            // const int TOP    = 0b1000;
            int computeOutCode(int u, int v);
    };
}

namespace yasarobo2025_26 {
    class MCL: public rclcpp::Node {
        public:
            explicit MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        private:
            // main loop
            void loop();
            void updateParticles();
            void caculateMeasurementModel();
            std::optional<std::vector<double>> caculateLikelihoodFieldModel(yasarobo2025_26::Particle particle_pose);
            void estimatePose();
            void resampleParticles();

            // field
            yasarobo2025_26::Field field_;

            // callback function
            void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

            // parameter
            int scan_step_;
            double odomNoise1_, odomNoise2_, odomNoise3_, odomNoise4_;
            double lfmSigma_;
            double z_rand_, z_hit_;
            double resample_threshold_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_;

            // robot estimated pose
            yasarobo2025_26::Pose mcl_pose_;

            // particles
            void resetParticlesDistribution(double noise_x, double noise_y, double noise_theta);
            std::vector<yasarobo2025_26::Particle> particles_;
            double effective_sample_size_;

            // env
            bool is_sim_;

            // tf
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

            // shared ptr
            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            geometry_msgs::msg::Twist::SharedPtr cmd_vel_;

            // publisher
            rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_particle_marker_;

            // subscriber
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

            // utility
            double randNormal(double n);
            void printParticle();
            void printPath();
            nav_msgs::msg::Path path_;
    };
}