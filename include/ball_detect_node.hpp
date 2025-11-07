#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include <nanoflann.hpp>
#include <random>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <optional>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/srv/ball_pose.hpp>

namespace DBSCAN {
    enum ClusterID {
        UNVISITED=-1,
        NOISE=-2,
    };

    class UnitVector {
        public:
            UnitVector(double x, double y, double eps);
            double getX() const;
            double getY() const;
        private:
            double x_, y_;
    };

    class Point {
        public:
            Point(float x, float y, int pointID, int clusterID=DBSCAN::ClusterID::UNVISITED);
            float getX() const;
            float getY() const;
            int getID() const;
            int getPointID() const;
            void setID(int id);
        private:
            float x_, y_;
            int clusterID_;
            int pointID_;
    };

    class PointCloud {
        public:
            std::vector<Point> points;
            inline size_t kdtree_get_point_count() const;
            inline double kdtree_get_pt(const size_t idx, const size_t dim) const;

            template <class BBOX>
            bool kdtree_get_bbox(BBOX&) const;
    };

    class Circle {
        public:
            Circle();
            Circle(std::vector<DBSCAN::Point> &cluster);
            void markClosest();
            double getX();
            double getY();
            double getR();
            bool isClosest();
        private:
            double is_closest_;
            double x_, y_, r_;
    };

    class Field {
        public:
            Field();
            Field(std::string map_dir);
            bool isBallOnField(DBSCAN::Circle &c);
            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v);
            double mapResolution_;
            int mapWidth_, mapHeight_;
            std::vector<double> mapOrigin_;
            cv::Mat mapImg_;
    };

    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, DBSCAN::PointCloud>,
        DBSCAN::PointCloud,
        2>;

    class BallDetect: public rclcpp::Node {
        public:
            BallDetect(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            std::optional<geometry_msgs::msg::Pose2D> detect();

        private:
            // callback
            void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

            // DBSCAN algorithm
            std::unordered_map<int, std::vector<DBSCAN::Point>> dbscan(
                std::vector<DBSCAN::Point> &points, 
                DBSCAN::KdTree &tree
            );
            void expandCluster(
                Point &p, 
                std::vector<DBSCAN::Point> &points,
                std::vector<size_t> &neighbors, 
                DBSCAN::KdTree& tree,
                const int cluster_id
            );
            std::vector<size_t> regionQuery(
                DBSCAN::Point &p, 
                DBSCAN::KdTree& tree
            );

            // delete wall
            double median(std::vector<double>& v);
            std::vector<int> deleteWall(
                std::unordered_map<int, std::vector<DBSCAN::Point>>& clusters
            );
            // <cluster_id, cluster_points>
            std::vector<std::pair<int, std::vector<DBSCAN::Point>>> collectBallPoints(
                const std::unordered_map<int, std::vector<DBSCAN::Point>>& clusters,
                const std::vector<int>& ball_cluster_ids
            );

            // search
            std::optional<geometry_msgs::msg::Pose2D> findClosestBall(
                std::vector<DBSCAN::Circle> &ball
            );

            // convert LaserScan to Point
            DBSCAN::PointCloud scan2Point(const sensor_msgs::msg::LaserScan scan);

            sensor_msgs::msg::PointCloud2 point2PointCloud2(
                const std::vector<std::pair<int, std::vector<DBSCAN::Point>>> &points
            );

            bool isBallOnField(DBSCAN::Field &f, DBSCAN::Circle &c);

            // service server callback
            void ballPoseCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::BallPose::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::BallPose::Response> response
            );

            // Lidar
            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLider_;

            // pose
            std::unique_ptr<geometry_msgs::msg::Pose2D> pose_;

            // DBSCAN parameter
            double EPS_;
            int MIN_PTS_;
            
            // delete wall parameter
            double DIAGONAL_THTRSHOLD_;
            double DIFF_THTRSHOLD_;
            double WALL_THTRSHOLD_;
            double LIDAR_THTRSHOLD_;
            double RADIUS_THTRSHOLD_;

            // ball 
            std::vector<DBSCAN::Circle> ball_;
            sensor_msgs::msg::PointCloud2 circle2PointCloud2(std::vector<DBSCAN::Circle> ball_position);

            // env
            bool is_sim_;

            // tf
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            rclcpp::TimerBase::SharedPtr timer_;

            // lidar frame
            std::string frame_id_;

            // field
            DBSCAN::Field field_;

            // publisher
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubClusters_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBallShape_;

            // subscriber
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subPose_;

            // detact action server
            rclcpp::Service<inrof2025_ros_type::srv::BallPose>::SharedPtr srv_ball_pose_;
    };
}

