#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include <nanoflann.hpp>
#include <random>

namespace DBSCAN {
    enum ClusterID {
        UNVISITED=-1,
        NOISE=-2,
    };

    class Point {
        public:
            Point(float x, float y, int clusterID=DBSCAN::ClusterID::UNVISITED);
            float getX() const;
            float getY() const;
            int getID() const;
            void setID(int id);
        private:
            float x_, y_;
            int clusterID_;
    };

    class PointCloud {
        public:
            std::vector<Point> points;
            inline size_t kdtree_get_point_count() const;
            inline double kdtree_get_pt(const size_t idx, const size_t dim) const;

            template <class BBOX>
            bool kdtree_get_bbox(BBOX&) const;
    };


    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, DBSCAN::PointCloud>,
        DBSCAN::PointCloud,
        2>;

    class BallDetect: public rclcpp::Node {
        public:
            BallDetect(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            geometry_msgs::msg::Pose2D detect();

        private:
            // callback
            void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

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
            std::unordered_map<int, std::vector<DBSCAN::Point>> deleteWall(
                std::unordered_map<int, std::vector<DBSCAN::Point>> clusters
            );

            // convert LaserScan to Point
            DBSCAN::PointCloud scan2Point(const sensor_msgs::msg::LaserScan scan);

            sensor_msgs::msg::PointCloud2 point2PointCloud2(
                const std::vector<DBSCAN::Point> &points
            );

            // Lidar
            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLider_;

            // DBSCAN parameter
            double EPS_;
            double MIN_PTS_;
            
            // env
            bool is_sim_;

            // For debug
            std::string frame_id_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubClusters_;
    };
}

