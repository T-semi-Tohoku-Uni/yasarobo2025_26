#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace yasarobo2025_26{
    class BallPathNode:public rclcpp::Node{
        public:
            BallPathNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        private:
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subPose_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
            void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
            geometry_msgs::msg::Pose2D::SharedPtr pose_;
            void genBallPath(
                const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::GenRoute::Response> response
            );

            int num_points_;
            rclcpp::Service<inrof2025_ros_type::srv::GenRoute>::SharedPtr srv_gen_route_;

    };
}