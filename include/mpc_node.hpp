#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <casadi/casadi.hpp>



namespace yasarobo2025_26{
    class MpcNode:public rclcpp::Node{
        public:
            MpcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


        private:
            void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
            void make_f();

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
            geometry_msgs::msg::Pose2D::SharedPtr pose_;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subpose_;
            
            
    };
}