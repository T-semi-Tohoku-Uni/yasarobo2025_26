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

            casadi::SX inverseKinematics(casadi::SX v1, casadi::SX v2, casadi::SX v3);
            casadi::SX compute_A(casadi::SX a, casadi::SX b, casadi::SX c);
            casadi::SX compute_stage_cost(casadi::SX states, casadi::SX controls);
            casadi::SX compute_terminal_cost(casadi::SX states); 
            casadi::SX x_ref_ = casadi::SX::zeros(nx);
            casadi::SX u_ref_ = casadi::SX::zeros(nu);
            casadi::SX x_lb_;
            casadi::SX u_lb_;
            casadi::SX x_ub_;
            casadi::SX u_ub_;
            casadi::SX Q_;
            casadi::SX R_;
            casadi::SX Qf_;
            casadi::Function make_f();
            casadi::Function make_nlp();
            
            

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subpose_;
            geometry_msgs::msg::Pose2D::SharedPtr pose_;

            int nx_ = 3;
            int nu_ = 3;
            int K_ = 10;
            double r_ = 0.14;
            
    };
}