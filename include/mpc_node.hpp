#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <casadi/casadi.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/action/follow.hpp>

namespace yasarobo2025_26{
    class MpcNode:public rclcpp::Node{
        public:
            MpcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


        private:
            void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
            void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
            
            rclcpp_action::GoalResponse handleGoal(
                const rclcpp_action::GoalUUID &,
                std::shared_ptr<const inrof2025_ros_type::action::Follow::Goal> goal
            );
            rclcpp_action::CancelResponse handleCancel(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
            );
            void handleAccepted(
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
            );

            
            void control();
            casadi::Function make_f();
            casadi::SX compute_stage_cost(const casadi::SX& x, const casadi::SX& u);
            casadi::SX compute_terminal_cost(const casadi::SX& x);
            casadi::Function make_nlp();
            std::pair<casadi::DM, casadi::DM> compute_optimal_control(
                casadi::Function& S,
                const casadi::DM& x_init,
                const casadi::DM& x0
            );

            std::array<casadi::SX, 3> inverseKinematics(casadi::SX& v1, casadi::SX& v2, casadi::SX& v3);
            std::array<double, 3> inverseKinematics(double v1, double v2, double v3);
            std::array<casadi::SX, 3> toFieldVel(casadi::SX& v_xr, casadi::SX& v_yr, casadi::SX& omega_r, casadi::SX& theta_f);

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
            geometry_msgs::msg::Pose2D::SharedPtr pose_;
            nav_msgs::msg::Path::SharedPtr path_;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subpose_;
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subpath_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp_action::Server<inrof2025_ros_type::action::Follow>::SharedPtr action_server_;
            std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle_;

            casadi::DM Q_;
            casadi::DM Qf_;
            casadi::DM R_;
            casadi::DM x_ref_;
            casadi::DM u_ref_;
            casadi::DM x_lb_;
            casadi::DM x_ub_;
            casadi::DM u_lb_;
            casadi::DM u_ub_;

            // waypoint parameter
            size_t current_index_;
            double lookahead_distance = 0.05;
            double max_reaching_distance = 0.05;

            int K_;
            int nx_;
            int nu_;

            double r_;
            double delta_t_;
    };
}