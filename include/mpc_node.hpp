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
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subpose_;
            rclcpp::TimerBase::SharedPtr timer_;
            casadi::DM Q_;
            casadi::DM Qf_;
            casadi::DM R_;
            casadi::DM x_ref_;
            casadi::DM u_ref_;
            casadi::DM x_lb_;
            casadi::DM x_ub_;
            casadi::DM u_lb_;
            casadi::DM u_ub_;

            int K_;
            int nx_;
            int nu_;

            double r_;
            double delta_t_;
    };
}