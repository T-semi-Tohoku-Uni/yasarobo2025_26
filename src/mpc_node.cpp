#include <mpc_node.hpp>

namespace yasarobo2025_26{
    MpcNode::MpcNode(const rclcpp::NodeOptions & options): Node("mpc_node", options){
        
        subpose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&MpcNode::poseCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10
        );
    }

    void MpcNode::make_f(){
        // nx は状態変数の次元数, nu は制御入力の次元数
        int nx = 3;
        int nu = 3;
        casadi::SX states = casadi::SX::sym("states", nx);
        casadi::SX controls = casadi::SX::sym("controls", nu);

        //状態変数の定義
        casadi::SX x_f = states(0);
        casadi::SX y_f = states(1);
        casadi::SX theta_f = states(2);

        //制御入力の定義
        casadi::SX v1 = controls(0);
        casadi::SX v2 = controls(1);
        casadi::SX v3 = controls(2);

        //

    }
}