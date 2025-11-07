#include "../include/bt_node.hpp"
#include "../include/bt_generate_route.hpp"

ActionNodes::GenerateRoute::GenerateRoute (
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<BTNode> ros_node
):  
    BT::SyncActionNode(name, config),
    ros_node_(ros_node) {};

BT::PortsList ActionNodes::GenerateRoute::providedPorts() {
    return {
        BT::InputPort<double> ("x"),
        BT::InputPort<double> ("y"),
        BT::InputPort<double> ("theta")
    };
}

BT::NodeStatus ActionNodes::GenerateRoute::tick() {
    RCLCPP_INFO(this->ros_node_->get_logger(), "Start GenerateRoute");

    BT::Expected<double> tmp_x = getInput<double>("x");
    BT::Expected<double> tmp_y = getInput<double>("y");
    BT::Expected<double> tmp_theta = getInput<double>("theta");
    if (!tmp_x) {
        throw BT::RuntimeError("missing required input x: ", tmp_x.error() );
    }
    if (!tmp_y) {
        throw BT::RuntimeError("missing required input x: ", tmp_y.error() );
    }

    double x = tmp_x.value();
    double y = tmp_y.value();
    double theta = tmp_theta.value();
    
    if (this->ros_node_ == nullptr) RCLCPP_INFO(this->ros_node_->get_logger(), "null ptr");

    this->ros_node_->send_pose(x, y, theta);

    return BT::NodeStatus::SUCCESS;
}

ActionNodes::GenerateRoute::~GenerateRoute() {
    this->ros_node_.reset();
}