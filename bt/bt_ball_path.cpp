#include <bt_ball_path.hpp>
#include <bt_node.hpp>

ActionNodes::BallPath::BallPath(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<BTNode> ros_node
):  
    BT::SyncActionNode(name, config),
    ros_node_(ros_node) {};

BT::PortsList ActionNodes::BallPath::providedPorts() {
    return {
        BT::InputPort<double> ("x"),
        BT::InputPort<double> ("y")
    };
}

BT::NodeStatus ActionNodes::BallPath::tick() {
    RCLCPP_INFO(this->ros_node_->get_logger(), "Start BallPath");

    BT::Expected<double> tmp_x = getInput<double>("x");
    BT::Expected<double> tmp_y = getInput<double>("y");
    if (!tmp_x) {
        throw BT::RuntimeError("missing required input x: ", tmp_x.error() );
    }
    if (!tmp_y) {
        throw BT::RuntimeError("missing required input x: ", tmp_y.error() );
    }

    double x = tmp_x.value();
    double y = tmp_y.value();
    
    if (this->ros_node_ == nullptr) RCLCPP_INFO(this->ros_node_->get_logger(), "null ptr");

    this->ros_node_->send_ball_pose(x, y);

    return BT::NodeStatus::SUCCESS;
}