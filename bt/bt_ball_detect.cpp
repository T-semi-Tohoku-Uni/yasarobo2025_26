#include "../include/bt_node.hpp"
#include "../include/bt_ball_detect.hpp"


ActionNodes::BallDetect::BallDetect(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<ActionNodes::BTNode> ros_node
):
    BT::SyncActionNode(name, config),
    ros_node_(ros_node) {};

BT::PortsList ActionNodes::BallDetect::providedPorts() {
    return {
        BT::OutputPort<double>("x"),
        BT::OutputPort<double>("y")
    };
}

BT::NodeStatus ActionNodes::BallDetect::tick() {
    double x, y;
    this->ros_node_->ball_detect(&x, &y);

    setOutput("x", x);
    setOutput("y", y);

    return BT::NodeStatus::SUCCESS;
}

ActionNodes::BallDetect::~BallDetect() {
    this->ros_node_.reset();
}