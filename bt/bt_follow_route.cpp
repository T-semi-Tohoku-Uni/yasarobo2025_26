#include "../include/bt_follow_route.hpp"
#include "../include/bt_node.hpp"

ActionNodes::FollowRoute::FollowRoute(
    const std::string& name, 
    const BT::NodeConfiguration& config, 
    std::shared_ptr<BTNode> ros_node
): 
    StatefulActionNode(name, config),
    ros_node_(ros_node){}

BT::NodeStatus ActionNodes::FollowRoute::onStart() {
    this->ros_node_->send_start_follow();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionNodes::FollowRoute::onRunning() {
    if (this->ros_node_->isRuning()) {
        return BT::NodeStatus::RUNNING;
    } else {
        return BT::NodeStatus::SUCCESS;
    }   
}

void ActionNodes::FollowRoute::onHalted() {
    RCLCPP_INFO(this->ros_node_->get_logger(), "interrupt follow node");
}

ActionNodes::FollowRoute::~FollowRoute() {
    this->ros_node_.reset();
}