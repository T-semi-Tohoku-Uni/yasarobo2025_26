#include "../include/bt_rotate.hpp"
#include "../include/bt_node.hpp"

ActionNodes::Rotate::Rotate(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<BTNode> ros_node
): 
    StatefulActionNode(name, config),
    ros_node_(ros_node){}


BT::PortsList ActionNodes::Rotate::providedPorts() {
    return { BT::InputPort<double>("theta") };
}

BT::NodeStatus ActionNodes::Rotate::onStart() {
    RCLCPP_INFO(this->ros_node_->get_logger(), "Call Rotate");
    // Get inputport
    BT::Expected<double> msg = getInput<double>("theta");
    if (!msg) { // Inputの値が適切でないときの処理
        throw BT::RuntimeError("missing required input [sample_input]: ", msg.error() );
    }
    double targetTheta = msg.value();
    this->ros_node_->send_rotate_position(targetTheta);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActionNodes::Rotate::onRunning() {
    if (this->ros_node_->isRotateRuning()) {
        return BT::NodeStatus::RUNNING;
    } else {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
}

void ActionNodes::Rotate::onHalted() {
    RCLCPP_INFO(this->ros_node_->get_logger(), "Interrupt Rotate");
}

ActionNodes::Rotate::~Rotate() {
    this->ros_node_.reset();
}