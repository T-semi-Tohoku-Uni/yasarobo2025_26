#include "../include/bt_vacume_on.hpp"
#include "../include/bt_node.hpp"

ActionNodes::VacumeOn::VacumeOn(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<ActionNodes::BTNode> ros_node
): 
    SyncActionNode(name, config),
    ros_node_(ros_node) {};

BT::PortsList ActionNodes::VacumeOn::providedPorts() {
    return {
        BT::InputPort<bool> ("on")
    };
}

BT::NodeStatus ActionNodes::VacumeOn::tick() {
    RCLCPP_INFO(this->ros_node_->get_logger(), "Call vacume on");

    BT::Expected<bool> tmp_on = getInput<bool>("on");
    if (!tmp_on) throw BT::RuntimeError("missing required input x: ", tmp_on.error() );

    double on = tmp_on.value();
    if (this->ros_node_ == nullptr) RCLCPP_ERROR(this->ros_node_->get_logger(), "null ptr");
    this->ros_node_->send_vacume_on(on);
    return BT::NodeStatus::SUCCESS;
}

ActionNodes::VacumeOn::~VacumeOn() {
    this->ros_node_.reset();
}