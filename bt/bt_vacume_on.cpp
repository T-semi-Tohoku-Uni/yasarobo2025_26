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
    BT::Expected<bool> tmp_on = getInput<bool>("on");
    if (!tmp_on) throw BT::RuntimeError("missing required input x: ", tmp_on.error() );

    bool on = tmp_on.value();
    if (this->ros_node_ == nullptr) RCLCPP_ERROR(this->ros_node_->get_logger(), "null ptr");
    this->ros_node_->send_vacume_on(on);

    if (on) RCLCPP_INFO(this->ros_node_->get_logger(), "VacumeOn");
    else RCLCPP_INFO(this->ros_node_->get_logger(), "Vacume OFF");

    return BT::NodeStatus::SUCCESS;
}

ActionNodes::VacumeOn::~VacumeOn() {
    this->ros_node_.reset();
}