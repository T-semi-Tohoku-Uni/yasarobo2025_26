#include <bt_get_pose.hpp>
#include <bt_node.hpp>

ActionNodes::GetPose::GetPose(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<BTNode> ros_node
):
    BT::SyncActionNode(name, config),
    ros_node_(ros_node) {};

BT::PortsList ActionNodes::GetPose::providedPorts() {
    return {
        BT::OutputPort<double> ("x"),
        BT::OutputPort<double> ("y"),
        BT::OutputPort<double> ("theta")
    };
}

BT::NodeStatus ActionNodes::GetPose::tick() {
    inrof2025_ros_type::srv::Pose::Response pose = this->ros_node_->get_pose();
    
    setOutput("x", pose.x);
    setOutput("y", pose.y);
    setOutput("theta", pose.theta);

    return BT::NodeStatus::SUCCESS;
}

ActionNodes::GetPose::~GetPose() {
    this->ros_node_.reset();
}