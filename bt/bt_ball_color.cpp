#include <bt_ball_color.hpp>
#include <bt_node.hpp>

namespace ActionNodes {
    BallColor::BallColor(
        const std::string& name, 
        const BT::NodeConfig& config, 
        std::shared_ptr<BTNode> ros_node
    ): 
        BT::SyncActionNode(name, config),
        ros_node_(ros_node) {};

    BT::PortsList BallColor::providedPorts() {
        return {
            BT::OutputPort<int> ("color")
        };
    }

    BT::NodeStatus BallColor::tick() {
        inrof2025_ros_type::srv::BallColor::Response ball_color = this->ros_node_->ball_color();

        setOutput("color", ball_color.color);

        return BT::NodeStatus::SUCCESS;
    }

    BallColor::~BallColor() {
        this->ros_node_.reset();
    }
}