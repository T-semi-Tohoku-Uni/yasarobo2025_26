#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ActionNodes {
    class BTNode;

    class GetPose: public BT::SyncActionNode {
        public:
            GetPose(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BTNode> ros_node);
            static BT::PortsList providedPorts();
            BT::NodeStatus tick() override;
            ~GetPose();
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}