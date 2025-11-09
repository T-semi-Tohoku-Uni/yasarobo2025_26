#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ActionNodes {
    class BTNode;

    class BallPath: public BT::SyncActionNode {
        public:
            BallPath(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BTNode> ros_node);
            static BT::PortsList providedPorts();
            BT::NodeStatus tick() override;
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}