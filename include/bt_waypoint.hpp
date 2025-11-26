#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ActionNodes {
    class BTNode;

    class Waypoint: public BT::SyncActionNode {
        public:
            Waypoint(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BTNode> ros_node);
            static BT::PortsList providedPorts();
            ~Waypoint() override = default;
        private:
            virtual BT::NodeStatus tick() override;
            std::shared_ptr<BTNode> ros_node_;
    };
}