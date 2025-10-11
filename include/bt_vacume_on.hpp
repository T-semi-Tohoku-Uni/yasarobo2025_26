#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ActionNodes {
    class BTNode;

    class VacumeOn: public BT::SyncActionNode {
        public:
            VacumeOn(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BTNode> ros_node);
            static BT::PortsList providedPorts();
            BT::NodeStatus tick() override;
            ~VacumeOn();
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}