#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ActionNodes {
    class BTNode;

    class Rotate: public BT::StatefulActionNode {
        public:
            Rotate(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BTNode> ros_node);
            static BT::PortsList providedPorts();
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
            ~Rotate();
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}