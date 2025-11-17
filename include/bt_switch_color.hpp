#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ControlNodes {
    class SwitchColor: public BT::ControlNode {
        public:
            SwitchColor(const std::string& name, const BT::NodeConfig& config);
            virtual ~SwitchColor() override = default;
            virtual void halt() override;
            static BT::PortsList providedPorts();
        private:
            virtual BT::NodeStatus tick() override;
    };
}