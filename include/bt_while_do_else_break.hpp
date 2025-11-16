#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ControlNodes {
    class WhileDoElseBreakNode: public BT::ControlNode {
        public:
            WhileDoElseBreakNode(const std::string& name);
            virtual ~WhileDoElseBreakNode() override = default;
            virtual void halt() override;
        private:
            virtual BT::NodeStatus tick() override;
            size_t child_idx_;
    };
}