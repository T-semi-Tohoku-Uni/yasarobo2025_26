#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace ActionNodes {
    class BTNode;

    class FollowRoute: public BT::StatefulActionNode {
        public:
            FollowRoute(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BTNode> ros_node);
            BT::NodeStatus onStart() override;
            BT::NodeStatus onRunning() override;
            void onHalted() override;
            ~FollowRoute();
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}