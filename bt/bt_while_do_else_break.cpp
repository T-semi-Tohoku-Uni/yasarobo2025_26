#include <bt_while_do_else_break.hpp>

namespace ControlNodes {
    WhileDoElseBreakNode::WhileDoElseBreakNode(const std::string& name)
        : ControlNode::ControlNode(name, {}), child_idx_(0) {}

    void WhileDoElseBreakNode::halt() {
        child_idx_ = 0;
        ControlNode::halt();
    }

    BT::NodeStatus WhileDoElseBreakNode::tick() {
        const size_t children_count = children_nodes_.size();

        if(children_count != 3)
        {
            throw std::logic_error("WhileDoElseNode must have 3 children");
        }

        setStatus(BT::NodeStatus::RUNNING);

        if (child_idx_ == 0 || child_idx_ == 2) {
            BT::NodeStatus condition_status = children_nodes_[0]->executeTick();

            if (condition_status == BT::NodeStatus::RUNNING) {
                return condition_status;
            }
            else if (condition_status == BT::NodeStatus::SUCCESS) {
                child_idx_ = 1;
            }
            else if (condition_status == BT::NodeStatus::FAILURE) {
                child_idx_ = 2;
            }
        }

        if (child_idx_ > 0) {
            BT::NodeStatus status = children_nodes_[child_idx_]->executeTick();
            if (status == BT::NodeStatus::RUNNING) {
                return BT::NodeStatus::RUNNING;
            }
            else {
                resetChildren();
                child_idx_ = 0;
                return status;
            }
        }
    }

}

