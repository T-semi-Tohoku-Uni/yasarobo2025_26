#include <bt_switch_color.hpp>

namespace ControlNodes {
    SwitchColor::SwitchColor(const std::string& name, const BT::NodeConfig& config)
        : ControlNode::ControlNode(name, config) {}

    void SwitchColor::halt() {
        ControlNode::halt();
    }

    BT::PortsList SwitchColor::providedPorts()
    {
        return {
            BT::InputPort<int32_t>("color")
        };
    }


    BT::NodeStatus SwitchColor::tick() {
        BT::Expected<int32_t> tmp_color = getInput<int32_t>("color");
        if (!tmp_color) {
            throw BT::RuntimeError("missing required input tmp_color: ", tmp_color.error());
        }

        int32_t color = tmp_color.value();

        BT::NodeStatus status = children_nodes_[color]->executeTick();
        if (status == BT::NodeStatus::RUNNING) {
            return BT::NodeStatus::RUNNING;
        }
        else {
            resetChildren();
            return status;
        }

    }
}