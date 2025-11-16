#include <bt_increment.hpp>
#include <bt_node.hpp>

ActionNodes::Increment::Increment(
    const std::string& name, 
    const BT::NodeConfig& config,
    std::shared_ptr<BTNode> ros_node
): 
    SyncActionNode(name, config),
    ros_node_(ros_node) {}

BT::PortsList ActionNodes::Increment::providedPorts() {
    return {
        BT::InputPort<double>("value"),
        BT::BidirectionalPort<double>("counter")
    };
}

BT::NodeStatus ActionNodes::Increment::tick() {
    BT::Expected<double> tmp_value = getInput<double>("value");
    BT::Expected<double> tmp_counter = getInput<double>("counter");

    if (!tmp_value) {
        throw BT::RuntimeError("missing required input x: ", tmp_value.error() );
    }
    if (!tmp_counter) {
        throw BT::RuntimeError("missing required input x: ", tmp_counter.error() );
    }

    double value = tmp_value.value();
    double counter = tmp_counter.value();

    setOutput("counter", counter+value);

    return BT::NodeStatus::SUCCESS;
 }

 ActionNodes::Increment::~Increment() {
    this->ros_node_.reset();
 }