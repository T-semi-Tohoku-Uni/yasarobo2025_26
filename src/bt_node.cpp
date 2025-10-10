#include "../include/action_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace ActionNodes;
using namespace BT;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<BTNode> ros_node = std::make_shared<BTNode>();

    BehaviorTreeFactory factory;

    BT::NodeBuilder builder_vacume_on = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<VacumeOn>(name, config, ros_node);
        };
    factory.registerBuilder<VacumeOn>("vacume_on", builder_vacume_on);

    BT::NodeBuilder builder_generate_route = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<GenerateRoute>(name, config, ros_node);
        };
    factory.registerBuilder<GenerateRoute>("generate_route", builder_generate_route);

    BT::NodeBuilder builder_rotate =
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<Rotate>(name, config, ros_node);
        };
    factory.registerBuilder<Rotate>("rotate", builder_rotate);

    BT::NodeBuilder builder_follow_route = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<FollowRoute>(name, config, ros_node);
        };
    factory.registerBuilder<FollowRoute>("follow_route", builder_follow_route);

    BT::NodeBuilder builder_ball_detect = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<BallDetact>(name, config, ros_node);
        };
    factory.registerBuilder<BallDetact>("ball_detect", builder_ball_detect);

    std::string package_path = ament_index_cpp::get_package_share_directory("yasarobo2025_26");
    factory.registerBehaviorTreeFromFile(package_path + "/config/main_bt.xml");

    BT::Tree tree = factory.createTree("MainBT");

    printTreeRecursively(tree.rootNode());

    NodeStatus status = NodeStatus::RUNNING;

    while(status == NodeStatus::RUNNING && rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        status = tree.tickOnce();
    }

    rclcpp::shutdown();

    return 0;
}