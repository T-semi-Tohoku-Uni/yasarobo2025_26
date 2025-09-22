#include "../include/action_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace ActionNodes;
using namespace BT;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    ros_node = std::make_shared<BTNode>();

    BehaviorTreeFactory factory;
    factory.registerNodeType<GenerateRoute>("generate_route");
    factory.registerNodeType<FollowRoute>("follow_route");
    factory.registerNodeType<Rotate>("rotate");
    factory.registerNodeType<VacumeOn>("vacume_on");
    factory.registerNodeType<BallDetact>("ball_detect");

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