#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>
#include <inrof2025_ros_type/srv/ball_pose.hpp>
#include <inrof2025_ros_type/srv/waypoint.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <bt_ball_path.hpp>
#include "../include/bt_node.hpp"
#include "../include/bt_vacume_on.hpp"
#include "../include/bt_ball_detect.hpp"
#include "../include/bt_generate_route.hpp"
#include "../include/bt_follow_route.hpp"
#include "../include/bt_rotate.hpp"
#include <bt_get_pose.hpp>
#include <inrof2025_ros_type/srv/ball_path.hpp>
#include <bt_increment.hpp>
#include <bt_while_do_else_break.hpp>
#include <bt_ball_color.hpp>
#include <bt_switch_color.hpp>
#include <bt_waypoint.hpp>

using namespace std::chrono_literals;
using namespace BT;

namespace ActionNodes {
    BTNode::BTNode(const rclcpp::NodeOptions & options): Node("bt_node", options) {
        // create service client
        srvGenRoute_ = this->create_client<inrof2025_ros_type::srv::GenRoute>("generate_route");
        srvVacume_ = this->create_client<inrof2025_ros_type::srv::Vacume>("/srv/vacume");
        srvBall_ = this->create_client<inrof2025_ros_type::srv::BallPose> ("ball_detect");
        srvBallRoute_ = this->create_client<inrof2025_ros_type::srv::BallPath>("generate_ball_path");
        srvPose_ = this->create_client<inrof2025_ros_type::srv::Pose>("pose");
        srvBallColor_ = this->create_client<inrof2025_ros_type::srv::BallColor>("color");
        srvWaypoint_ = this->create_client<inrof2025_ros_type::srv::Waypoint>("waypoint");
        actFollow_ = rclcpp_action::create_client<inrof2025_ros_type::action::Follow> (this, "follow");
        actRotate_ = rclcpp_action::create_client<inrof2025_ros_type::action::Rotate> (this, "rotate");
    };

    void BTNode::send_waypoint(double x, double y) {
        while(!this->srvWaypoint_->wait_for_service(1s)) {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvWaypoint_ not available");
        }

        inrof2025_ros_type::srv::Waypoint_Request::SharedPtr request 
            = std::make_shared<inrof2025_ros_type::srv::Waypoint::Request>();
        request->x = x;
        request->y = y;

        rclcpp::Client<inrof2025_ros_type::srv::Waypoint>::FutureAndRequestId result_future
            = srvWaypoint_->async_send_request(request);
        
        if (
            rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future,
                std::chrono::seconds(1)
            ) == rclcpp::FutureReturnCode::SUCCESS
        ) {}
    }

    void BTNode::send_ball_pose(double x, double y, bool is_return) {
        // check action server available
        while (!this->srvBallRoute_->wait_for_service(1s))
        {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvBallRoute_ not available");
        }

        auto request = std::make_shared<inrof2025_ros_type::srv::BallPath::Request>();
        request->x = x;
        request->y = y;
        request->is_return = is_return;

        auto result_future = srvBallRoute_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future,
                std::chrono::seconds(1))
            == rclcpp::FutureReturnCode::SUCCESS) {}
    }

    void BTNode::send_pose(double x, double y, double theta) {
        // check action server available
        while (!this->srvGenRoute_->wait_for_service(1s))
        {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvGenRoute not available");
        }

        auto request = std::make_shared<inrof2025_ros_type::srv::GenRoute::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;

        auto result_future = srvGenRoute_->async_send_request(request);

        if (
            rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future,
                std::chrono::seconds(1)
            ) == rclcpp::FutureReturnCode::SUCCESS
        ) {}
    }

    inrof2025_ros_type::srv::Pose::Response BTNode::get_pose() {
        while (!this->srvPose_->wait_for_service(1s)) {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvPose not available");
        }

        auto request = std::make_shared<inrof2025_ros_type::srv::Pose::Request>();
        auto result_future = srvPose_->async_send_request(request);

        if (
            rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future,
                std::chrono::seconds(1)
            ) == rclcpp::FutureReturnCode::SUCCESS
        ) {
            std::shared_ptr<inrof2025_ros_type::srv::Pose::Response> response = result_future.get();
            return *response;
        }
    }

    inrof2025_ros_type::srv::BallColor::Response BTNode::ball_color() {
        while(!this->srvBallColor_->wait_for_service(1s)) {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvColor not available");
        }

        auto request = std::make_shared<inrof2025_ros_type::srv::BallColor::Request>();
        auto result_future = srvBallColor_->async_send_request(request);

        if (
            rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future,
                std::chrono::seconds(1)
            ) == rclcpp::FutureReturnCode::SUCCESS
        ) {
            std::shared_ptr<inrof2025_ros_type::srv::BallColor::Response> response = result_future.get();
            return *response;
        }
    }

    bool BTNode::isRuning() {
        return isRun_;
    }

    bool BTNode::ball_detect(double *x, double *y) {
        while(!srvBall_->wait_for_service(1s)) {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvBall_ not available");
        }

        auto request = std::make_shared<inrof2025_ros_type::srv::BallPose::Request>();
        auto result_future = srvBall_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                result_future,
                std::chrono::seconds(1))
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            *x = response->x;
            *y = response->y;
            return response->detect;
        } else {
            RCLCPP_WARN(this->get_logger(), "spin until future is fail");
            return false;
        }
    }

    void BTNode::send_vacume_on(bool on) {
        while (!srvVacume_->wait_for_service(1s)){
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvVacume not available");
        }
        std::cout << "srvVacume service available" << std::endl;

        auto request = std::make_shared<inrof2025_ros_type::srv::Vacume::Request>();
        request->on = on;

        srvVacume_->async_send_request(request);
    }

    void BTNode::send_start_follow() {
        while (!actFollow_->wait_for_action_server(1s)){
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "actFollow_ not available");
        }

        auto goal_msg = inrof2025_ros_type::action::Follow::Goal();
        auto send_goal_options = rclcpp_action::Client<inrof2025_ros_type::action::Follow>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&BTNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&BTNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&BTNode::resultCallback, this, std::placeholders::_1);

        actFollow_->async_send_goal(goal_msg, send_goal_options);
        this->isRun_ = true;
    }

    void BTNode::goalResponseCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr goal_handle){
        if (goal_handle) {

        } else {

        }
    }

    void BTNode::feedbackCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr goal_handle, 
        const std::shared_ptr<const inrof2025_ros_type::action::Follow::Feedback> feedback)
    {
        (void) goal_handle;
    }

    void BTNode::resultCallback(const rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::WrappedResult result) {
        this->isRun_ = false;
    }

    // rotate
    void BTNode::send_rotate_position(double theta) {
        while(!actRotate_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "actRotate_ not available");
        }

        auto goal_msg = inrof2025_ros_type::action::Rotate::Goal();
        auto send_goal_options = rclcpp_action::Client<inrof2025_ros_type::action::Rotate>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&BTNode::rotateGoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&BTNode::rotateFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&BTNode::rotateResultCallback, this, std::placeholders::_1);
        goal_msg.theta = theta;

        actRotate_->async_send_goal(goal_msg, send_goal_options);
        this->isRotateRun_ = true;
    } 

    void BTNode::rotateGoalResponseCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Rotate>::SharedPtr goal_handle){
        if (goal_handle) {

        } else {
            
        }
    }

    void BTNode::rotateFeedbackCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Rotate>::SharedPtr goal_handle, 
        const std::shared_ptr<const inrof2025_ros_type::action::Rotate::Feedback> feedback)
    {
        (void) goal_handle;
    }

    void BTNode::rotateResultCallback(const rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Rotate>::WrappedResult result) {
        this->isRotateRun_ = false;
    }


    bool BTNode::isRotateRuning() {
        return this->isRotateRun_;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<ActionNodes::BTNode> ros_node = std::make_shared<ActionNodes::BTNode>();

    BehaviorTreeFactory factory;

    BT::NodeBuilder builder_vacume_on = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::VacumeOn>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::VacumeOn>("vacume_on", builder_vacume_on);

    BT::NodeBuilder builder_generate_route = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::GenerateRoute>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::GenerateRoute>("generate_route", builder_generate_route);

    BT::NodeBuilder builder_rotate =
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::Rotate>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::Rotate>("rotate", builder_rotate);

    BT::NodeBuilder builder_follow_route = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::FollowRoute>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::FollowRoute>("follow_route", builder_follow_route);

    BT::NodeBuilder builder_ball_detect = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::BallDetect>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::BallDetect>("ball_detect", builder_ball_detect);

    BT::NodeBuilder builder_ball_path = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::BallPath>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::BallPath>("ball_path", builder_ball_path);

    BT::NodeBuilder builder_get_pose = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::GetPose>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::GetPose>("get_pose", builder_get_pose);

    BT::NodeBuilder builder_increment = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::Increment>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::Increment>("increment", builder_increment);

    BT::NodeBuilder builder_ball_color = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::BallColor>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::BallColor>("ball_color", builder_ball_color);

    BT::NodeBuilder builder_waypoint = 
        [ros_node](const std::string& name, const NodeConfiguration& config) {
            return std::make_unique<ActionNodes::Waypoint>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::Waypoint>("waypoint", builder_waypoint);

    factory.registerNodeType<ControlNodes::WhileDoElseBreakNode>("WhileDoElseBreak");
    factory.registerNodeType<ControlNodes::SwitchColor>("SwitchColor");

    std::string package_path = ament_index_cpp::get_package_share_directory("yasarobo2025_26");
    factory.registerBehaviorTreeFromFile(package_path + "/config/main_bt.xml");

    BT::Tree tree = factory.createTree("MainBT");

    BT::Groot2Publisher groot2_publisher(tree);
    printTreeRecursively(tree.rootNode());

    NodeStatus status = NodeStatus::RUNNING;

    while(status == NodeStatus::RUNNING && rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        status = tree.tickOnce();
    }

    tree.haltTree();
    rclcpp::shutdown();

    return 0;
}