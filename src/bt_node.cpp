#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>
#include <inrof2025_ros_type/srv/ball_pose.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include "../include/bt_node.hpp"
#include "../include/bt_vacume_on.hpp"

using namespace std::chrono_literals;
using namespace BT;

namespace ActionNodes {
    class BallDetact: public SyncActionNode {
        public:
            BallDetact(const std::string& name, const NodeConfig& config, std::shared_ptr<BTNode> ros_node):
                SyncActionNode(name, config),
                ros_node_(ros_node) {};

            // port info
            static PortsList providedPorts() {
                return {
                    OutputPort<double>("x"),
                    OutputPort<double>("y")
                };
            }

            NodeStatus tick() override {
                double x, y;
                this->ros_node_->ball_detect(&x, &y);

                setOutput("x", x);
                setOutput("y", y);

                return NodeStatus::SUCCESS;
            }

            ~BallDetact() override {
                this->ros_node_.reset();
            }
        private:
            std::shared_ptr<BTNode> ros_node_;
    };

    class GenerateRoute: public SyncActionNode {
        public:
            GenerateRoute(const std::string& name, const NodeConfig& config, std::shared_ptr<BTNode> ros_node): 
                SyncActionNode(name, config),
                ros_node_(ros_node) {};

            // port info
            static PortsList providedPorts() {
                return {
                    InputPort<double> ("x"),
                    InputPort<double> ("y")
                };
            }

            NodeStatus tick() override {
                std::cout << "call generate route" << std::endl;

                Expected<double> tmp_x = getInput<double>("x");
                Expected<double> tmp_y = getInput<double>("y");
                if (!tmp_x) {
                    throw BT::RuntimeError("missing required input x: ", tmp_x.error() );
                }
                if (!tmp_y) {
                    throw BT::RuntimeError("missing required input x: ", tmp_y.error() );
                }

                double x = tmp_x.value();
                double y = tmp_y.value();
                
                if (this->ros_node_ == nullptr) std::cerr << "null ptr" << std::endl;

                this->ros_node_->send_pose(x, y);

                return NodeStatus::SUCCESS;
            }

            ~GenerateRoute() {
                this->ros_node_.reset();
            }
        private:
            std::shared_ptr<BTNode> ros_node_;
    }; 

    class FollowRoute: public StatefulActionNode {
        public:
            FollowRoute(const std::string& name, const NodeConfiguration& config, std::shared_ptr<BTNode> ros_node) :
                StatefulActionNode(name, config),
                ros_node_(ros_node){}

            NodeStatus onStart() override {
                this->ros_node_->send_start_follow();
                return NodeStatus::RUNNING;
            }

            NodeStatus onRunning() override {
                
                if (this->ros_node_->isRuning()) {
                    return NodeStatus::RUNNING;
                } else {
                    return NodeStatus::SUCCESS;
                }
            }

            void onHalted() override {
                // TODO
                std::cout << "interrupt SampleNode" << std::endl;
            }

            ~FollowRoute() {
                this->ros_node_.reset();
            }
        private:
            std::shared_ptr<BTNode> ros_node_;
    };

    class Rotate : public StatefulActionNode {
        public:
            Rotate(const std::string& name, const NodeConfig& config, std::shared_ptr<BTNode> ros_node) :
                StatefulActionNode(name, config),
                ros_node_(ros_node){}

            static PortsList providedPorts() {
                return { InputPort<double>("theta") };
            }

            NodeStatus onStart() override {
                std::cout << "call SampleNode" << std::endl;
                
                // InputPortの値を受け取る
                Expected<double> msg = getInput<double>("theta");
                if (!msg) { // Inputの値が適切でないときの処理
                    throw BT::RuntimeError("missing required input [sample_input]: ", msg.error() );
                }
                double targetTheta = msg.value();
                this->ros_node_->send_rotate_position(targetTheta);

                return NodeStatus::RUNNING;
            }

            NodeStatus onRunning() override {
            
                if (this->ros_node_->isRotateRuning()) {
                    return NodeStatus::RUNNING;
                } else {
                    return NodeStatus::SUCCESS;
                }
                return NodeStatus::SUCCESS;
            }

            void onHalted() override {
                std::cout << "interrupt SampleNode" << std::endl;
            }

            ~Rotate() {
                this->ros_node_.reset();
            }
            
        private:
            std::shared_ptr<BTNode> ros_node_;
    };

    BTNode::BTNode(const rclcpp::NodeOptions & options): Node("bt_node", options) {
        // create service client
        srvGenRoute_ = this->create_client<inrof2025_ros_type::srv::GenRoute>("generate_route");
        srvVacume_ = this->create_client<inrof2025_ros_type::srv::Vacume>("/srv/vacume");
        srvBall_ = this->create_client<inrof2025_ros_type::srv::BallPose> ("ball_pose");
        actFollow_ = rclcpp_action::create_client<inrof2025_ros_type::action::Follow> (this, "follow");
        actRotate_ = rclcpp_action::create_client<inrof2025_ros_type::action::Rotate> (this, "rotate");
    };

    void BTNode::send_pose(double x, double y) {
        // check action server available
        while (!this->srvGenRoute_->wait_for_service(1s))
        {
            if (!rclcpp::ok()) break;
            RCLCPP_WARN(this->get_logger(), "srvGenRoute not available");
        }

        auto request = std::make_shared<inrof2025_ros_type::srv::GenRoute::Request>();
        request->x = x;
        request->y = y;

        srvGenRoute_->async_send_request(request);
    }

    bool BTNode::isRuning() {
        return isRun_;
    }

    void BTNode::ball_detect(double *x, double *y) {
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
        } else {

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
            RCLCPP_INFO(this->get_logger(), "get goal_handle");
        } else {
            RCLCPP_WARN(this->get_logger(), "empty goal_handle");
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
            RCLCPP_INFO(this->get_logger(), "get goal_handle");
        } else {
            RCLCPP_WARN(this->get_logger(), "empty goal_handle");
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
            return std::make_unique<ActionNodes::BallDetact>(name, config, ros_node);
        };
    factory.registerBuilder<ActionNodes::BallDetact>("ball_detect", builder_ball_detect);

    std::string package_path = ament_index_cpp::get_package_share_directory("yasarobo2025_26");
    factory.registerBehaviorTreeFromFile(package_path + "/config/main_bt.xml");

    BT::Tree tree = factory.createTree("MainBT");

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