#pragma once
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include "ros_node.hpp"

using namespace BT;

namespace ActionNodes {
    class VacumeOn: public SyncActionNode {
        public:
            VacumeOn(const std::string& name, const NodeConfig& config, std::shared_ptr<BTNode> ros_node): 
                SyncActionNode(name, config),
                ros_node_(ros_node) {};

            // port info
            static PortsList providedPorts() {
                return {
                    InputPort<bool> ("on")
                };
            }

            NodeStatus tick() override {
                std::cout << "call generate route" << std::endl;

                Expected<bool> tmp_on = getInput<bool>("on");
                if (!tmp_on) {
                    throw BT::RuntimeError("missing required input x: ", tmp_on.error() );
                }

                double on = tmp_on.value();
                
                if (this->ros_node_ == nullptr) std::cerr << "null ptr" << std::endl;

                this->ros_node_->send_vacume_on(on);

                return NodeStatus::SUCCESS;
            }
        private:
            std::shared_ptr<BTNode> ros_node_;
    };

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
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}