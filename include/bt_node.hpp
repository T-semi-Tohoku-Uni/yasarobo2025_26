#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/srv/gen_route.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>
#include <inrof2025_ros_type/srv/ball_pose.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>

namespace ActionNodes {
    class BTNode: public rclcpp::Node {
        public:
            explicit BTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            void send_pose(double x, double y);
            bool isRuning();
            void ball_detect(double *x, double *y);
            void send_vacume_on(bool on);
            void send_start_follow();
            void goalResponseCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr goal_handle);
            void feedbackCallback(
                rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr goal_handle, 
                const std::shared_ptr<const inrof2025_ros_type::action::Follow::Feedback> feedback
            );
            void resultCallback(const rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::WrappedResult result);
            void send_rotate_position(double theta);
            void rotateGoalResponseCallback(rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Rotate>::SharedPtr goal_handle);
            void rotateFeedbackCallback(
                rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Rotate>::SharedPtr goal_handle, 
                const std::shared_ptr<const inrof2025_ros_type::action::Rotate::Feedback> feedback
            );
            void rotateResultCallback(const rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Rotate>::WrappedResult result);
            bool isRotateRuning();
        private:
            rclcpp::Client<inrof2025_ros_type::srv::GenRoute>::SharedPtr srvGenRoute_;
            rclcpp::Client<inrof2025_ros_type::srv::Vacume>::SharedPtr srvVacume_;
            rclcpp::Client<inrof2025_ros_type::srv::BallPose>::SharedPtr srvBall_;
            rclcpp_action::Client<inrof2025_ros_type::action::Follow>::SharedPtr actFollow_;
            rclcpp_action::ClientGoalHandle<inrof2025_ros_type::action::Follow>::SharedPtr currentFollow_;
            rclcpp_action::Client<inrof2025_ros_type::action::Rotate>::SharedPtr actRotate_;
            bool isRun_{false};
            bool isRotateRun_{false};
    };
}