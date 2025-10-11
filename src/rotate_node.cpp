#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>

class RotateNode: public rclcpp::Node {
    public:
        explicit RotateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("rotate_node", options) {
            this->declare_parameter<std::float_t>("p", 1);
            this->p_ = this->get_parameter("p").as_double();

            action_server_ = rclcpp_action::create_server<inrof2025_ros_type::action::Rotate> (
                this,
                "rotate",
                std::bind(&RotateNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&RotateNode::handleCancel, this, std::placeholders::_1),
                std::bind(&RotateNode::handleAccepted, this, std::placeholders::_1)
            );

            rclcpp::QoS poseQos(rclcpp::KeepLast(5));
            poseSub_ = this->create_subscription<geometry_msgs::msg::Pose2D> (
                "pose", poseQos, std::bind(&RotateNode::odomCallback, this, std::placeholders::_1)
            );

            velPub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RotateNode::control, this));
        
            this->max_speed_    = 0.5;
            this->slow_speed_   = 0.2;
            this->accel_angle_  = M_PI/10;
            this->stop_angle_   = M_PI/90;

            RCLCPP_INFO(this->get_logger(), "fgpui34wp9tfgjopiarejgoiqa34uwhioahjfgi3arkehagvjaekrngvklresjgers"); 
            RCLCPP_INFO(this->get_logger(), "Success intialize rotate_node");
        }

        // action server callback
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &,
            std::shared_ptr<const inrof2025_ros_type::action::Rotate::Goal> goal
        ) {
            if (!goal_handle_) {
                targetTheta_ = goal->theta;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        rclcpp_action::CancelResponse handleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Rotate>> goal_handle
        ) {
            goal_handle_.reset();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Rotate>> goal_handle
        ) {
            goal_handle_ = goal_handle;
        }

        void control() {
            if (!goal_handle_) return;
            double err = normalizePi(targetTheta_ - pose_.theta);
            double abs_err = std::abs(err);
            double speed_cmd = 0.0;

            if (abs_err > accel_angle_) {
                // まだ十分遠い → 最大速度
                speed_cmd = max_speed_;
            } else if (abs_err > stop_angle_) {
                // 減速域 → 一定の遅い速度
                speed_cmd = slow_speed_;
            } else {
                // 目標到達
                auto result = std::make_shared<inrof2025_ros_type::action::Rotate::Result>();
                result->success = true;

                geometry_msgs::msg::Twist cmd;
                cmd.angular.z = 0.0;
                velPub_->publish(cmd);

                goal_handle_->succeed(result);
                goal_handle_.reset();
                return;
            }

            // 方向だけ err の符号で制御
            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = (err > 0 ? +1 : -1) * speed_cmd;

            velPub_->publish(cmd);

        }

        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Rotate>> goal_handle_;
    private:
        void odomCallback(geometry_msgs::msg::Pose2D msgs) {
            pose_.x = msgs.x;
            pose_.y = msgs.y;
            pose_.theta = msgs.theta;
        }

        double normalizePi(double a) {
            a = std::fmod(a + M_PI, 2.0*M_PI);
            if (a < 0) a += 2.0*M_PI;
            return a - M_PI;
        }

        rclcpp_action::Server<inrof2025_ros_type::action::Rotate>::SharedPtr action_server_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr poseSub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;
        geometry_msgs::msg::Pose2D pose_;
        rclcpp::TimerBase::SharedPtr timer_;

        double targetTheta_;
        double p_;
        double accel_angle_;
        double stop_angle_;
        double max_speed_;
        double slow_speed_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotateNode>());
    rclcpp::shutdown();
    return 0;
}