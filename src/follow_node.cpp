#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>

class FollowNode: public rclcpp::Node {
    public:
        explicit FollowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("follow_node", options) {
            this->declare_parameter<double>("lookahead_distance", 0.05);
            this->declare_parameter<double>("max_linear_speed", 0.2);
            this->declare_parameter<double>("max_theta_speed", 2.0);
            this->declare_parameter<double>("Kp_linear", 1.0);
            this->declare_parameter<double>("Ki_linear", 0.20);
            this->declare_parameter<double>("Kd_linear", 0.00);
            this->get_parameter("lookahead_distance", lookahead_distance_);
            this->get_parameter("max_linear_speed", max_linear_speed_);
            this->get_parameter("max_theta_speed", max_theta_speed_);
            this->get_parameter("Kp_linear", Kp_linear);
            this->get_parameter("Ki_linear", Ki_linear);
            this->get_parameter("Kd_linear", Kd_linear);
            





            rclcpp::QoS pathQos(rclcpp::KeepLast(5));
            path_sub_ = this->create_subscription<nav_msgs::msg::Path> (
                "route", pathQos, std::bind(&FollowNode::pathCallback, this, std::placeholders::_1)
            );
            rclcpp::QoS odomQos(rclcpp::KeepLast(5));
            // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
            //     "odom", odomQos, std::bind(&FollowNode::odomCallback, this, std::placeholders::_1)
            // );
            rclcpp::QoS poseQos(rclcpp::KeepLast(5));
            pose_sub_= this->create_subscription<geometry_msgs::msg::Pose2D> (
                "pose", poseQos, std::bind(&FollowNode::odomCallback, this, std::placeholders::_1)
            );
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&FollowNode::controlLoop, this)
            );
            action_server_ = rclcpp_action::create_server<inrof2025_ros_type::action::Follow>(
                this,
                "follow",
                std::bind(&FollowNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FollowNode::handleCancel, this, std::placeholders::_1),
                std::bind(&FollowNode::handleAccepted, this, std::placeholders::_1)
            );
        }

        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle_;

    private:
        // action server callback
        rclcpp_action::GoalResponse handleGoal(
            const rclcpp_action::GoalUUID &,
            std::shared_ptr<const inrof2025_ros_type::action::Follow::Goal> goal
        ) {
            if (!goal_handle_){
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        rclcpp_action::CancelResponse handleCancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
        ) {
            goal_handle_.reset();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handleAccepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
        ) {
            goal_handle_ = goal_handle;
        }

        void pathCallback(nav_msgs::msg::Path msgs) {
            // std::lock_guard<std::mutex> lock(mutex_);
            path_ = msgs.poses;
            current_waypoint_index_ = 0;
        }
        void odomCallback(geometry_msgs::msg::Pose2D msgs) {
            // std::lock_guard<std::mutex> lock(mutex_);
            pose_.x = msgs.x;
            pose_.y = msgs.y;
            pose_.theta = msgs.theta;
            // RCLCPP_INFO(this->get_logger(), "%.4f %.4f", pose_.x, pose_.y);
        }
        void controlLoop() {
            //do nothing if if there is no goal or path
            if (!goal_handle_){
                return;
            }
            if (path_.empty()){
                return;
            }

            //PID gains
            
            //double Kp_theta= 1.00;
            //double Ki_theta= 0.00;
            //double Kd_theta= 0.00;

            //decide tolerance range
            double max_linear_tolerance = 0.06;  //m
            double max_reaching_distance = 0.05; //m
            double lookahead_distance = 1.0; //m
            //double max_theta_tolerance = 0.05;  //rad

            //error calculation
            double dx = path_[current_waypoint_index_].pose.position.x - pose_.x;
            double dy = path_[current_waypoint_index_].pose.position.y - pose_.y;
            double d2 = dx*dx + dy*dy;
            double linear_error = std::hypot(dx, dy);
            double linear_goal_x = path_[path_.size() -1].pose.position.x - pose_.x;
            double linear_goal_y = path_[path_.size() -1].pose.position.y - pose_.y;
            double linear_goal_distance = std::hypot(linear_goal_x, linear_goal_y);
            //double target_theta = atan2(dy, dx);
            //double theta_error = target_theta - pose_.theta;
            //normalize angle to [-pi, pi]
            //while (theta_error > M_PI) theta_error -= 2*M_PI;
            //while (theta_error < -M_PI) theta_error += 2*M_PI;

        
            if (max_linear_tolerance > linear_error){ //&& max_theta_tolerance > std::abs(theta_error)) {
                if (current_waypoint_index_ < (int)path_.size() -1){ //&&  linear_error < lookahead_distance_ / 2.0 ) {
                    //move to next waypoint
                    current_waypoint_index_++;
                } 

                if (linear_goal_distance < max_reaching_distance) {
                    //goal reached
                    RCLCPP_INFO(this->get_logger(), "Goal reached.");
                    publishZero();
                    auto result_msg = std::make_shared<inrof2025_ros_type::action::Follow::Result>();
                    result_msg->success = true;
                    goal_handle_->succeed(result_msg);
                    goal_handle_.reset();
                }

            } else {
                    double dt = 0.1;

                    //PID control for linear speed
                    static double linear_error_prev_dx = 0.0;
                    static double linear_error_integral_x = 0.0;
                    linear_error_integral_x += dx * dt;
                    double linear_error_derivative_x = (dx - linear_error_prev_dx) / dt;
                    
                    double linear_speed_cmd_x = Kp_linear * dx
                                            + Ki_linear * linear_error_integral_x
                                            + Kd_linear * linear_error_derivative_x;
                    linear_error_prev_dx = dx;

                    //RCLCPP_INFO(this->get_logger(), "dx: %.4f dy: %.4f", dx, dy);

                    static double linear_error_prev_dy = 0.0;
                    static double linear_error_integral_y = 0.0;
                    linear_error_integral_y += dy * dt;
                    double linear_error_derivative_y = (dy - linear_error_prev_dy) / dt;

                    double linear_speed_cmd_y = Kp_linear * dy
                                            + Ki_linear * linear_error_integral_y
                                            + Kd_linear * linear_error_derivative_y;
                    linear_error_prev_dy = dy;



                    //RCLCPP_INFO(this->get_logger(), "pose:", "%.4f %.4f ", pose_.x, pose_.y);
                    //RCLCPP_INFO(this->get_logger(), "path:", "%.4f %.4f ", path_[current_waypoint_index_].pose.position.x, path_[current_waypoint_index_].pose.position.y);
                    //RCLCPP_INFO(this->get_logger(), "dx, dy:", "%.4f %.4f ", dx, dy);
                    //RCLCPP_INFO(this->get_logger(), "linear cmd: %.4f %.4f ", linear_speed_cmd_x, linear_speed_cmd_y);


                    


                    //PID control for theta speed
                    //static double theta_error_prev = 0.0;
                    //static double theta_error_integral = 0.0;

                    //double theta_error_derivative = (theta_error - theta_error_prev)/dt;
                    //theta_error_integral += theta_error * dt;
                    //double theta_speed_cmd = Kp_theta * theta_error 
                    //                       + Ki_theta * theta_error_integral
                    //                       + Kd_theta * theta_error_derivative; 
                    //theta_error_prev = theta_error;

                    //apply speed limits                    
                    if (linear_speed_cmd_x > max_linear_speed_) linear_speed_cmd_x = max_linear_speed_;
                    if (linear_speed_cmd_x < -max_linear_speed_) linear_speed_cmd_x = -max_linear_speed_;
                    if (linear_speed_cmd_y > max_linear_speed_) linear_speed_cmd_y = max_linear_speed_;
                    if (linear_speed_cmd_y < -max_linear_speed_) linear_speed_cmd_y = -max_linear_speed_;
                    //if (theta_speed_cmd > max_theta_speed_) theta_speed_cmd = max_theta_speed_;
                    //if (theta_speed_cmd < -max_theta_speed_) theta_speed_cmd = -max_theta_speed_;

                    //日本語のやつはchatGPT
                    // theta_error は [-pi, pi] に正規化済み
                    //double linear_speed_cmd_limited = linear_speed_cmd_x; 


                    // ロボットが後ろ向きの場合は線速度を反転
                    //if (std::abs(theta_error) > M_PI_2) {  
                    //    linear_speed_cmd_limited = -linear_speed_cmd;
                        // 角度も補正（theta_error を π の範囲内にして回転方向を逆に）
                    //    if (theta_error > 0) theta_error -= M_PI;
                    //    else theta_error += M_PI;
                    //}


                    double linear_speed_cmd_x_1 = cos(pose_.theta) * linear_speed_cmd_x + sin(pose_.theta) * linear_speed_cmd_y;
                    double linear_speed_cmd_y_1 = -sin(pose_.theta) * linear_speed_cmd_x + cos(pose_.theta) * linear_speed_cmd_y;



                    geometry_msgs::msg::Twist cmd;
                    cmd.linear.x = linear_speed_cmd_x_1;
                    cmd.linear.y = linear_speed_cmd_y_1;
                    //cmd.angular.z = Kp_theta * theta_error + Ki_theta*theta_error_integral + Kd_theta*theta_error_derivative;


                    //send cmd
                    //geometry_msgs::msg::Twist cmd;
                    //cmd.linear.x = linear_speed_cmd;
                    //cmd.angular.z = theta_speed_cmd;
                    cmd_pub_->publish(cmd);

                    //publish feedback
                    auto feedback_msg = std::make_shared<inrof2025_ros_type::action::Follow::Feedback>();
                    feedback_msg->x = pose_.x;
                    feedback_msg->y = pose_.y;
                    feedback_msg->theta = pose_.theta;
                    goal_handle_->publish_feedback(feedback_msg);
                }

           

        }
         

        void publishZero()
        {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
        }

        // action server
        rclcpp_action::Server<inrof2025_ros_type::action::Follow>::SharedPtr action_server_;

        double lookahead_distance_;
        double max_linear_speed_;
        double max_theta_speed_;

        // subscriber
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        std::mutex mutex_;
        geometry_msgs::msg::Pose2D pose_;


        // PID gains
        double Kp_linear;
        double Ki_linear;
        double Kd_linear;

        // waypoint index
        int current_waypoint_index_;    

        // action server
        rclcpp_action::Server<inrof2025_ros_type::action::Follow>::SharedPtr action_server_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle_;

        // rotate action server
        rclcpp_action::Server<inrof2025_ros_type::action::Rotate>::SharedPtr action_rotate_server_;
        std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Rotate>> goal_rotate_handle_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}