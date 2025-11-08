#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals; 

typedef struct MotorVel {
        float v1;
        float v2;
        float v3;
    } MotorVel;

class FollowNode: public rclcpp::Node {
    public:
        explicit FollowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("follow_node", options) {
            this->declare_parameter<double>("lookahead_distance", 0.05);
            this->declare_parameter<double>("max_linear_speed", 0.2);
            this->declare_parameter<double>("max_theta_speed", 2.0);
            this->declare_parameter<double>("Kp_linear", 0.10);
            this->declare_parameter<double>("Ki_linear", 0.00);
            this->declare_parameter<double>("Kd_linear", 0.00);
            this->declare_parameter<double>("Kp_theta", 0.40);
            this->declare_parameter<double>("Ki_theta", 0.00);
            this->declare_parameter<double>("Kd_theta", 0.00);
            this->declare_parameter<double>("max_linear_tolerance", 0.08);
            this->declare_parameter<double>("max_reaching_distance", 0.02);
            this->declare_parameter<double>("max_theta_tolerance", 0.3);
            this->declare_parameter<double>("max_reaching_theta", 0.1);
            this->get_parameter("lookahead_distance", lookahead_distance_);
            this->get_parameter("max_linear_speed", max_linear_speed_);
            this->get_parameter("max_theta_speed", max_theta_speed_);
            this->get_parameter("Kp_linear", Kp_linear);
            this->get_parameter("Ki_linear", Ki_linear);
            this->get_parameter("Kd_linear", Kd_linear);
            this->get_parameter("Kp_theta", Kp_theta);
            this->get_parameter("Kt_theta", Ki_theta);
            this->get_parameter("Kd_theta", Kd_theta);
            this->get_parameter("max_linear_tolerance", max_linear_tolerance);
            this->get_parameter("max_reaching_distance", max_reaching_distance);
            this->get_parameter("max_theta_tolerance", max_theta_tolerance);
            this->get_parameter("max_reaching_theta", max_reaching_theta);






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

            timer_ = rclcpp::create_timer(
                this,
                this->get_clock(),
                100ms,
                std::bind(&FollowNode::controlLoop, this)
            );
            target_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("target_pose", 10);
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


            // publish goal position
            geometry_msgs::msg::Pose2D target_pose;
            target_pose.x = path_[current_waypoint_index_].pose.position.x;
            target_pose.y = path_[current_waypoint_index_].pose.position.y;
            //goal_pose.theta = path_[]

            target_pub_ ->publish(target_pose);

            //error calculation linear
            double dx = path_[current_waypoint_index_].pose.position.x - pose_.x;
            double dy = path_[current_waypoint_index_].pose.position.y - pose_.y;
            double d2 = dx*dx + dy*dy;
            double linear_error = std::hypot(dx, dy);
            double linear_goal_x = path_[path_.size() -1].pose.position.x - pose_.x;
            double linear_goal_y = path_[path_.size() -1].pose.position.y - pose_.y;
            double linear_goal_distance = std::hypot(linear_goal_x, linear_goal_y);


            //quoternion to yaw
            tf2::Quaternion q(
                path_[current_waypoint_index_].pose.orientation.x,
                path_[current_waypoint_index_].pose.orientation.y,
                path_[current_waypoint_index_].pose.orientation.z,
                path_[current_waypoint_index_].pose.orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            
            tf2::Quaternion q_goal(
                path_[path_.size() - 1].pose.orientation.x,
                path_[path_.size() - 1].pose.orientation.y,
                path_[path_.size() - 1].pose.orientation.z,
                path_[path_.size() - 1].pose.orientation.w
            );

            double roll_goal, pitch_goal, yaw_goal;
            tf2::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);

            double theta_goal = yaw_goal - pose_.theta;

            //error calculation theta
            double target_theta = yaw;
            double theta_error = target_theta - pose_.theta;
            //normalize angle to [-pi, pi]
            while (theta_error > M_PI) theta_error -= 2*M_PI;
            while (theta_error < -M_PI) theta_error += 2*M_PI;


        
            if (max_linear_tolerance > linear_error){  // && max_theta_tolerance > std::abs(theta_error)) { 
                if (current_waypoint_index_ < (int)path_.size() -1){
                    //move to next waypoint
                    current_waypoint_index_++;
                } 

                if (linear_goal_distance < max_reaching_distance){ //&& theta_goal < max_reaching_theta) {
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
                    static double linear_error_derivative_x_prev = 0.0;
                    double linear_error_derivative_x = dx - linear_error_prev_dx;
                    double linear_error_derivative_dx = linear_error_derivative_x - linear_error_derivative_x_prev;
                    
                    double linear_speed_cmd_dx = Kp_linear * linear_error_derivative_x
                                               + Ki_linear * dx * dt
                                               + Kd_linear * linear_error_derivative_dx / dt;

                    static double linear_speed_cmd_x = 0.0;
                    linear_speed_cmd_x += linear_speed_cmd_dx;

                    linear_error_prev_dx = dx;
                    linear_error_derivative_x_prev = linear_error_derivative_x;



                    static double linear_error_prev_dy = 0.0;
                    static double linear_error_derivative_y_prev = 0.0;
                    double linear_error_derivative_y = dy - linear_error_prev_dy;
                    double linear_error_derivative_dy = linear_error_derivative_y - linear_error_derivative_y_prev;
                    

                    double linear_speed_cmd_dy = Kp_linear * linear_error_derivative_y
                                               + Ki_linear * dy * dt
                                               + Kd_linear * linear_error_derivative_dy / dt;

                    static double linear_speed_cmd_y = 0.0; 
                    linear_speed_cmd_y += linear_speed_cmd_dy; 

                    
                    linear_error_prev_dy = dy;
                    linear_error_derivative_y_prev = linear_error_derivative_y;



                    
                    //PID control for theta speed
                    static double theta_error_prev = 0.0;
                    static double theta_error_integral = 0.0;

                    double theta_error_derivative = (theta_error - theta_error_prev) / dt;
                    theta_error_integral += theta_error * dt;

                    double theta_speed_cmd = Kp_theta * theta_error
                                             + Ki_theta * theta_error_integral
                                             + Kd_theta * theta_error_derivative; 
                    theta_error_prev = theta_error;

    


                    geometry_msgs::msg::Twist linear_speed;
                    linear_speed.linear.x = cos(pose_.theta) * linear_speed_cmd_x + sin(pose_.theta) * linear_speed_cmd_y;
                    linear_speed.linear.y = -sin(pose_.theta) * linear_speed_cmd_x + cos(pose_.theta) * linear_speed_cmd_y;
                    linear_speed.angular.z = theta_speed_cmd;

                    //apply speed limits 
                    geometry_msgs::msg::Twist clipped_v = clip(linear_speed);
                    cmd_pub_->publish(clipped_v);


                    double clipped_v_x_r = clipped_v.linear.x;
                    double clipped_v_y_r = clipped_v.linear.y;

     
                    double clipped_v_x_f = cos(pose_.theta) * clipped_v_x_r - sin(pose_.theta) * clipped_v_y_r;
                    double clipped_v_y_f = sin(pose_.theta) * clipped_v_x_r + cos(pose_.theta) * clipped_v_y_r; 


                    linear_speed_cmd_x = clipped_v_x_f;
                    linear_speed_cmd_y = clipped_v_y_f;

                    
                    //publish feedback
                    auto feedback_msg = std::make_shared<inrof2025_ros_type::action::Follow::Feedback>();
                    feedback_msg->x = pose_.x;
                    feedback_msg->y = pose_.y;
                    feedback_msg->theta = pose_.theta;
                    goal_handle_->publish_feedback(feedback_msg);
                }

        }
         

        MotorVel forwardKinematics(float vx, float vy, float vtheta) {
                MotorVel motor_vel;
                // motor_vel.v1 = vx + r_*vtheta;
                // motor_vel.v2 = 0.5 * vx + std::sqrt(3)/2*vy - r_*vtheta;
                // motor_vel.v3 = -0.5 * vx + std::sqrt(3)/2*vy + r_*vtheta;
                motor_vel.v1 = (-vy) + r_*vtheta;
                motor_vel.v2 = 0.5 * (-vy) + std::sqrt(3)/2*vx - r_*vtheta;
                motor_vel.v3 = -0.5 * (-vy) + std::sqrt(3)/2*vx + r_*vtheta;
                return motor_vel;
            }


        geometry_msgs::msg::Twist inverseKinematics(float v1, float v2, float v3) {
                geometry_msgs::msg::Twist twist;
                // twist.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
                // twist.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
                // twist.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
                twist.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
                twist.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
                twist.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
                return twist;
            }


        geometry_msgs::msg::Twist clip(geometry_msgs::msg::Twist cmd){
            double linear_speed_cmd_x;
            double linear_speed_cmd_y;
            double theta_speed_cmd;
            linear_speed_cmd_x = cmd.linear.x;
            linear_speed_cmd_y = cmd.linear.y;
            theta_speed_cmd = cmd.angular.z;

            MotorVel v_motor = forwardKinematics(linear_speed_cmd_x, linear_speed_cmd_y, theta_speed_cmd);
            double v1 = v_motor.v1;
            double v2 = v_motor.v2;
            double v3 = v_motor.v3;

            double v_max = std::max({std::abs(v1), std::abs(v2), std::abs(v3)});

            if (v_max > max_linear_speed_){
                double scale = max_linear_speed_ / v_max;
                v1 *= scale;
                v2 *= scale;
                v3 *= scale; 
            }

            return inverseKinematics(v1, v2, v3);
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
        float r_ = 0.14;

        // subscriber
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr target_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        std::mutex mutex_;
        geometry_msgs::msg::Pose2D pose_;
        


        // PID gains
        double Kp_linear;
        double Ki_linear;
        double Kd_linear;
        double Kp_theta;
        double Ki_theta;
        double Kd_theta;

        //tolerance and reaching distance
        double max_linear_tolerance;
        double max_theta_tolerance;
        double max_reaching_distance;
        double max_reaching_theta;

        // waypoint index
        int current_waypoint_index_;    
        
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