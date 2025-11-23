#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>
#include <vector>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2025_ros_type/action/follow.hpp>
#include <inrof2025_ros_type/action/rotate.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "visualization_msgs/msg/marker.hpp"
#include <functional>

using namespace std::chrono_literals; 

typedef struct MotorVel {
        float v1;
        float v2;
        float v3;
    } MotorVel;



class PIDController {
    public:
        PIDController() = default;
        PIDController(double Kp, double Ki, double Kd, double dt, std::function<double(double)> normalize_func = nullptr)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), prev_error_(0.0), integral_(0.0), dt_(dt), normalize_func_(normalize_func) {}

        double compute(double setpoint, double measured_value) {
            double error = setpoint - measured_value;
            if (normalize_func_) {
                error = normalize_func_(error);
            }
            integral_ += error * dt_;
            double derivative = (error - prev_error_) / dt_;
            prev_error_ = error;
            return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        }

    private:
        double Kp_;
        double Ki_;
        double Kd_;
        double dt_;
        double prev_error_;
        double integral_;
        std::function<double(double)> normalize_func_;
};


class FollowNode: public rclcpp::Node {
    public:
        explicit FollowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("follow_node", options) {
            double Kp_tan, Ki_tan, Kd_tan;
            double Kp_norm, Ki_norm, Kd_norm;
            double Kp_theta, Ki_theta, Kd_theta;
            double dt = 0.1; 
            this->declare_parameter<double>("lookahead_distance", 0.05);
            this->declare_parameter<double>("max_linear_speed", 0.2);
            this->declare_parameter<double>("max_theta_speed", 2.0);
            this->declare_parameter<double>("Kp_tan", 0.80);
            this->declare_parameter<double>("Ki_tan", 0.00);
            this->declare_parameter<double>("Kd_tan", 0.00);
            this->declare_parameter<double>("Kp_norm", 0.80);
            this->declare_parameter<double>("Ki_norm", 0.00);
            this->declare_parameter<double>("Kd_norm", 0.00);
            this->declare_parameter<double>("Kp_theta", 0.40);
            this->declare_parameter<double>("Ki_theta", 0.00);
            this->declare_parameter<double>("Kd_theta", 0.00);
            this->declare_parameter<double>("max_linear_tolerance", 0.25);
            this->declare_parameter<double>("max_reaching_distance", 0.15);
            this->declare_parameter<double>("max_theta_tolerance", 0.3);
            this->declare_parameter<double>("max_reaching_theta", 0.1);
            this->declare_parameter<double>("min_approach_distance", 0.10);
            this->declare_parameter<int>("x", 2);
            this->declare_parameter<double>("L_min_", 0.05);
            this->declare_parameter<double>("L_max_", 0.50);
            this->declare_parameter<double>("L0_", 0.25);
            this->declare_parameter<double>("k_v_", 0.50);
            this->declare_parameter<double>("k_c_", 0.04);
            this->declare_parameter<double>("k_l_", 1.5);
            this->get_parameter("lookahead_distance", lookahead_distance_);
            this->get_parameter("max_linear_speed", max_linear_speed_);
            this->get_parameter("max_theta_speed", max_theta_speed_);
            this->get_parameter("Kp_tan", Kp_tan);
            this->get_parameter("Ki_tan", Ki_tan);
            this->get_parameter("Kd_tan", Kd_tan);
            this->get_parameter("Kp_norm", Kp_norm);
            this->get_parameter("Ki_norm", Ki_norm);
            this->get_parameter("Kd_norm", Kd_norm);
            this->get_parameter("Kp_theta", Kp_theta);
            this->get_parameter("Kt_theta", Ki_theta);
            this->get_parameter("Kd_theta", Kd_theta);
            this->get_parameter("max_linear_tolerance", max_linear_tolerance);
            this->get_parameter("max_reaching_distance", max_reaching_distance);
            this->get_parameter("max_theta_tolerance", max_theta_tolerance);
            this->get_parameter("max_reaching_theta", max_reaching_theta);
            this->get_parameter("min_approach_distance", min_approach_distance);
            this->get_parameter("x", x_);
            this->get_parameter("L_min_", L_min_);
            this->get_parameter("L_max_", L_max_);
            this->get_parameter("L0_", L0_);
            this->get_parameter("k_v_", k_v_);
            this->get_parameter("k_c_", k_c_);
            this->get_parameter("k_l_", k_l_);



            linear_PID_tan_ = PIDController(Kp_tan, Ki_tan, Kd_tan, dt);
            linear_PID_norm_ = PIDController(Kp_norm, Ki_norm, Kd_norm, dt);
            omega_PID_ = PIDController(Kp_theta, Ki_theta, Kd_theta, dt, [](double e){
                while (e > M_PI) e -= 2*M_PI;
                while (e < -M_PI) e += 2*M_PI;
                return e;
            });



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

            rclcpp::QoS markerQos(rclcpp::KeepLast(10));
            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", markerQos);
            lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "lookahead_marker", 10);
            rclcpp::QoS poseArrowQos(rclcpp::KeepLast(10));
            pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", poseArrowQos);
            rclcpp::QoS cmdVelArrowQos(rclcpp::KeepLast(10));
            cmd_vel_arrow_pub = this->create_publisher<visualization_msgs::msg::Marker>("cmd_vel_arrow_marker", cmdVelArrowQos);

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


        
        void updateCurrentWaypoint(){

            double span = 5.0; 
            if (path_.empty()) return;
            if (current_waypoint_index_ >= static_cast<int>(path_.size()) - 1) return;
            

            while (current_waypoint_index_ < static_cast<int>(path_.size()) -1) {

                //calculate vector from current waypoint to next waypoint
                double Ax = path_[current_waypoint_index_ + span].pose.position.x - path_[current_waypoint_index_].pose.position.x;
                double Ay = path_[current_waypoint_index_ + span].pose.position.y - path_[current_waypoint_index_].pose.position.y;
                
                //calculate vector from next waypoint to robot pose
                double Bx = pose_.x - path_[current_waypoint_index_ + span].pose.position.x;
                double By = pose_.y - path_[current_waypoint_index_ + span].pose.position.y;

                // double Cx = pose_.x - path_[current_waypoint_index_].pose.position.x;
                // double Cy = pose_.y - path_[current_waypoint_index_].pose.position.y;

                double AB = Ax * Bx + Ay * By;
                double A2 = Ax * Ax + Ay * Ay;
                // double AC = Ax * Cx + Ay * Cy;

                RCLCPP_INFO(this->get_logger(), "AB: %.4f", AB);
 
                double t = AB / A2;
                if (A2 < 1e-6) {
                    current_waypoint_index_++;
                    continue;
                }

                double linear_error = std::hypot(
                    path_[current_waypoint_index_ + span].pose.position.x - pose_.x,
                    path_[current_waypoint_index_ + span].pose.position.y - pose_.y
                );


                if (t < 1.0) break;
                if (linear_error > max_linear_tolerance) break;
                current_waypoint_index_++;

                if (current_waypoint_index_ >= static_cast<int>(path_.size()) - 1) break;
            }

        }


        void updateCurrentWaypoint3() {

            if (path_.empty()) return;

            while (current_waypoint_index_ < static_cast<int>(path_.size()) - 1) {

                int i = current_waypoint_index_;
                auto &P0 = path_[i].pose.position;
                auto &P1 = path_[i+5].pose.position;

                // segment vector and robot vector
                double Ax = P1.x - P0.x;
                double Ay = P1.y - P0.y;
                double Bx = pose_.x - P1.x;
                double By = pose_.y - P1.y;
                double Cx = pose_.x - path_[current_waypoint_index_].pose.position.x;
                double Cy = pose_.y - path_[current_waypoint_index_].pose.position.y;

                double A_len2 = Ax*Ax + Ay*Ay;
                if (A_len2 < 1e-6) {
                    current_waypoint_index_++;
                    continue;
                }

                double t = (Ax*Bx + Ay*By) / A_len2;

                // robot projection point
                double proj_x = P0.x + Ax * t;
                double proj_y = P0.y + Ay * t;

                double dist_to_proj = std::hypot(pose_.x - proj_x, pose_.y - proj_y);
                double dist_to_P1   = std::hypot(pose_.x - P1.x, pose_.y - P1.y);

                // 外積 sign → P1 がロボット後方にあるか判定
                double cross = Ax * By - Ay * Bx;

                bool passed_forward  = (t > 1.0);                      // 正面から通過
                bool passed_side     = (dist_to_P1 < max_linear_tolerance); // 横から通過
                // bool passed_backside = (cross * t < 0);                // 後方側侵入（大きく逸脱）

                if (!(passed_forward || passed_side) ){ //|| passed_backside)) {
                    break;
                }

                current_waypoint_index_++;
                // RCLCPP_INFO(this->get_logger(), "Waypoint advanced");

                if (current_waypoint_index_ >= static_cast<int>(path_.size()) - 1) break;
            }
        }


        //たまにうまく動かなかったから、一旦max_linear_toleranceの値をゆるくした。
        void updateCurrentWaypoint2(){
            if (path_.empty()) {
                return;
            }
            if (current_waypoint_index_ >= static_cast<int>(path_.size()) - 1) {
                return;
            }
            double linear_error = std::hypot(
                path_[current_waypoint_index_ + 1].pose.position.x - pose_.x,
                path_[current_waypoint_index_ + 1].pose.position.y - pose_.y
            );

            if (linear_error < max_linear_tolerance) {
                current_waypoint_index_++;
                // RCLCPP_INFO(this->get_logger(), "Waypoint advanced");
            }
        }


        geometry_msgs::msg::Point lookaheadPoint(double L){
            
            geometry_msgs::msg::Point result;
            double idx = current_waypoint_index_;
            double remain_L = L;

            double linear_goal_x = path_[path_.size() -1].pose.position.x - pose_.x;
            double linear_goal_y = path_[path_.size() -1].pose.position.y - pose_.y;
            double linear_goal_distance = std::hypot(linear_goal_x, linear_goal_y);

            if (path_.empty()) {
                return result;
            }

            if (idx < 0) idx = 0;

            //path is longer than lookahead distance
            if (idx >= static_cast<int>(path_.size()) - 1) {
                scan_index_ = path_.size() - 1;
                return path_.back().pose.position;
            }

            if (linear_goal_distance < max_reaching_distance) {
                scan_index_ = path_.size() - 1;
                return path_.back().pose.position;
            }

            //search for lookahead point
            while (idx < static_cast<int>(path_.size()) - 1){
                double seg_x = path_[idx + 1].pose.position.x - path_[idx].pose.position.x;
                double seg_y = path_[idx + 1].pose.position.y - path_[idx].pose.position.y;
                double seg_len = std::hypot(seg_x, seg_y);

                //skip zero-length segment
                if (seg_len < 1e-6){
                    idx++;
                    continue;
                }

                //
                if (seg_len < remain_L){
                    remain_L = remain_L - seg_len;
                    idx++;
                    continue;
                }


                double ratio = remain_L / seg_len;
                
                result.x = path_[idx].pose.position.x + seg_x * ratio;
                result.y = path_[idx].pose.position.y + seg_y * ratio;
                result.z = 0.0;

                scan_index_ = idx + ratio;
                return result;
            }

            scan_index_ = path_.size() - 1;
            return path_.back().pose.position;
        }


        
        

        double computeDyanamicL(double speed, double curvature, double lat_error){
            double L = L0_ + k_v_ * speed - k_c_ * curvature - k_l_ * lat_error;
            if (L <= L_min_) L = L_min_;
            if (L >= L_max_) L = L_max_;
            RCLCPP_INFO(this->get_logger(), "L: %.2f", L);
            return L;
        }


        double estimateCurvature(double current_waypoint_index, double scan_index){
            int i = static_cast<int>(current_waypoint_index);
            int j = std::min(static_cast<int>(scan_index)+1, static_cast<int>(path_.size()) -1);
            int span = 5;

            if (i < 0 || j >= static_cast<int>(path_.size()) || i >= j) {
                return 0.0;
            }

            double Ax = path_[i+span].pose.position.x - path_[i].pose.position.x;
            double Ay = path_[i+span].pose.position.y - path_[i].pose.position.y;
            double Bx = path_[j+span].pose.position.x - path_[j].pose.position.x;
            double By = path_[j+span].pose.position.y - path_[j].pose.position.y;

            double norm_A = std::hypot(Ax, Ay);
            double norm_B = std::hypot(Bx, By);

            if (norm_A < 1e-6 || norm_B < 1e-6) {
                return 0.0;
            }

            return std::abs(Ax * Bx + Ay * By) / (norm_A * norm_B);

        }


        //lateral error calculation
        double computeLateralError(){
            int i = current_waypoint_index_;
            if (i >= static_cast<int>(path_.size()) - 1) {
                return 0.0;
            }
            double seg_x = path_[i + 1].pose.position.x - path_[i].pose.position.x;
            double seg_y = path_[i + 1].pose.position.y - path_[i].pose.position.y;
            double seg = std::hypot(seg_x, seg_y);
    
            if (seg < 1e-6) return 0.0;

            double dx = pose_.x - path_[i].pose.position.x;
            double dy = pose_.y - path_[i].pose.position.y;

            double dirx = seg_x / seg;
            double diry = seg_y / seg;

            double cross = dirx * dy - diry * dx;
            return std::abs(cross);
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

            
            updateCurrentWaypoint2();
            

            //error calculation linear
            // span is to smooth the path direction
            double span = 5.0;
            double dx = path_[scan_index_].pose.position.x - pose_.x;
            double dy = path_[scan_index_].pose.position.y - pose_.y;
            double tx = path_[scan_index_ + span].pose.position.x - path_[scan_index_].pose.position.x;
            double ty = path_[scan_index_ + span].pose.position.y - path_[scan_index_].pose.position.y;
            double norm = std::hypot(tx, ty);
            if (norm > 0) {
                tx /= norm;
                ty /= norm;
            }
            double nx = -ty;
            double ny = tx;
            double error_tan = dx * tx + dy * ty;
            double error_norm = dx * nx + dy * ny;
            double linear_error = std::hypot(dx, dy);
            double linear_goal_x = path_[path_.size() -1].pose.position.x - pose_.x;
            double linear_goal_y = path_[path_.size() -1].pose.position.y - pose_.y;
            double linear_goal_distance = std::hypot(linear_goal_x, linear_goal_y);


            //quoternion to yaw
            tf2::Quaternion q(
                path_[scan_index_].pose.orientation.x,
                path_[scan_index_].pose.orientation.y,
                path_[scan_index_].pose.orientation.z,
                path_[scan_index_].pose.orientation.w
            );

            double roll, pitch, target_theta;
            tf2::Matrix3x3(q).getRPY(roll, pitch, target_theta);

            double theta_error = target_theta - pose_.theta;
            while (theta_error > M_PI) theta_error -= 2*M_PI;
            while (theta_error < -M_PI) theta_error += 2*M_PI;

            tf2::Quaternion q_goal(
                path_[path_.size() - 1].pose.orientation.x,
                path_[path_.size() - 1].pose.orientation.y,
                path_[path_.size() - 1].pose.orientation.z,
                path_[path_.size() - 1].pose.orientation.w
            );

            double roll_goal, pitch_goal, yaw_goal;
            tf2::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);

            double theta_goal = yaw_goal - pose_.theta;
            while (theta_goal >= M_2_PI) theta_goal -= M_2_PI;
            while (theta_goal < 0) theta_goal += M_2_PI;

            
            printWayPointArrow(path_[current_waypoint_index_].pose, path_[path_.size()-1].pose);



            if ((linear_goal_distance < max_reaching_distance)){ //&& theta_goal < max_reaching_theta) {
                //goal reached
                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                publishZero();
                auto result_msg = std::make_shared<inrof2025_ros_type::action::Follow::Result>();
                result_msg->success = true;
                goal_handle_->succeed(result_msg);
                goal_handle_.reset();
                return;
            }

            //PID control for linear speed
            double linear_cmd_tan = linear_PID_tan_.compute(error_tan, 0.0);
            double linear_cmd_norm = linear_PID_norm_.compute(error_norm, 0.0);
            
            //PID control for theta speed
            double theta_speed_cmd = omega_PID_.compute(target_theta, pose_.theta);


            //convert to x,y speed
            double linear_speed_cmd_x = linear_cmd_tan * tx + linear_cmd_norm * nx;
            double linear_speed_cmd_y = linear_cmd_tan * ty + linear_cmd_norm * ny;

            double linear_speed_norm = std::hypot(linear_speed_cmd_x, linear_speed_cmd_y);

            if (linear_goal_distance < max_reaching_distance + min_approach_distance){
                linear_speed_norm = std::max(linear_speed_norm, min_linear_speed);
            }

            if (linear_speed_norm <= 0) return;
            double scale = linear_speed_norm / std::hypot(linear_speed_cmd_x, linear_speed_cmd_y);
            linear_speed_cmd_x *= scale;
            linear_speed_cmd_y *= scale;

            //後で、curvatureも考慮するようにする
            lookahead_distance_ = computeDyanamicL(linear_speed_norm, estimateCurvature(current_waypoint_index_, scan_index_), computeLateralError());



            geometry_msgs::msg::Point lookahead_point = lookaheadPoint(lookahead_distance_);
            double index = scan_index_;
            
            
            //後で、lateral errorも考慮するようにする




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


            printCmdVelArrow(linear_speed_cmd_x, linear_speed_cmd_y, clipped_v_x_f, clipped_v_y_f);

            
            //publish feedback
            auto feedback_msg = std::make_shared<inrof2025_ros_type::action::Follow::Feedback>();
            feedback_msg->x = pose_.x;
            feedback_msg->y = pose_.y;
            feedback_msg->theta = pose_.theta;
            goal_handle_->publish_feedback(feedback_msg);



            
           // --- ★ここから修正版：RVizに表示するためのMarker publish ---
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // TFに合わせる
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoint_marker";
            marker.id = 0; // ← 常に同じIDを使うことで「上書き表示」できる！
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = path_[current_waypoint_index_].pose.position.x;
            marker.pose.position.y = path_[current_waypoint_index_].pose.position.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // 点の大きさ
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;

            // 色：赤
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // lifetimeを少し短くして上書き更新を確実にする
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            marker_pub_->publish(marker);
            // --- ★ここまで修正版 ---

            // --- Lookahead point marker (for RViz) ---
            visualization_msgs::msg::Marker marker2;
            marker2.header.frame_id = "map";
            marker2.header.stamp = this->get_clock()->now();
            marker2.ns = "lookahead_marker";
            marker2.id = 0;  // 同じIDで上書き表示
            marker2.type = visualization_msgs::msg::Marker::SPHERE;
            marker2.action = visualization_msgs::msg::Marker::ADD;

            marker2.pose.position.x = lookahead_point.x;
            marker2.pose.position.y = lookahead_point.y;
            marker2.pose.position.z = 0.0;
            marker2.pose.orientation.w = 1.0;

            // サイズ（色違いの球）
            marker2.scale.x = 0.15;
            marker2.scale.y = 0.15;
            marker2.scale.z = 0.15;

            // 色：青
            marker2.color.r = 0.0;
            marker2.color.g = 0.0;
            marker2.color.b = 1.0;
            marker2.color.a = 1.0;

            // lifetime
            marker2.lifetime = rclcpp::Duration::from_seconds(0.2);

            // publish
            lookahead_marker_pub_->publish(marker2);


        }

        void printWayPointArrow(geometry_msgs::msg::Pose waypoint_pose, geometry_msgs::msg::Pose goal_pose) {
            visualization_msgs::msg::Marker arrow;

            // pub waypoint pose
            arrow.header.frame_id = "map";
            arrow.ns = "way_point_arrow";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = waypoint_pose;
            arrow.scale.x = 0.08;
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 0.0f;
            arrow.color.b = 1.0f;
            arrow.color.a = 1.0f;
            pose_arrow_pub_ -> publish(arrow);

            // pub goal pose
            arrow.header.frame_id = "map";
            arrow.ns = "goal_point_arrow";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = goal_pose;
            arrow.scale.x = 0.08;
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;
            pose_arrow_pub_ -> publish(arrow);
        }

        void printCmdVelArrow(double vx, double vy, double cliped_vx, double cliped_vy) {
            // convert pose2d to pose
            geometry_msgs::msg::Pose pose;
            pose.position.x = pose_.x;
            pose.position.y = pose_.y;
            pose.position.z = 0.0;
            tf2::Quaternion q;
            double yaw;
            if (std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6) {
                yaw = 0;
            } else {
                yaw = std::atan2(vy, vx);
            }
            q.setRPY(0, 0, yaw);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            // pub raw cmd vel 
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.ns = "cmd_vel";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = pose;
            arrow.scale.x = std::hypot(vx, vy);
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 0.0f;
            arrow.color.b = 1.0f;
            arrow.color.a = 0.5f;
            cmd_vel_arrow_pub->publish(arrow);

            // pub cliped cmd_vel
            arrow.header.frame_id = "map";
            arrow.ns = "cliped_cmd_vel";
            arrow.id = 0;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose = pose;
            arrow.scale.x = std::hypot(cliped_vx, cliped_vy);
            arrow.scale.y = 0.04;
            arrow.scale.z = 0.04;

            arrow.color.r = 0.0f;
            arrow.color.g = 1.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0f;
            cmd_vel_arrow_pub->publish(arrow);
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
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_arrow_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vel_arrow_pub;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        std::mutex mutex_;
        geometry_msgs::msg::Pose2D pose_;

        //PID control
        PIDController linear_PID_tan_, linear_PID_norm_, omega_PID_;

        

        //tolerance and reaching distance
        double max_linear_tolerance;
        double max_theta_tolerance;
        double max_reaching_distance;
        double max_reaching_theta;
        double min_approach_distance;
        double min_linear_speed = 0.05;

        //dynamic lookahead parameters
        double L_min_;
        double L_max_;
        double L0_;
        double k_v_;
        double k_c_;
        double k_l_;

        // waypoint index
        double current_waypoint_index_ = 0;    
        double scan_index_ = 0.1;
        int x_;
        
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