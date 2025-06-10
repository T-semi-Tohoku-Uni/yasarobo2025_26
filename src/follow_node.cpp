#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>


class FollowNode: public rclcpp::Node {
    public:
        explicit FollowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("follow_node", options) {
            this->declare_parameter<double>("lookahead_distance", 0.05);
            this->declare_parameter<double>("max_linear_speed", 0.2);
            this->declare_parameter<double>("max_angular_speed", 2.0);
            this->get_parameter("lookahead_distance", lookahead_distance_);
            this->get_parameter("max_linear_speed", max_linear_speed_);
            this->get_parameter("max_angular_speed", max_angular_speed_);

            rclcpp::QoS pathQos(rclcpp::KeepLast(5));
            path_sub_ = this->create_subscription<nav_msgs::msg::Path> (
                "route", pathQos, std::bind(&FollowNode::pathCallback, this, std::placeholders::_1)
            );
            rclcpp::QoS odomQos(rclcpp::KeepLast(5));
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
                "odom", odomQos, std::bind(&FollowNode::odomCallback, this, std::placeholders::_1)
            );
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&FollowNode::controlLoop, this)
            );
        }
    private:
        void pathCallback(nav_msgs::msg::Path msgs) {
            // std::lock_guard<std::mutex> lock(mutex_);
            path_ = msgs.poses;
            current_waypoint_index_ = 0;
        }
        void odomCallback(nav_msgs::msg::Odometry msgs) {
            // std::lock_guard<std::mutex> lock(mutex_);
            pose_.x = msgs.pose.pose.position.x;
            pose_.y = msgs.pose.pose.position.y;
            auto &q = msgs.pose.pose.orientation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            double yaw = std::atan2(siny_cosp, cosy_cosp);
            pose_.theta = yaw;
            RCLCPP_INFO(this->get_logger(), "%.4f %.4f", pose_.x, pose_.y);
        }
        void controlLoop() {
            // std::lock_guard<std::mutex> lock(mutex_);
            if (path_.empty()) {
                // publishZero();
                return;
            }

            size_t nearest_idx = 0;
            double min_dist_sq = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < path_.size(); ++i) {
                double dx = path_[i].pose.position.x - pose_.x;
                double dy = path_[i].pose.position.y - pose_.y;
                double d2 = dx*dx + dy*dy;
                if (d2 < min_dist_sq) {
                    min_dist_sq = d2;
                    nearest_idx = i;
                }
            }

            size_t target_index = nearest_idx;
            for (size_t i=nearest_idx; i<path_.size(); i++ ) {
                double dx   = path_[i].pose.position.x - pose_.x;
                double dy   = path_[i].pose.position.y - pose_.y;
                double dist = std::hypot(dx, dy);
                if (dist > lookahead_distance_) {
                    target_index = i;
                    break;
                }
            }

            RCLCPP_INFO(this->get_logger(), "%d", target_index);

            double tx = path_[target_index].pose.position.x - pose_.x;
            double ty = path_[target_index].pose.position.y - pose_.y;
            double x_r = cos(pose_.theta) * tx + sin(pose_.theta) * ty;
            double y_r = -sin(pose_.theta) * tx + cos(pose_.theta) * ty;
            
            double curvature = 0.0;
            if (x_r != 0) {
                curvature = (2.0 * y_r) / (lookahead_distance_ * lookahead_distance_);
            }

            double linear = max_linear_speed_;
            double angular = linear * curvature;
            if (angular > max_angular_speed_) angular = max_angular_speed_;
            if (angular < -max_angular_speed_) angular = -max_angular_speed_;
            // auto & goal_pose = path_.back().pose.position;
            // RCLCPP_INFO(this->get_logger(), "%.4f", goal_pose.x);

            double goal_dist = std::hypot(path_.back().pose.position.x - pose_.x, path_.back().pose.position.y - pose_.y);
            // RCLCPP_INFO(this->get_logger(), "%.4f", goal_dist);
            if (goal_dist < 0.1) {
                publishZero();
            } else {
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x  = linear;
                cmd.angular.z = angular;
                cmd_pub_->publish(cmd);
            }
            // geometry_msgs::msg::Twist cmd;
            // cmd.linear.x  = linear;
            // cmd.angular.z = angular;
            // try {
            //     cmd_pub_->publish(cmd);
            // } catch (const std::exception& e) {
            //     RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            // }
        }

        void publishZero()
        {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
        }

        double lookahead_distance_;
        double max_linear_speed_;
        double max_angular_speed_;

        // subscriber
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<geometry_msgs::msg::PoseStamped> path_;
        std::mutex mutex_;
        geometry_msgs::msg::Pose2D pose_;
        int current_waypoint_index_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}