#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cmd_vel {
    class CmdVelFeedBack: public rclcpp::Node {
        public:
            explicit CmdVelFeedBack(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("cmd_vel_feedback", options) {
                rclcpp::QoS twistQos(rclcpp::KeepLast(10));
                rclcpp::QoS callbackQos(rclcpp::KeepLast(10));

                pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_feedback", twistQos);
                subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", callbackQos, std::bind(&CmdVelFeedBack::callback, this, std::placeholders::_1)
                );
            }
        
        private:
            void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                geometry_msgs::msg::Twist twist;
                twist.linear.x = msg->twist.twist.linear.x;
                twist.linear.y = msg->twist.twist.linear.y;
                twist.angular.z = msg->twist.twist.angular.z;
                pub_->publish(twist);
            }

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cmd_vel::CmdVelFeedBack>());
    rclcpp::shutdown();
    return 0;
}