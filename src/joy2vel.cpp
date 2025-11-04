#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace joy2Vel {
    class Joy2Vel: public rclcpp::Node {
        public:
            Joy2Vel(): Node("joy2vel") {
                rclcpp::QoS qos(rclcpp::KeepLast(10));
                sub_ = create_subscription<sensor_msgs::msg::Joy>(
                    "/joy", qos, std::bind(&Joy2Vel::callback, this, std::placeholders::_1)
                );
                pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
            }

        private:
            void callback(const sensor_msgs::msg::Joy::UniquePtr msg) {
                std::float_t leftJoyx_ = msg->axes[0];
                std::float_t leftJoyy_ = msg->axes[1];
                // std::float_t rightJoyx_ = msg->axes[2];
                std::float_t rightJoyy_ = msg->axes[3];

                geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
                twist.linear.set__x(leftJoyy_*0.1);
                twist.linear.set__y(leftJoyx_*0.1);
                twist.linear.set__z(0.0);
                twist.angular.set__x(0.0);
                twist.angular.set__y(0.0);
                twist.angular.set__z(rightJoyy_);
                pub_->publish(twist);
            }

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<joy2Vel::Joy2Vel>());
    rclcpp::shutdown();

    return 0;
}