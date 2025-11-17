#include <rclcpp/rclcpp.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>
#include <inrof2025_ros_type/srv/ball_color.hpp>

namespace dummy {
    class Vacume: public rclcpp::Node {
        public:
            explicit Vacume(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("vacume", options) {
                srvVacume_ = this->create_service<inrof2025_ros_type::srv::Vacume> (
                    "/srv/vacume",
                    std::bind(&Vacume::vacumeCallback, this, std::placeholders::_1, std::placeholders::_2)
                );

                srvColor_ = this->create_service<inrof2025_ros_type::srv::BallColor> (
                    "color",
                    std::bind(&Vacume::colorCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
            }
        private:
            void vacumeCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Response> response
            ) {}

            void colorCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::BallColor::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::BallColor::Response> response
            ) {
                response->color = 1;
            }

            rclcpp::Service<inrof2025_ros_type::srv::Vacume>::SharedPtr srvVacume_;
            rclcpp::Service<inrof2025_ros_type::srv::BallColor>::SharedPtr srvColor_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dummy::Vacume>());
    rclcpp::shutdown();
    return 0;
}