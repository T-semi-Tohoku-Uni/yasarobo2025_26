#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <cstring> // std::strerror のため
#include <vector>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <functional>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <inrof2025_ros_type/srv/vacume.hpp>
#include <inrof2025_ros_type/srv/ball_color.hpp>


namespace raspi{
    class ColorVacumeNode: public rclcpp::Node{
        public:
            explicit ColorVacumeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
             : Node("color_vacume_node", options) {
                
                // --- 1. シリアルポートを1回だけ開く ---
                serial_fd_ = open_serial("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.2");
                if (serial_fd_ < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port. Shutting down.");
                    rclcpp::shutdown();
                    return;
                }

                // --- 2. color ノードの機能（Publisher と 受信タイマー） ---
                auto sendQ = rclcpp::QoS(rclcpp::KeepLast(10));
                pubColor_ = this->create_publisher<std_msgs::msg::UInt8>("/color", sendQ);
                
                receive_timer_ = this->create_wall_timer(
                    std::chrono::microseconds(100), std::bind(&ColorVacumeNode::receive_color_callback, this)
                );

                // --- 3. Vacume ノードの機能（Subscriber と Service） ---
                rclcpp::QoS callbackVacQ(rclcpp::KeepLast(10));
                subVac_ = this->create_subscription<std_msgs::msg::Bool>(
                    "/vac", callbackVacQ, std::bind(&ColorVacumeNode::sendVac, this, std::placeholders::_1)
                );

                srvVacume_ = this->create_service<inrof2025_ros_type::srv::Vacume> (
                    "/srv/vacume",
                    std::bind(&ColorVacumeNode::vacumeCallback, this, std::placeholders::_1, std::placeholders::_2)
                );

                srvColor_ = this->create_service<inrof2025_ros_type::srv::BallColor> (
                    "color",
                    std::bind(&ColorVacumeNode::colorCallback, this, std::placeholders::_1, std::placeholders::_2)
                );
            }
        
        private:
            void colorCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::BallColor::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::BallColor::Response> response
            ) {
                switch(color_) {
                    case 0:
                        RCLCPP_INFO(this->get_logger(), "RED");
                        break;
                    case 1:
                        RCLCPP_INFO(this->get_logger(), "YELLO");
                        break;
                    case 2:
                        RCLCPP_INFO(this->get_logger(), "BLUE");
                        break;
                }
                response->color = color_;
            }
            
            int open_serial(const char *device_name)
            {
                int fd = ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: could not open %s (%s)",
                                 device_name, std::strerror(errno));
                    return -1;
                }
                fcntl(fd, F_SETFL, 0);
            
                struct termios tty;
                if (tcgetattr(fd, &tty) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: tcgetattr error (%s)", std::strerror(errno));
                    ::close(fd);
                    return -1;
                }
            
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);
                cfmakeraw(&tty);
            
                tty.c_cflag &= ~PARENB;
                tty.c_cflag &= ~CSTOPB;
                tty.c_cflag &= ~CRTSCTS;
                tty.c_cflag |= (CS8 | CLOCAL | CREAD);
            
                tty.c_cc[VMIN]  = 0;
                tty.c_cc[VTIME] = 0;
            
                if (tcsetattr(fd, TCSANOW, &tty) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: tcsetattr error (%s)", std::strerror(errno));
                    ::close(fd);
                    return -1;
                }
            
                tcflush(fd, TCIOFLUSH);
            
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Serial opened: %s @ 115200, 8N1, raw", device_name);
                return fd;
            }

            // --- color ノードの受信コールバック ---
            void receive_color_callback() {
                uint8_t tmp[256];
                ssize_t n = read(serial_fd_, tmp, sizeof(tmp)); 
                static constexpr uint8_t DELTM[] = {'\r', '\n'};
                
                if (n > 0) {
                    recev_buffer_.insert(recev_buffer_.end(), tmp, tmp+n);

                    while (1) {
                        std::vector<uint8_t>::iterator it_delim = std::search(
                            recev_buffer_.begin(), recev_buffer_.end(),
                            std::begin(DELTM), std::end(DELTM)
                        );

                        if (it_delim == recev_buffer_.end()) break;

                        std::size_t frame_len = std::distance(recev_buffer_.begin(), it_delim);

                        if (frame_len == 1) {
                            uint8_t data_byte = recev_buffer_[0];
                            auto msg = std_msgs::msg::UInt8();
                            msg.data = data_byte - 48;
                            pubColor_->publish(msg);
                            color_ = msg.data;
                        } 

                        recev_buffer_.erase(recev_buffer_.begin(), it_delim + 2);
                    }
                }
            }

            void vacumeCallback(
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Request> request,
                const std::shared_ptr<inrof2025_ros_type::srv::Vacume::Response> response
            ) {
                
                (void)response;
                std_msgs::msg::Bool msg;
                msg.data = request->on;
                sendVac(msg);
            }

            //Vacumeノードの送信関数
            void sendVac(std_msgs::msg::Bool msg) {
                uint8_t buf[3];
                memset(buf, 0x00, sizeof(buf));

                if (msg.data) {
                    buf[0] = 0x31; // '1'
                } else {
                    buf[0] = 0x30; // '0'
                }

                buf[1] = 0x0d; // '\r'
                buf[2] = 0x0a; // '\n'

                RCLCPP_INFO(this->get_logger(), "Sending command: %c", buf[0]);
                ::write(serial_fd_, buf, 3); // <-- fd_vac_ や color_value ではなく serial_fd_ を使用
            }


            int serial_fd_; 

            // color用
            rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pubColor_;
            rclcpp::TimerBase::SharedPtr receive_timer_;
            std::vector<uint8_t> recev_buffer_;

            // Vacume用
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subVac_;
            rclcpp::Service<inrof2025_ros_type::srv::Vacume>::SharedPtr srvVacume_;
            rclcpp::Service<inrof2025_ros_type::srv::BallColor>::SharedPtr srvColor_;
            int color_;
    };
} // namespace raspi

// --- 共通の main 関数 ---
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raspi::ColorVacumeNode>());
    rclcpp::shutdown();
    return 0;
}