#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <geometry_msgs/msg/twist.hpp>

namespace raspi {
    typedef union {
        uint8_t byte[4];
        float value;
    } U32Bytes;

    typedef struct MotorVel {
        float v1;
        float v2;
        float v3;
    } MotorVel;

    class CmdVel: public rclcpp::Node {
        public:
            explicit CmdVel(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("cmd_vel_feedback", options) {
                fd_ = open_serial("/dev/ttyACM0");
                r_ = 0.14;
                rclcpp::QoS feedbackQ(rclcpp::KeepLast(10));
                pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_feedback", feedbackQ);
                receive_timer_ = this->create_wall_timer(
                    std::chrono::microseconds(10), std::bind(&CmdVel::receive_callback, this)
                );
                rclcpp::QoS sendQ(rclcpp::KeepLast(10));
                sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel", sendQ, std::bind(&CmdVel::send, this, std::placeholders::_1)
                );
            }
        private:
            void send(geometry_msgs::msg::Twist::SharedPtr msg) {
                U32Bytes u32_bytes[3];
                uint8_t buf[14];
                memset(buf, 0x00, sizeof(buf));
                float vel_x = msg->linear.x;
                float vel_y = msg->linear.y;
                float vel_theta = msg->angular.z;

                MotorVel motor_vel = forwardKinematics(vel_x, vel_y, vel_theta);
    
                u32_bytes[0].value = motor_vel.v1;
                u32_bytes[1].value = motor_vel.v2;
                u32_bytes[2].value = motor_vel.v3;

                for (int i=0; i<3; i++ ) {
                    std::memcpy(
                        buf+i*4,
                        u32_bytes[i].byte,
                        4
                    );
                }
                buf[12] = '\r';
                buf[13] = '\n';

                ::write(fd_, buf, 14);
            }
            void receive_callback() {
                uint8_t tmp[256];
                ssize_t n = read(fd_, tmp, sizeof(tmp));
                static constexpr uint8_t DELTM[] = {'\r', '\n'};
                
                if (n > 0) {
                    recev_buffer_.insert(recev_buffer_.end(), tmp, tmp+n);
                    float cmd_feedback[3];

                    // search \r\n
                    while (1) {
                        std::vector<uint8_t>::iterator it_delim = std::search(
                            recev_buffer_.begin(), recev_buffer_.end(),
                            std::begin(DELTM), std::end(DELTM)
                        );

                        if (it_delim == recev_buffer_.end()) break;

                        std::size_t frame_len = std::distance(recev_buffer_.begin(), it_delim);

                        if (frame_len != 12) {
                            recev_buffer_.erase(recev_buffer_.begin(), it_delim+2);
                            continue;
                        }

                        std::array<uint8_t, 12> frame;
                        std::copy_n(recev_buffer_.begin(), 12, frame.begin());

                        recev_buffer_.erase(recev_buffer_.begin(), it_delim+2);

                        for (int i=0; i<3; i++ ) {
                            U32Bytes u32_byte;
                            std::copy_n(
                                frame.begin()+i*4,
                                4,
                                u32_byte.byte
                            );
                            
                            cmd_feedback[i] = u32_byte.value;
                        }

                        // caculate x, y, theta
                        // TODO: 時間付きで渡してあげたい気持ち
                        geometry_msgs::msg::Twist twist = inverseKinematics(cmd_feedback[0], cmd_feedback[1], cmd_feedback[2]);
                        // twist.linear.x = cmd_feedback[0];
                        // twist.linear.y = cmd_feedback[1];
                        // twist.angular.z = cmd_feedback[2];
                        pub_->publish(twist);
                    }
                }
            }


            int open_serial(const char *device_name)
            {
                // 1. オープン（ノンブロッキングで open → 後からブロッキングモードに切り替え）
                int fd = ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: could not open %s (%s)",
                                 device_name, std::strerror(errno));
                    return -1;
                }
                // ノンブロックをクリアしてブロッキングに
                fcntl(fd, F_SETFL, 0);
            
                // 2. 現在の端末設定を取得
                struct termios tty;
                if (tcgetattr(fd, &tty) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: tcgetattr error (%s)", std::strerror(errno));
                    ::close(fd);
                    return -1;
                }
            
                // 3. ボーレート設定 (入力／出力ともに 115200)
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);
            
                // 4. RAW モード設定
                cfmakeraw(&tty);
            
                // 5. フラグ設定
                //  - CS8: 8 ビットデータ
                //  - CLOCAL: ローカルライン (モデム制御なし)
                //  - CREAD: 受信有効
                tty.c_cflag &= ~PARENB;        // パリティなし
                tty.c_cflag &= ~CSTOPB;        // ストップビット 1
                tty.c_cflag &= ~CRTSCTS;       // ハードウェアフロー制御なし
                tty.c_cflag |= (CS8 | CLOCAL | CREAD);
            
                // 6. 非同期読み出し設定 (VMIN/VTIME)
                //    VMIN=0, VTIME=0 → read() が即リターン（バイトがなければ0を返す）
                tty.c_cc[VMIN]  = 0;
                tty.c_cc[VTIME] = 0;
            
                // 7. 設定を反映
                if (tcsetattr(fd, TCSANOW, &tty) != 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                                 "Serial Fail: tcsetattr error (%s)", std::strerror(errno));
                    ::close(fd);
                    return -1;
                }
            
                // 8. 入出力バッファをクリア
                tcflush(fd, TCIOFLUSH);
            
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                            "Serial opened: %s @ 115200, 8N1, raw", device_name);
                return fd;
            }


            MotorVel forwardKinematics(float vx, float vy, float vtheta) {
                MotorVel motor_vel;
                motor_vel.v1 = vx + r_*vtheta;
                motor_vel.v2 = 0.5 * vx + std::sqrt(3)/2*vy - r_*vtheta;
                motor_vel.v3 = -0.5 * vx + std::sqrt(3)/2*vy + r_*vtheta;
                return motor_vel;
            }

            geometry_msgs::msg::Twist inverseKinematics(float v1, float v2, float v3) {
                geometry_msgs::msg::Twist twist;
                twist.linear.x = v2 - v3;
                twist.linear.y = -2/std::sqrt(3)*v1 + std::sqrt(3)*v2 -1/std::sqrt(3)*v3;
                twist.angular.z = 1/r_*v1 - 1/r_*v2 + 1/r_*v3;
                return twist;
            }

            int fd_;
            float r_;
            std::vector<uint8_t> recev_buffer_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
            rclcpp::TimerBase::SharedPtr receive_timer_;
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raspi::CmdVel>());
    rclcpp::shutdown();
    return 0;
}