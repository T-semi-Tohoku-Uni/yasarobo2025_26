#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

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
                this->declare_parameter<double>("Kp_linear", 0.00);
                this->declare_parameter<double>("Kp_angular", 0.00);
                this->declare_parameter<double>("max_linear_acceleration", 0.10);
                this->declare_parameter<double>("max_angular_acceleration", 0.00);
                this->get_parameter("Kp_linear", Kp_linear);
                this->get_parameter("Kp_angular", Kp_angular);
                this->get_parameter("max_linear_acceleration", max_linear_acceleration);
                this->get_parameter("max_angular_acceleration", max_angular_acceleration);
                fd_vel_ = open_serial("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.2");
                r_ = 0.14;
                auto feedbackQ = rclcpp::QoS(rclcpp::KeepLast(10));
                pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_feedback", feedbackQ);
                receive_timer_ = this->create_wall_timer(
                    std::chrono::microseconds(10), std::bind(&CmdVel::receive_vel_callback, this)
                );
                rclcpp::QoS sendQ(rclcpp::KeepLast(10));
                sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel", sendQ, std::bind(&CmdVel::accelerationControl, this, std::placeholders::_1)
                );
                // TODO
                control_timer_ = this->create_wall_timer(
                    std::chrono::microseconds(20), std::bind(&CmdVel::cascadeControl, this)
                );
            }
        private:

            //dtは後で考える
            void accelerationControl(geometry_msgs::msg::Twist::SharedPtr msg) {
                
                cmd_vel_prime_ = *msg;

                dt = 0.02;

                //input linear velocity 
                double vx_0 = cmd_vel_prime_.linear.x;
                double vy_0 = cmd_vel_prime_.linear.y;
                double omega_0 = cmd_vel_prime_.angular.z;

                //previous linear velocity
                double vx_prev = cmd_vel_.linear.x;
                double vy_prev = cmd_vel_.linear.y;
                double omega_prev = cmd_vel_.angular.z;

                double v_target_abs= std::hypot(vx_0, vy_0);
                double v_prev_abs = std::hypot(vx_prev, vy_prev);

                //max delta_v tolerance
                double max_delta_v = max_linear_acceleration * dt;
                double max_delta_omega = max_angular_acceleration * dt;

                double delta_v = v_target_abs - v_prev_abs;
                double delta_omega = omega_0 - omega_prev;

                if (std::abs(delta_v) > max_delta_v){

                    delta_v = (delta_v > 0 ? max_delta_v : -max_delta_v);
                }

                double v_new_abs = v_prev_abs + delta_v;

                //Limit linear acceleration
                double vx_new = 0.0;
                double vy_new = 0.0;

                //non 0
                if (v_target_abs > 1e-6){
                    vx_new = v_new_abs * (vx_0 / v_target_abs);
                    vy_new = v_new_abs * (vy_0 / v_target_abs);
                } else if (v_prev_abs > 1e-6) {
                    vx_new = v_new_abs * (vx_prev / v_prev_abs);
                    vy_new = v_new_abs * (vy_prev / v_prev_abs);
                }            

                //Limit angular acceleration
                double delata_omeaga = omega_0 - omega_prev;
                if (std::abs(delta_omega) > max_delta_omega) {
                    delta_omega = (delta_omega > 0 ? max_delta_omega : -max_delta_omega);
                }
                double omega_new = omega_prev + delta_omega;


                cmd_vel_.linear.x = vx_new;
                cmd_vel_.linear.y = vy_new;
                cmd_vel_.angular.z = omega_new;

               // RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f", msg->linear.x, vx_0, vx_prev, v_target_abs, v_new_abs, v_new_abs);
            }
        


            void cascadeControl(){ 

                double vx_target = cmd_vel_.linear.x;
                double vy_target = cmd_vel_.linear.y;
                double omega_target = cmd_vel_.angular.z;

                double vx_cur = cmd_vel_feedback_.linear.x;
                double vy_cur = cmd_vel_feedback_.linear.y;
                double omega_cur = cmd_vel_feedback_.angular.z;

                double error_vx = vx_target - vx_cur;
                double error_vy = vy_target - vy_cur;
                double error_omega = omega_target - omega_cur;

                double vx = vx_target + Kp_linear * error_vx;
                double vy = vy_target + Kp_linear * error_vy;
                double omega = omega_target + Kp_angular * error_omega;
            
                geometry_msgs::msg::Twist twist;
                twist.linear.x = vx;
                twist.linear.y = vy;
                twist.angular.z = omega;

                sendVel(twist);
            }


            void sendVel(geometry_msgs::msg::Twist msg) {
                U32Bytes u32_bytes[3];
                uint8_t buf[14];
                memset(buf, 0x00, sizeof(buf));
                float vel_x = msg.linear.x;
                float vel_y = msg.linear.y;
                float vel_theta = msg.angular.z;

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

                ::write(fd_vel_, buf, 14);
            }
            void receive_vel_callback() {
                uint8_t tmp[256];
                ssize_t n = read(fd_vel_, tmp, sizeof(tmp));
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
                        cmd_vel_feedback_ = twist;
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

            int fd_vel_;
            float r_;
            double Kp_linear, Kp_angular;
            double max_linear_acceleration;
            double max_angular_acceleration;
            double dt;
            std::vector<uint8_t> recev_buffer_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
            rclcpp::TimerBase::SharedPtr receive_timer_;
            rclcpp::TimerBase::SharedPtr control_timer_;
            geometry_msgs::msg::Twist cmd_vel_;
            geometry_msgs::msg::Twist cmd_vel_prime_;
            geometry_msgs::msg::Twist cmd_vel_feedback_;
            
    };
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raspi::CmdVel>());
    rclcpp::shutdown();
    return 0;
}