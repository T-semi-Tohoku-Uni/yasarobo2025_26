#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <functional>
#include <cstring> 


namespace raspi{
    class color: public rclcpp::Node{
        public:
            explicit color(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("color", options) {
                color_value = open_serial("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.2");

                auto sendQ = rclcpp::QoS(rclcpp::KeepLast(10));
                subColor_ = this->create_publisher<std_msgs::msg::UInt8>("/color", sendQ);
                receive_timer_ = this->create_wall_timer(
                    std::chrono::microseconds(10), std::bind(&color::receive_color_callback, this)
                );


            }
        
        private:
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
                tty.c_cflag &= ~PARENB;      // パリティなし
                tty.c_cflag &= ~CSTOPB;      // ストップビット 1
                tty.c_cflag &= ~CRTSCTS;     // ハードウェアフロー制御なし
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

            void receive_color_callback() {
                uint8_t tmp[256];
                ssize_t n = read(color_value, tmp, sizeof(tmp));
                static constexpr uint8_t DELTM[] = {'\r', '\n'};
                
                if (n > 0) {
                    // 受信データをバッファに追加
                    recev_buffer_.insert(recev_buffer_.end(), tmp, tmp+n);

                    // バッファ内に \r\n があるか検索
                    while (1) {
                        std::vector<uint8_t>::iterator it_delim = std::search(
                            recev_buffer_.begin(), recev_buffer_.end(),
                            std::begin(DELTM), std::end(DELTM)
                        );

                        // \r\n が見つからなければループを抜ける
                        if (it_delim == recev_buffer_.end()) break;

                        // \r\n の前のデータ長を計算
                        std::size_t frame_len = std::distance(recev_buffer_.begin(), it_delim);

                        // データ長が 1 (1バイトデータ + \r\n) の場合
                        if (frame_len == 1) {
                            // 1文字目（データ本体）を取得
                            uint8_t data_byte = recev_buffer_[0];
        
                            // メッセージを作成 (std_msgs::msg::UInt8型)
                            auto msg = std_msgs::msg::UInt8();
                            msg.data = data_byte;
        
                            // subColor_ でパブリッシュ
                            subColor_->publish(msg);
                        
                        } 

                        // 処理したフレーム (データ + \r\n) をバッファから削除
                        // (frame_lenが1でなくても、不正なデータとして削除する)
                        recev_buffer_.erase(recev_buffer_.begin(), it_delim + 2);
                    }
                }
            }



        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr subColor_;
        rclcpp::TimerBase::SharedPtr receive_timer_;
        std::vector<uint8_t> recev_buffer_;
        int color_value;

    };
} // namespace raspi

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<raspi::color>());
    rclcpp::shutdown();
    return 0;
}