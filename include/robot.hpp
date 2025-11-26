#include <rclcpp/rclcpp.hpp>

namespace yasarobo2025_26 {
    class Robot {
        public:
            Robot() = default;
            template<typename T>
            std::array<T, 3> forwardKinematics(T vx, T vy, T vtheta);
            template<typename T>
            std::array<T, 3> inverseKinematics(T v1, T v2, T v3);
        private:
    };
}