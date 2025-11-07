#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

namespace yasarobo2025_26 {
    class Coordinate;
    class Field;

    class Coordinate {
        public:
            explicit Coordinate(std::shared_ptr<Field> f, int u, int v);
            explicit Coordinate(std::shared_ptr<Field> f, double x, double y);
            bool isOnField();
        private:
            void xy2uv(double x, double y);
            void uv2xy(int u, int v);
            int u_, v_;
            double x_, y_;
            std::shared_ptr<Field> f_;

            friend class Field;
    };

    class Field {
        public:
            Field(std::string dir, int threshold);
            cv::Mat createDistanceField();
            double getMapResolution();
            double getMapHeight();
            double getMapWidth();
        private:
            bool isOnField(int u, int v);

            cv::Mat mapImg_;
            std::vector<double> mapOrigin_;
            int threshold_;
            double mapResolution_, mapWidth_, mapHeight_;

            friend class Coordinate;
    };
}