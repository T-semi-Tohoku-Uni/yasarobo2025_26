#include <field.hpp>
#include <yaml-cpp/yaml.h>

yasarobo2025_26::Field::Field(std::string dir, int threshold) {
    try {
        std::string yaml_path = dir + "map.yaml";
        YAML::Node lconf = YAML::LoadFile(dir + yaml_path);

        std::string map_path = dir + "map.pgm";
        cv::Mat mapImg = cv::imread(dir+"map.pgm", 0);
        mapImg_ = mapImg.clone();

        // read parameter
        mapResolution_ = lconf["resolution"].as<double>();
        mapOrigin_ = lconf["origin"].as<std::vector<double>>();
        mapWidth_ = mapImg.cols;
        mapHeight_ = mapImg.rows;

        for (int v=0; v<mapHeight_; v++) {
            for (int u=0; u<mapWidth_; u++) {
                uchar val = mapImg.at<uchar>(v, u);
                if (val >= threshold) {
                    mapImg_.at<uchar>(v, u) = 1;
                } else {
                    mapImg_.at<uchar>(v, u) = 0;
                }
            }
        }
    } catch (const YAML::Exception &e) {
        throw std::runtime_error(e.what());
    } catch (const std::exception &e) {
        throw std::runtime_error(e.what());
    }
}

cv::Mat yasarobo2025_26::Field::createDistanceField() {
    // create distance field
    cv::Mat distFieldF(mapHeight_, mapWidth_, CV_32FC1);
    cv::Mat distFieldD(mapHeight_, mapWidth_, CV_64FC1);
    cv::distanceTransform(this->mapImg_, distFieldF, cv::DIST_L2, 5);

    // convert grid distance to field distance
    for (int v=0; v<mapHeight_; v++ ) {
        for (int u=0; u<mapWidth_; u++ ) {
            double d = distFieldF.at<double>(v, u);
            distFieldD.at<double>(v,u) = (double)d * mapResolution_;
        }
    }

    return distFieldD;
}

double yasarobo2025_26::Field::getMapResolution() {
    return mapResolution_;
}

double yasarobo2025_26::Field::getMapHeight() {
    return mapHeight_;
}

double yasarobo2025_26::Field::getMapWidth() {
    return mapWidth_;
}


bool yasarobo2025_26::Field::isOnField(int u, int v) {
    if (u < 0 || v < 0 || u >= mapWidth_ || v >= mapHeight_) return false;
    if (mapImg_.at<uchar>(v, u) == 0) return false;
    return true;
}

bool yasarobo2025_26::Coordinate::isOnField() {
    return this->f_->isOnField(u_, v_);
}

yasarobo2025_26::Coordinate::Coordinate(std::shared_ptr<Field> f, int u, int v): u_(u), v_(v), f_(f) {
    uv2xy(u, v);
}

yasarobo2025_26::Coordinate::Coordinate(std::shared_ptr<Field> f, double x, double y): x_(x), y_(y), f_(f) {
    xy2uv(x, y);
}

void yasarobo2025_26::Coordinate::xy2uv(double x, double y) {
    u_ = (int32_t)(x / this->f_->getMapResolution());
    v_ = this->f_->getMapHeight() - 1 - (int32_t)(y / this->f_->getMapResolution());
}

void yasarobo2025_26::Coordinate::uv2xy(int u, int v) {
    throw std::logic_error("Coordinate::uv2xy() is not yet implemented.");
}