#include "ball_detect_node.hpp"

DBSCAN::BallDetect::BallDetect(const rclcpp::NodeOptions & options): Node("ball_detect_node", options) {
    // parameter
    this->declare_parameter<std::string>("frame_id", "ldlidar_base");
    this->declare_parameter<double>("eps", 0.1);
    this->declare_parameter<double>("min_pts", 10);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("eps", EPS_);
    this->get_parameter("min_pts", MIN_PTS_);

    // subscribe topic
    rclcpp::SensorDataQoS lidarScanQos = rclcpp::SensorDataQoS();
    this->subLider_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/ldlidar_node/scan", lidarScanQos, std::bind(&BallDetect::lidarCallback, this, std::placeholders::_1)
    );

    // publish topic
    rclcpp::SensorDataQoS clustersQos = rclcpp::SensorDataQoS();
    this->pubClusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/clusters", clustersQos
    );

    const char *sim = std::getenv("WITH_SIM");
    if (!sim || std::string(sim) != "1") is_sim_ = false;
    else is_sim_ = true;
    RCLCPP_INFO(this->get_logger(), "WITH SIM env is %d", is_sim_);
}

void DBSCAN::BallDetect::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->scan_ = msg; // TODO: mutex and sharedPtr

    detect();
}

geometry_msgs::msg::Pose2D DBSCAN::BallDetect::detect() {
    geometry_msgs::msg::Pose2D ball_pose;

    if (!scan_) {
        RCLCPP_WARN(this->get_logger(), "scan_ is empty, so skip detect function");
        return ball_pose;
    }

    /*
        construct KD-tree
    */
    // convert LaserScan to PointCloud
    PointCloud point_cloud = scan2Point(*scan_);
    // construct KD-tree
    DBSCAN::KdTree tree(
        2,
        point_cloud,
        nanoflann::KDTreeSingleIndexAdaptorParams(1)
    );
    tree.buildIndex();

    /*
        execute dbscan
    */
    DBSCAN::BallDetect::dbscan(point_cloud.points, tree);

    return ball_pose;
}

void DBSCAN::BallDetect::dbscan(std::vector<DBSCAN::Point> &points, DBSCAN::KdTree &tree) {
    int C = 0;
    for (DBSCAN::Point &p: points) {
        if (p.getID() != DBSCAN::ClusterID::UNVISITED) continue; // Not marked
        std::vector<size_t> neighbors = DBSCAN::BallDetect::regionQuery(p, tree); // TODO: use set
        if (neighbors.size() < MIN_PTS_) p.setID(DBSCAN::ClusterID::NOISE);
        else {
            expandCluster(p, points, neighbors, tree, C);
            C++;
        }  
    }

    this->pubClusters_->publish(point2PointCloud2(points));
}

void DBSCAN::BallDetect::expandCluster(
    Point &p,
    std::vector<DBSCAN::Point> &points,
    std::vector<size_t> &neighbors_p,
    DBSCAN::KdTree& tree,
    const int cluster_id
) {
    // set cluster of p
    p.setID(cluster_id);
    
    // search same cluster point
    for (size_t neighbors_idx=0; neighbors_idx<neighbors_p.size(); neighbors_idx++ ) {
        DBSCAN::Point &q = points[neighbors_p[neighbors_idx]];

        // search core node
        if (q.getID() == DBSCAN::ClusterID::UNVISITED) {
            // get neighbors
            std::vector<size_t> neighbors_q = DBSCAN::BallDetect::regionQuery(q, tree);
            if (neighbors_q.size() >= MIN_PTS_) {
                for (int neighbors_q_idx: neighbors_q) {
                    neighbors_p.push_back(neighbors_q_idx);
                }
            }
        }

        // when q isn't have any cluster
        // (borderline node)
        if (q.getID() < 0) {
            q.setID(cluster_id);
        }
    }
}

std::vector<size_t> DBSCAN::BallDetect::regionQuery(
    DBSCAN::Point &p, 
    DBSCAN::KdTree& tree
) {
    const float radius = EPS_*EPS_;
    std::vector<std::pair<uint32_t, float>> matches;

    const float query_pt[2] = { p.getX(), p.getY() };
    nanoflann::SearchParams params;
    tree.radiusSearch(&query_pt[0], radius, matches, params);

    std::vector<size_t> neighbors;
    neighbors.reserve(matches.size());

    for (std::pair<uint32_t, float> &m : matches) {
        const uint32_t idx = m.first;
        neighbors.push_back(idx);
    }

    return neighbors;
}

DBSCAN::PointCloud DBSCAN::BallDetect::scan2Point(const sensor_msgs::msg::LaserScan scan) {
    DBSCAN::PointCloud point_cloud;
    for (size_t i=0; i<scan.ranges.size(); i++) {
        double r, theta;
        r = scan.ranges[i];
        if (std::isnan(r) || r<scan.range_min || scan.range_max<r) continue;
        if (is_sim_) theta = scan.angle_min + ((std::double_t)(i))*scan.angle_increment;
        else theta = scan.angle_min + ((std::double_t)(i))*scan.angle_increment - 3.0*M_PI/2.0;

        DBSCAN::Point p(r*cos(theta), r*sin(theta));
        point_cloud.points.push_back(p);
    }
    return point_cloud;
}

sensor_msgs::msg::PointCloud2 DBSCAN::BallDetect::point2PointCloud2(
    const std::vector<DBSCAN::Point> &points
) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = rclcpp::Clock().now();
    cloud.height = 1;
    cloud.width = points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

    // ---- クラスタごとの点数をカウント ----
    std::unordered_map<int, int> cluster_counts;
    for (const Point &p : points) {
        cluster_counts[p.getID()]++;
    }

    // ---- クラスタを点数の多い順にソート ----
    std::vector<std::pair<int, int>> sorted_clusters(cluster_counts.begin(), cluster_counts.end());
    std::sort(sorted_clusters.begin(), sorted_clusters.end(),
              [](auto &a, auto &b) { return a.second > b.second; });

    // ---- 固定色パレット（お好みで変更可）----
    std::vector<std::array<uint8_t, 3>> color_palette = {
        {255, 0, 0},     // red
        {0, 255, 0},     // green
        {0, 0, 255},     // blue
        {255, 255, 0},   // yellow
        {255, 0, 255},   // magenta
        {0, 255, 255},   // cyan
        {255, 128, 0},   // orange
        {128, 0, 255},   // purple
        {128, 128, 128}, // gray
        {0, 128, 255}    // light blue
    };

    // ---- クラスタごとに色を固定割り当て ----
    std::map<int, std::array<uint8_t, 3>> cluster_colors;
    size_t color_idx = 0;
    for (auto &pair : sorted_clusters) {
        int cluster_id = pair.first;
        if (cluster_id < 0) { // Noiseや未分類は灰色に
            cluster_colors[cluster_id] = {128, 128, 128};
            continue;
        }
        cluster_colors[cluster_id] = color_palette[color_idx % color_palette.size()];
        color_idx++;
    }

    // ---- PointCloud2 に書き込み ----
    for (const Point &p : points) {
        *iter_x = p.getX();
        *iter_y = p.getY();
        *iter_z = 0.0f;

        auto color = cluster_colors[p.getID()];
        *iter_r = color[0];
        *iter_g = color[1];
        *iter_b = color[2];

        ++iter_x; ++iter_y; ++iter_z;
        ++iter_r; ++iter_g; ++iter_b;
    }

    return cloud;
}

DBSCAN::Point::Point(float x, float y, int clusterID) {
    this->x_ = x;
    this->y_ = y;
    this->clusterID_ = clusterID;
}

size_t DBSCAN::PointCloud::kdtree_get_point_count() const {
    return points.size();
}

double DBSCAN::PointCloud::kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0) return points[idx].getX();
    else return points[idx].getY();
}

template <class BBOX>
bool DBSCAN::PointCloud::kdtree_get_bbox(BBOX&) const {
    return false;
}

float DBSCAN::Point::getX() const {
    return x_;
}

float DBSCAN::Point::getY() const {
    return y_;
}

int DBSCAN::Point::getID() const {
    return clusterID_;
}

void DBSCAN::Point::setID(int id) {
    clusterID_ = id;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DBSCAN::BallDetect>());
    rclcpp::shutdown();
    return 0;
}