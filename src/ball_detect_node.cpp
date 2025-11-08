#include "ball_detect_node.hpp"

DBSCAN::BallDetect::BallDetect(const rclcpp::NodeOptions & options): Node("ball_detect_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // parameter
    this->declare_parameter<std::string>("frame_id", "ldlidar_base");
    this->declare_parameter<double>("eps", 0.1);
    this->declare_parameter<int>("min_pts", 10);
    this->declare_parameter<double>("diagonal_threshold", 0.1);
    this->declare_parameter<double>("wall_threshold", 0.01);
    this->declare_parameter<double>("diff_threshold", 1e-8);
    this->declare_parameter<double>("lidar_threshold", 1.0/5.0*M_PI);
    this->declare_parameter<double>("radius_threshold", 0.1);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("eps", EPS_);
    this->get_parameter("min_pts", MIN_PTS_);
    this->get_parameter("diagonal_threshold", DIAGONAL_THTRSHOLD_);
    this->get_parameter("wall_threshold", WALL_THTRSHOLD_);
    this->get_parameter("diff_threshold", DIFF_THTRSHOLD_);
    this->get_parameter("lidar_threshold", LIDAR_THTRSHOLD_);
    this->get_parameter("radius_threshold", RADIUS_THTRSHOLD_);

    // initialize field
    field_ = DBSCAN::Field("src/yasarobo2025_26/map/");

    // subscribe topic
    rclcpp::SensorDataQoS lidarScanQos = rclcpp::SensorDataQoS();
    this->subLider_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/ldlidar_node/scan", lidarScanQos, std::bind(&BallDetect::lidarCallback, this, std::placeholders::_1)
    );
    this->subPose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "/pose", 10, std::bind(&DBSCAN::BallDetect::poseCallback, this, std::placeholders::_1)
    );

    // publish topic
    rclcpp::SensorDataQoS clustersQos = rclcpp::SensorDataQoS();
    this->pubClusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/clusters", clustersQos
    );

    rclcpp::SensorDataQoS ballShapeQos = rclcpp::SensorDataQoS();
    this->pubBallShape_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ball_shape", ballShapeQos
    );

    // service
    srv_ball_pose_ = this->create_service<inrof2025_ros_type::srv::BallPose>(
        "ball_detect", std::bind(&DBSCAN::BallDetect::ballPoseCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    const char *sim = std::getenv("WITH_SIM");
    if (!sim || std::string(sim) != "1") is_sim_ = false;
    else is_sim_ = true;
    RCLCPP_INFO(this->get_logger(), "WITH SIM env is %d", is_sim_);
}

void DBSCAN::BallDetect::ballPoseCallback(
    const std::shared_ptr<inrof2025_ros_type::srv::BallPose::Request> request,
    const std::shared_ptr<inrof2025_ros_type::srv::BallPose::Response> response
) {
    std::optional<geometry_msgs::msg::Pose2D> ball_pose = DBSCAN::BallDetect::detect();
    if (!ball_pose) {
        RCLCPP_WARN(this->get_logger(), "No ball");
        response->detect = false;
        return;
    }

    // save response
    response->detect = true;
    response->x = ball_pose.value().x;
    response->y = ball_pose.value().y;

    RCLCPP_INFO(this->get_logger(), "Closest ball is (x, y)=(%f, %f)", response->x, response->y);
}

void DBSCAN::BallDetect::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->scan_ = msg; // TODO: mutex and sharedPtr

    detect();
}

void DBSCAN::BallDetect::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    if (this->pose_) {
        *(this->pose_) = *msg;
    } else {
        this->pose_ = std::make_unique<geometry_msgs::msg::Pose2D>(*msg);
    }
}

std::optional<geometry_msgs::msg::Pose2D> DBSCAN::BallDetect::detect() {
    if (!scan_) {
        RCLCPP_WARN(this->get_logger(), "scan_ is empty, so skip detect function");
        return std::nullopt; // no ball
    }

    /*
        construct KD-tree
    */
    // convert LaserScan to PointCloud, point_cloud origin is field.
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
    // get cluster
    std::unordered_map<int, std::vector<DBSCAN::Point>> clusters =
        DBSCAN::BallDetect::dbscan(point_cloud.points, tree);
    // delete wall cluster
    std::vector<int> ball_cluster_ids = DBSCAN::BallDetect::deleteWall(clusters);
    std::vector<std::pair<int, std::vector<DBSCAN::Point>>> ball_clusters = DBSCAN::BallDetect::collectBallPoints(
        clusters, ball_cluster_ids
    );
    // print rviz2
    this->pubClusters_->publish(point2PointCloud2(ball_clusters));

    /*
        decision target ball
    */
    // if ball_cluster is empty, continue searching ball
    if(ball_clusters.size() == 0) return std::nullopt; // no ball
    // caculate ball position
    std::vector<DBSCAN::Circle> ball_position;
    for (std::pair<int, std::vector<DBSCAN::Point>> &_ball: ball_clusters) {
        DBSCAN::Circle c = Circle(_ball.second);
        // check ball on field
        if (c.getR() > RADIUS_THTRSHOLD_) continue;
        if(this->isBallOnField(field_, c)) ball_position.push_back(Circle(_ball.second));
    }
    // find closest ball
    std::optional<geometry_msgs::msg::Pose2D> closest_ball = DBSCAN::BallDetect::findClosestBall(ball_position);
    if (!closest_ball) return std::nullopt; // no ball
    // print rviz2
    sensor_msgs::msg::PointCloud2 ball_point_cloud = DBSCAN::BallDetect::circle2PointCloud2(ball_position);
    this->pubBallShape_->publish(ball_point_cloud);

    return closest_ball;
}

std::optional<geometry_msgs::msg::Pose2D> DBSCAN::BallDetect::findClosestBall(
    std::vector<DBSCAN::Circle> &ball
) {
    geometry_msgs::msg::Pose2D closest_ball;

    if (ball.size() == 0) return std::nullopt;

    if (!pose_) {
        RCLCPP_WARN(this->get_logger(), "pose topic is empty");
        closest_ball.x = ball[0].getX();
        closest_ball.y = ball[0].getY();
        closest_ball.theta = 0.0;
        return closest_ball;
    }

    int closest_index=0;
    double min_distance = std::numeric_limits<double>::max();
    for (size_t i=0; i<ball.size(); i++) {
        double d = std::hypot(ball[i].getX()-pose_->x, ball[i].getY()-pose_->y);
        if (min_distance > d) {
            closest_index = i;
            min_distance = d;
        }
    }

    closest_ball.x = ball[closest_index].getX();
    closest_ball.y = ball[closest_index].getY();
    closest_ball.theta = 0.0;

    // mark closest ball
    ball[closest_index].markClosest();

    return closest_ball;
}

std::vector<std::pair<int, std::vector<DBSCAN::Point>>> DBSCAN::BallDetect::collectBallPoints(
    const std::unordered_map<int, std::vector<DBSCAN::Point>>& clusters,
    const std::vector<int>& ball_cluster_ids
){
    std::vector<std::pair<int, std::vector<DBSCAN::Point>>> ball_cluster;
    ball_cluster.reserve(ball_cluster_ids.size());

    for (size_t cid: ball_cluster_ids) {
        auto it = clusters.find(static_cast<int>(cid)); 
        if (it == clusters.end()) continue;
        const std::vector<DBSCAN::Point> &pts = it->second;
        ball_cluster.push_back(std::make_pair(static_cast<int>(cid), pts));
    }
    
    return ball_cluster;
}

double DBSCAN::BallDetect::median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    return v[v.size()/2];
}

std::vector<int> DBSCAN::BallDetect::deleteWall(
    std::unordered_map<int, std::vector<DBSCAN::Point>>& clusters
) {
    std::vector<int> ball_cluster_ids;

    for (auto& [cid, pts]: clusters) {
        if (pts.size() < 3) continue;

        // /*
        //     regtancle th
        // */
        // double min_x = std::numeric_limits<double>::max();
        // double min_y = std::numeric_limits<double>::max();
        // double max_x = std::numeric_limits<double>::min();
        // double max_y = std::numeric_limits<double>::min();
        // for (DBSCAN::Point &p: pts) {
        //     if (min_x > p.getX()) min_x = p.getX();
        //     if (min_y > p.getY()) min_y = p.getY();
        //     if (max_x < p.getX()) max_x = p.getX();
        //     if (max_y < p.getY()) max_y = p.getY();
        // }
        // double diagonal = std::hypot(max_x-min_x, max_y-min_y);
        // if (diagonal > DIAGONAL_THTRSHOLD_) continue;

        /*
            second diff th
        */
        // sort on x
        std::sort(
            pts.begin(), 
            pts.end(),
            [](DBSCAN::Point& a, DBSCAN::Point& b) {
                return a.getPointID() < b.getPointID();
            }
        );

        std::vector<double> second_deriv;
        for (size_t i=1; i+1<pts.size(); i++) {
            double x1 = pts[i-1].getX(), y1 = pts[i-1].getY();
            double x2 = pts[i].getX(),     y2 = pts[i].getY();
            double x3 = pts[i+1].getX(), y3 = pts[i+1].getY();
            
            DBSCAN::UnitVector v1 = DBSCAN::UnitVector(x2-x1, y2-y1, DIFF_THTRSHOLD_);
            DBSCAN::UnitVector v2 = DBSCAN::UnitVector(x3-x2, y3-y2, DIFF_THTRSHOLD_);

            double ax = v2.getX() - v1.getX();
            double ay = v2.getY() - v1.getY();

            second_deriv.push_back(std::hypot(ax, ay));
        }

        double med = DBSCAN::BallDetect::median(second_deriv);
        if (std::abs(med) > WALL_THTRSHOLD_) {
            ball_cluster_ids.push_back(cid);
        }


    }
    return ball_cluster_ids;
}

std::unordered_map<int, std::vector<DBSCAN::Point>> DBSCAN::BallDetect::dbscan(std::vector<DBSCAN::Point> &points, DBSCAN::KdTree &tree) {
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

    // this->pubClusters_->publish(point2PointCloud2(points));

    // sort point on x
    std::unordered_map<int, std::vector<DBSCAN::Point>> clusters;
    for(DBSCAN::Point& p: points) {
        if (p.getID() < 0) continue;
        clusters[p.getID()].push_back(p); 
    }

    return clusters;
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

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // get tf
        transform_stamped = tf_buffer_.lookupTransform("odom", frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return point_cloud;
    }


    for (size_t i=0; i<scan.ranges.size(); i++) {
        double r, theta;
        r = scan.ranges[i];
        if (std::isnan(r) || r<scan.range_min || scan.range_max<r) continue;
        if (is_sim_) theta = scan.angle_min + ((std::double_t)(i))*scan.angle_increment;
        else {
            theta = scan.angle_min + ((std::double_t)(i))*scan.angle_increment - 3.0*M_PI/2.0;
            // normalize
            if (theta > M_PI) theta -= M_2_PI;
            if (theta < -M_PI) theta += M_2_PI;
            
            if (theta < -M_PI_2+this->LIDAR_THTRSHOLD_) continue;
            if (theta >  M_PI_2-this->LIDAR_THTRSHOLD_) continue;
        }
        // convert origin
        try {
            geometry_msgs::msg::PointStamped point_st;
            point_st.header.frame_id = frame_id_;
            point_st.header.stamp = this->now();
            point_st.point.x = r*cos(theta);
            point_st.point.y = r*sin(theta);
            point_st.point.z = 0.0;

            geometry_msgs::msg::PointStamped point;
            tf2::doTransform(point_st, point, transform_stamped);
            point_cloud.points.push_back(Point(point.point.x, point.point.y, i));

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }
    return point_cloud;
}

sensor_msgs::msg::PointCloud2 DBSCAN::BallDetect::point2PointCloud2(
    const std::vector<std::pair<int, std::vector<DBSCAN::Point>>> &points
) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = rclcpp::Clock().now();
    cloud.height = 1;
    cloud.width = points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    size_t total_points = 0;
    for (const std::pair<int, std::vector<DBSCAN::Point>>& point: points) {
        total_points += point.second.size();
    }
    modifier.resize(total_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");

    // count each clusters point
    std::unordered_map<int, int> cluster_counts;
    for (const std::pair<int, std::vector<DBSCAN::Point>> &p: points) {
        cluster_counts[p.first] = p.second.size();
    }

    // sort clusters
    std::vector<std::pair<int, int>> sorted_clusters(cluster_counts.begin(), cluster_counts.end());
    std::sort(sorted_clusters.begin(), sorted_clusters.end(),
              [](auto &a, auto &b) { return a.second > b.second; });

    // fixed color
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

    // assign color to each clusters
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

    // write point cloud
    for (const std::pair<int, std::vector<DBSCAN::Point>> &cluster : points) {
        int cluster_id = cluster.first;
        for (const DBSCAN::Point &p: cluster.second) {
            *iter_x = p.getX();
            *iter_y = p.getY();
            *iter_z = 0.0f;
    
            auto color = cluster_colors[cluster_id];
            *iter_r = color[0];
            *iter_g = color[1];
            *iter_b = color[2];
    
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
    }

    return cloud;
}

DBSCAN::Point::Point(float x, float y, int pointID, int clusterID) {
    this->x_ = x;
    this->y_ = y;
    this->pointID_ = pointID;
    this->clusterID_ = clusterID;
}

int DBSCAN::Point::getPointID() const {
    return this->pointID_;
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

DBSCAN::UnitVector::UnitVector(double x, double y, double eps) {
    double norm = std::hypot(x, y);
    if (norm > eps) {
        x_ = x / norm;
        y_ = y / norm;
    } else {
        x_ = 0.0;
        y_ = 0.0;
    }
}

double DBSCAN::UnitVector::getX() const {
    return x_;
}

double DBSCAN::UnitVector::getY() const {
    return y_;
}

DBSCAN::Circle::Circle(): is_closest_(false), x_(0.0), y_(0.0), r_(0.0) {}

// O(n)
DBSCAN::Circle::Circle(std::vector<DBSCAN::Point> &points) {
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());

    for (size_t i=0; i<points.size(); i++ ) {
        double x = points[i].getX();
        double y = points[i].getY();
        A(i,0) = 2*x;
        A(i,1) = 2*y;
        A(i,2) = 1.0;
        b(i) = x*x + y*y;
    }

    // 3*3 linear equation -> O(1)
    Eigen::Vector3d sol = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    
    this->x_ = sol(0);
    this->y_ = sol(1);
    this->r_ = std::sqrt(x_*x_ + y_*y_ + sol(2));
    this->is_closest_ = false;
}

void DBSCAN::Circle::markClosest() {
    this->is_closest_ = true;
}

double DBSCAN::Circle::getX() {
    return this->x_;
} 

double DBSCAN::Circle::getY() {
    return this->y_;
}

double DBSCAN::Circle::getR() {
    return this->r_;
}

bool DBSCAN::Circle::isClosest() {
    return this->is_closest_;
}

sensor_msgs::msg::PointCloud2 DBSCAN::BallDetect::circle2PointCloud2(std::vector<DBSCAN::Circle> ball_position)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = rclcpp::Clock().now();

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    const int NUM_POINTS = 36;
    modifier.resize((NUM_POINTS + 1)*ball_position.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    for (DBSCAN::Circle &_b: ball_position) {
        // color settings
        uint8_t red = 255, green = 255, blue = 255;
        if (_b.isClosest()) {
            red = 0; green = 0; blue = 255;
        }

        for (int i = 0; i < NUM_POINTS; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
            float angle = 2.0 * M_PI * i / NUM_POINTS;
            *iter_x = _b.getX() + _b.getR() * std::cos(angle);
            *iter_y = _b.getY() + _b.getR() * std::sin(angle);
            *iter_z = 0.0;

            *iter_r = red;
            *iter_g = green;
            *iter_b = blue;
        }

        // 中心点
        *iter_x = _b.getX();
        *iter_y = _b.getY();
        *iter_z = 0.0;
        *iter_r = red;
        *iter_g = green;
        *iter_b = blue;
    }

    return cloud_msg;
}

DBSCAN::Field::Field(std::string map_dir) {
    YAML::Node lconf = YAML::LoadFile(map_dir + "map.yaml");
    mapResolution_ = lconf["resolution"].as<std::double_t>();
    mapOrigin_ = lconf["origin"].as<std::vector<std::double_t>>();

    std::string imgFile = map_dir + "map.pgm";
    cv::Mat mapImg = cv::imread(imgFile, 0);
    mapWidth_ = mapImg.cols;
    mapHeight_ = mapImg.rows;

    mapImg_ = mapImg.clone();
    for (int v = 0; v < mapHeight_; v++ ) {
        for (int u = 0; u < mapWidth_; u++ ) {
            uchar val = mapImg_.at<uchar>(v, u);
            if (val >= 239) {
                mapImg_.at<uchar>(v, u) = 1;
            } else {
                mapImg_.at<uchar>(v, u) = 0;
            }
        }
    }
}
DBSCAN::Field::Field() {}

bool DBSCAN::BallDetect::isBallOnField(DBSCAN::Field &f, DBSCAN::Circle &c) {
    // convert map field
    int u, v;
    f.xy2uv(c.getX(), c.getY(), &u, &v);
    
    // circle center position
    if (u < 0 || v < 0 || u >= f.mapWidth_ || v >= f.mapHeight_) return false;
    if (f.mapImg_.at<uchar>(v, u) == 0) return false;

    // 
    bool inForbidden = false;
    const double step = M_PI / 8.0;
    for (double theta = 0.0; theta < 2 * M_PI; theta += step) {
        double x_edge = c.getX() + c.getR() * std::cos(theta);
        double y_edge = c.getY() + c.getR() * std::sin(theta);

        int ue, ve;
        f.xy2uv(x_edge, y_edge, &ue, &ve);
        if (ue < 0 || ve < 0 || ue >= f.mapWidth_ || ve >= f.mapHeight_) {
            inForbidden = true;
            break;
        }

        if (f.mapImg_.at<uchar>(ve, ue) == 0) {
            inForbidden = true;
            break;
        }
    }

    if (inForbidden) return false;
    
    return true;
}


bool DBSCAN::Field::isBallOnField(DBSCAN::Circle &c) {
    // convert map field
    int u, v;
    xy2uv(c.getX(), c.getY(), &u, &v);
    printf("%d %d", u, v);

    // circle center position
    if (u < 0 || v < 0 || u >= mapWidth_ || v >= mapHeight_) return false;
    if (this->mapImg_.at<uchar>(v, u) == 0) return false;


    // 
    bool inForbidden = false;
    const double step = M_PI / 8.0;
    for (double theta = 0.0; theta < 2 * M_PI; theta += step) {
        double x_edge = c.getX() + c.getR() * std::cos(theta);
        double y_edge = c.getY() + c.getR() * std::sin(theta);

        int ue, ve;
        xy2uv(x_edge, y_edge, &ue, &ve);
        if (ue < 0 || ve < 0 || ue >= mapWidth_ || ve >= mapHeight_) {
            inForbidden = true;
            break;
        }

        if (mapImg_.at<uchar>(ve, ue) == 0) {
            inForbidden = true;
            break;
        }
    }

    // if (inForbidden) return false;
    
    return true;
}

void DBSCAN::Field::xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v) {
    *u = (std::int32_t)(x / mapResolution_);
    *v = mapHeight_ - 1 - (std::int32_t)(y / mapResolution_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DBSCAN::BallDetect>());
    rclcpp::shutdown();
    return 0;
}