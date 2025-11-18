#include <ball_path_node.hpp>

namespace yasarobo2025_26{
    BallPathNode::BallPathNode(const rclcpp::NodeOptions & options): Node("ball_path_node", options){
        subPose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&BallPathNode::poseCallback, this, std::placeholders::_1)
        );     

        rclcpp::QoS pathQos = rclcpp::QoS(rclcpp::KeepLast(10))
                                .reliable()
                                .transient_local();
        path_pub_ = create_publisher<nav_msgs::msg::Path>("route", pathQos);


        srv_gen_route_ = this->create_service<inrof2025_ros_type::srv::BallPath>(
            "generate_ball_path", std::bind(&BallPathNode::genBallPath, this, std::placeholders::_1, std::placeholders::_2)
        );

        rclcpp::QoS poseArrowQos(rclcpp::KeepLast(10));
        pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", poseArrowQos);

        this->declare_parameter<int>("num_points_", 10);
        this->declare_parameter<double>("shorten", 0.04);
        this->declare_parameter<double>("theta_offset", 0.0);
        this->get_parameter("num_points_", num_points_);
        this->get_parameter("shorten", shorten_);
        this->get_parameter("theta_offset", theta_offset_);
    };

    void BallPathNode::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
        pose_ = msg;
    };

    

    void BallPathNode::genBallPath(
        const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Request> request,
        const std::shared_ptr<inrof2025_ros_type::srv::BallPath::Response> response
    ){
        RCLCPP_INFO(this->get_logger(), "Generating ball path to (%.2f, %.2f)", request->x, request->y);
        if (!pose_){
            RCLCPP_INFO(this->get_logger(), "no pose");
            return;
        }

        //generate path
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        
        geometry_msgs::msg::PoseStamped start_pose;
        geometry_msgs::msg::PoseStamped goal_pose;
        start_pose.header = path_msg.header;
        goal_pose.header = path_msg.header;

        //get position
        start_pose.pose.position.x = pose_->x;
        start_pose.pose.position.y = pose_->y;
        goal_pose.pose.position.x = request->x;
        goal_pose.pose.position.y = request->y;

        double theta = 0.0;

        double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
        double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        double ux = dx / distance;
        double uy = dy / distance;

        if (dx == 0.0 && dy == 0.0){
            RCLCPP_INFO(this->get_logger(), "start and goal are the same");
            return;
        }
        else if (dx == 0.0){
            if (dy > 0.0){
                theta = M_PI / 2.0;
            }else{
                theta = -M_PI / 2.0;
            }
            return;
        }
        else{
            theta = atan2(dy, dx);
        }

        theta = theta - 2.0*M_PI/3.0 + theta_offset_;

        if (request->is_return) {
            theta = pose_->theta;
        }

       
        //shorten path
        if (distance > shorten_){
            goal_pose.pose.position.x -= shorten_ * ux;
            goal_pose.pose.position.y -= shorten_ * uy;
	}
        //create path
        for (int i=0; i<=num_points_; ++i){
            double t = static_cast<double>(i) / num_points_;
            geometry_msgs::msg::PoseStamped p;
            p.header = path_msg.header;
            p.pose.position.x = start_pose.pose.position.x + t * (goal_pose.pose.position.x - start_pose.pose.position.x);
            p.pose.position.y = start_pose.pose.position.y + t * (goal_pose.pose.position.y - start_pose.pose.position.y);

            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            p.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(p);
        }

        visualization_msgs::msg::Marker arrow;
        geometry_msgs::msg::Pose goal_robot_pose;

        // pub waypoint pose
        arrow.header.frame_id = "map";
        arrow.ns = "goal_point_arrow";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = path_msg.poses[path_msg.poses.size()-1].pose;
        arrow.scale.x = 0.08;
        arrow.scale.y = 0.04;
        arrow.scale.z = 0.04;
        arrow.color.r = 0.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;
        pose_arrow_pub_ -> publish(arrow);

        path_pub_->publish(path_msg);
    };

}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<yasarobo2025_26::BallPathNode>());
    rclcpp::shutdown();
    return 0;
}
