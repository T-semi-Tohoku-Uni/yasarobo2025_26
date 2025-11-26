#include <mpc_node.hpp>

using namespace std::chrono_literals; 

namespace yasarobo2025_26{
    MpcNode::MpcNode(const rclcpp::NodeOptions & options): Node("mpc_node", options){
        
        // Q
        double q_x, q_y, q_theta;
        this->declare_parameter<double>("q_x", 1.0);
        this->declare_parameter<double>("q_y", 1.0);
        this->declare_parameter<double>("q_theta", 0.10);
        this->get_parameter<double>("q_x", q_x);
        this->get_parameter<double>("q_y", q_y);
        this->get_parameter<double>("q_theta", q_theta);

        // Qf
        double q_fx, q_fy, q_ftheta;
        this->declare_parameter<double>("q_fx", 1.0);
        this->declare_parameter<double>("q_fy", 1.0);
        this->declare_parameter<double>("q_ftheta", 0.10);
        this->get_parameter<double>("q_fx", q_fx);
        this->get_parameter<double>("q_fy", q_fy);
        this->get_parameter<double>("q_ftheta", q_ftheta);

        // R
        double r_vx, r_vy, r_vtheta;
        this->declare_parameter<double>("r_vx", 0.10);
        this->declare_parameter<double>("r_vy", 0.10);
        this->declare_parameter<double>("r_vtheta", 0.10);
        this->get_parameter<double>("r_vx", r_vx);
        this->get_parameter<double>("r_vy", r_vy);
        this->get_parameter<double>("r_vtheta", r_vtheta);

        // mpc parameter
        double max_speed;
        this->declare_parameter<double>("max_speed", 0.10);
        this->get_parameter<double>("max_speed", max_speed);
        
        this->declare_parameter<int32_t>("K", 20);
        this->get_parameter<int32_t>("K", K_);

        this->declare_parameter<double>("lookahead_distance", 0.20);
        this->get_parameter<double>("lookahead_distance", lookahead_distance_);

        this->declare_parameter<double>("max_reaching_distance", 0.05);
        this->get_parameter<double>("max_reaching_distance", max_reaching_distance_);

        subpose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&MpcNode::poseCallback, this, std::placeholders::_1)
        );
        rclcpp::QoS pathQos(rclcpp::KeepLast(5));
        subpath_ = create_subscription<nav_msgs::msg::Path>(
            "route", pathQos, std::bind(&MpcNode::pathCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10
        );

        rclcpp::QoS poseArrowQos(rclcpp::KeepLast(10));
        pose_arrow_pub_= this->create_publisher<visualization_msgs::msg::Marker>("pose_arrow_marker", poseArrowQos);

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            100ms,
            std::bind(&MpcNode::control, this)
        );

        action_server_ = rclcpp_action::create_server<inrof2025_ros_type::action::Follow>(
            this,
            "follow",
            std::bind(&MpcNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MpcNode::handleCancel, this, std::placeholders::_1),
            std::bind(&MpcNode::handleAccepted, this, std::placeholders::_1)
        );

        Q_ = casadi::SX::diag({q_x, q_y, q_theta});
        Qf_ = casadi::SX::diag({q_fx, q_fy, q_ftheta});
        R_ = casadi::SX::diag({r_vx, r_vy, r_vtheta});
        x_ref_ = casadi::DM({0.3, 2.0, 0.0});
        u_ref_ = casadi::DM({0.0, 0.0, 0.0});

        x_lb_ = casadi::DM({-casadi::inf, -casadi::inf, -casadi::inf});
        x_ub_ = casadi::DM({casadi::inf, casadi::inf, casadi::inf});
        u_lb_ = casadi::DM({-max_speed, -max_speed, -max_speed});
        u_ub_ = casadi::DM({max_speed, max_speed, max_speed});

        // K_ = 20;
        r_ = 0.14;
        delta_t_ = 0.10;
        nx_ = 3;
        nu_ = 3;
    }

    void MpcNode::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        pose_ = msg;
    }

    void MpcNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg;
        current_index_ = 0;
    }

    void MpcNode::control() {
        if (!goal_handle_) {
            return;
        }
        if (!pose_) {
            RCLCPP_WARN(this->get_logger(), "pose not available");
            return;
        }
        if (!path_) {
            RCLCPP_WARN(this->get_logger(), "path not available");
            return;
        }
        
        // update waypoint
        double linear_error = std::hypot(
            path_->poses[current_index_].pose.position.x - pose_->x,
            path_->poses[current_index_].pose.position.y - pose_->y
        );
        while(lookahead_distance_ > linear_error) {
            if (current_index_+1 >= static_cast<size_t>(path_->poses.size())) break;
            current_index_++;
            linear_error = std::hypot(
                path_->poses[current_index_].pose.position.x - pose_->x,
                path_->poses[current_index_].pose.position.y - pose_->y
            );
        }

        // check reaching goal
        double goal_error = std::hypot(
            path_->poses[path_->poses.size()-1].pose.position.x - pose_->x,
            path_->poses[path_->poses.size()-1].pose.position.y - pose_->y
        );
        if (max_reaching_distance_ > goal_error) {
            // TODO: check velocity is zero
            RCLCPP_WARN(this->get_logger(), "Reaching goal");

            // publish zero velocity
            geometry_msgs::msg::Twist t;
            t.linear.x = 0.0;
            t.linear.y = 0.0;
            t.angular.z = 0.0;
            cmd_pub_->publish(t);

            std::shared_ptr<inrof2025_ros_type::action::Follow_Result> result_msg = 
                std::make_shared<inrof2025_ros_type::action::Follow::Result>();
            result_msg->success = true;
            goal_handle_->succeed(result_msg);
            goal_handle_.reset();
            return;
        }

        printWayPointArrow(path_->poses[current_index_].pose, path_->poses[path_->poses.size()-1].pose);

        // get yaw
        tf2::Quaternion tf_q;
        tf2::fromMsg(path_->poses[current_index_].pose.orientation, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
        x_ref_ = casadi::DM({
            path_->poses[current_index_].pose.position.x, 
            path_->poses[current_index_].pose.position.y, 
            yaw // TODO
        });

        // initialize current state
        casadi::DM x_current = casadi::DM::vertcat({pose_->x, pose_->y, pose_->theta});
        casadi::DM x0 = casadi::DM::zeros(nx_*(K_+1)+nu_*K_);
        casadi::Function S = MpcNode::make_nlp();

        // mpc
        // (u, x)
        std::pair<casadi::DM, casadi::DM> result = 
            compute_optimal_control(S, x_current, x0);
        
        // publish
        geometry_msgs::msg::Twist t;
        std::array<double, 3> v_r = MpcNode::inverseKinematics(result.first(0).scalar(), result.first(1).scalar(), result.first(2).scalar());
        t.linear.x = v_r[0];
        t.linear.y = v_r[1];
        t.angular.z = v_r[2];
        cmd_pub_->publish(t);
    }

    std::pair<casadi::DM, casadi::DM> MpcNode::compute_optimal_control(
        casadi::Function& S,
        const casadi::DM& x_init,
        const casadi::DM& x0
    ) {
        std::vector<double> lbx, ubx;
        lbx.reserve(nx_*(K_+1) + nu_*K_);
        ubx.reserve(nx_*(K_+1) + nu_*K_);

        // bounds of x
        for (int k=0; k<=K_; k++ ) {
            if (k==0) {
                for (int i=0; i<nx_; i++ ) {
                    lbx.push_back(static_cast<double>(x_init(i)));
                    ubx.push_back(static_cast<double>(x_init(i)));
                }
            }
            else {
                for (int i=0; i<nx_; i++ ) {
                    lbx.push_back(static_cast<double>(x_lb_(i)));
                    ubx.push_back(static_cast<double>(x_ub_(i)));
                }
            }
        }

        // bounds of u
        for (int k=0; k<K_; k++ ) {
            for (int j=0; j<nu_; j++ ) {
                lbx.push_back(static_cast<double>(u_lb_(j)));
                ubx.push_back(static_cast<double>(u_ub_(j)));
            }
        }

        std::vector<double> lbg(nx_*K_, 0.0);
        std::vector<double> ubg(nx_*K_, 0.0);

        casadi::DMDict arg;
        arg["lbx"] = casadi::DM(lbx);
        arg["ubx"] = casadi::DM(ubx);
        arg["lbg"] = casadi::DM(lbg);
        arg["ubg"] = casadi::DM(ubg);
        arg["x0"]  = x0;

        casadi::DMDict res = S(arg);        
        casadi::DM x_opt = res.at("x");

        int offset = nx_*(K_+1);
        casadi::DM u_opt = x_opt(casadi::Slice(offset, offset+nu_));

        return {u_opt, x_opt};
    }

    casadi::Function MpcNode::make_nlp() {
        casadi::Function F = MpcNode::make_f();

        std::vector<casadi::SX> U(K_);
        std::vector<casadi::SX> X(K_+1);

        for (int k=0; k<K_; k++ ) {
            U[k] = casadi::SX::sym("u_"+std::to_string(k), nu_);
        }
        for (int k=0; k<K_+1; k++ ) {
            X[k] = casadi::SX::sym("x_"+std::to_string(k), nx_);
        }

        std::vector<casadi::SX> G;
        casadi::SX J=0;

        for (int k=0; k<K_; k++ ) {
            J += MpcNode::compute_stage_cost(X[k], U[k]);

            // x[k+1] - F(x[k], u[k], k) = 0
            casadi::SXDict input;
            input["x"] = X[k];
            input["u"] = U[k];
            casadi::SX eq = X[k+1] - F(input).at("x_next");
            G.push_back(eq);
        }


        J += MpcNode::compute_terminal_cost(X[X.size()-1]);

        std::vector<casadi::SX> all_vars;
        all_vars.insert(all_vars.end(), X.begin(), X.end());
        all_vars.insert(all_vars.end(), U.begin(), U.end());
        casadi::SX x_var = casadi::SX::vertcat(all_vars);
        casadi::SX g_var = casadi::SX::vertcat(G);

        casadi::SXDict nlp;
        nlp["x"] = x_var;
        nlp["f"] = J;
        nlp["g"] = g_var;

        casadi::Dict opts;
        opts["print_time"] = false;

        casadi::Dict ipopt_opts;
        ipopt_opts["print_level"] = 0;        // Ipopt 自体の出力を抑制
        ipopt_opts["sb"] = "yes";             // Ipopt の "suppress banner"
        ipopt_opts["print_user_options"] = "no";

        opts["ipopt"] = ipopt_opts;           // 組み込む


        casadi::Function S = casadi::nlpsol("S", "ipopt", nlp, opts);

        return S;
    }

    casadi::Function MpcNode::make_f(){
        // nx は状態変数の次元数, nu は制御入力の次元数
        casadi::SX states = casadi::SX::sym("states", nx_);
        casadi::SX controls = casadi::SX::sym("controls", nu_);

        //状態変数の定義
        casadi::SX x_f = states(0);
        casadi::SX y_f = states(1);
        casadi::SX theta_f = states(2);

        //制御入力の定義
        casadi::SX v1 = controls(0);
        casadi::SX v2 = controls(1);
        casadi::SX v3 = controls(2);

        std::array<casadi::SX, 3> v_r = MpcNode::inverseKinematics(v1, v2, v3);
        std::array<casadi::SX, 3> v_f = MpcNode::toFieldVel(v_r[0], v_r[1], v_r[2], theta_f);

        casadi::SX x_f_next      = x_f + v_f[0]*delta_t_;
        casadi::SX y_f_next      = y_f + v_f[1]*delta_t_;
        casadi::SX theta_f_next  = theta_f + v_f[2]*delta_t_;
        
        casadi::SX states_next = casadi::SX::vertcat({x_f_next, y_f_next, theta_f_next});
        casadi::Function F = 
            casadi::Function("F", {states, controls}, {states_next}, {"x", "u"}, {"x_next"});
        
        return F;
    }

    casadi::SX MpcNode::compute_stage_cost(const casadi::SX& x, const casadi::SX& u) {
        casadi::SX x_diff = x - casadi::SX(x_ref_);
        casadi::SX u_diff = u - casadi::SX(u_ref_);

        casadi::SX cost = 0.5 * (
            casadi::SX::mtimes(x_diff.T(), casadi::SX::mtimes(Q_, x_diff)) +
            casadi::SX::mtimes(u_diff.T(), casadi::SX::mtimes(R_, u_diff))
        );

        return cost;
    }

    casadi::SX MpcNode::compute_terminal_cost(const casadi::SX& x) {
        casadi::SX x_diff = x - casadi::SX(x_ref_);
        casadi::SX cost = 0.5*casadi::SX::mtimes(x_diff.T(), casadi::SX::mtimes(casadi::SX(Qf_), x_diff));
        return cost;
    }

    std::array<casadi::SX, 3> MpcNode::inverseKinematics(casadi::SX& v1, casadi::SX& v2, casadi::SX& v3) {
        std::array<casadi::SX, 3> out;
        
        out[0] = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
        out[1] = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
        out[2] = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;

        return out;
    }

    std::array<double, 3> MpcNode::inverseKinematics(double v1, double v2, double v3) {
        std::array<double, 3> out;
        
        out[0] = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
        out[1] = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
        out[2] = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;

        return out;
    }

    std::array<casadi::SX, 3> MpcNode::toFieldVel(casadi::SX& v_xr, casadi::SX& v_yr, casadi::SX& omega_r, casadi::SX& theta_f) {
        std::array<casadi::SX, 3> out;

        out[0] = cos(theta_f)*v_xr - sin(theta_f)*v_yr;
        out[1] = sin(theta_f)*v_xr + cos(theta_f)*v_yr;
        out[2] = omega_r;

        return out;
    }

    rclcpp_action::GoalResponse MpcNode::handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const inrof2025_ros_type::action::Follow::Goal> goal
    ) {
        if (!goal_handle_){
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse MpcNode::handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
    ) {
        goal_handle_.reset();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void MpcNode::handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2025_ros_type::action::Follow>> goal_handle
    ) {
        goal_handle_ = goal_handle;
    }

    void MpcNode::printWayPointArrow(geometry_msgs::msg::Pose waypoint_pose, geometry_msgs::msg::Pose goal_pose) {
        visualization_msgs::msg::Marker arrow;

        // pub waypoint pose
        arrow.header.frame_id = "map";
        arrow.ns = "way_point_arrow";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = waypoint_pose;
        arrow.scale.x = 0.08;
        arrow.scale.y = 0.04;
        arrow.scale.z = 0.04;

        arrow.color.r = 0.0f;
        arrow.color.g = 0.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;
        pose_arrow_pub_ -> publish(arrow);

        // pub goal pose
        arrow.header.frame_id = "map";
        arrow.ns = "goal_point_arrow";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose = goal_pose;
        arrow.scale.x = 0.08;
        arrow.scale.y = 0.04;
        arrow.scale.z = 0.04;

        arrow.color.r = 0.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 0.0f;
        arrow.color.a = 1.0f;
        pose_arrow_pub_ -> publish(arrow);
    }

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<yasarobo2025_26::MpcNode>());
    rclcpp::shutdown();
    return 0;
}