#include <mpc_node.hpp>

using namespace std::chrono_literals; 

namespace yasarobo2025_26{
    MpcNode::MpcNode(const rclcpp::NodeOptions & options): Node("mpc_node", options){
        
        subpose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&MpcNode::poseCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10
        );

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            100ms,
            std::bind(&MpcNode::control, this)
        );

        Q_ = casadi::SX::diag({1.0, 1.0, 0.001});
        Qf_ = casadi::SX::diag({1.0, 1.0, 0.001});
        R_ = casadi::SX::diag({0.1, 0.1, 0.1});
        x_ref_ = casadi::DM({0.3, 2.0, 0.0});
        u_ref_ = casadi::DM({0.0, 0.0, 0.0});

        x_lb_ = casadi::DM({-casadi::inf, -casadi::inf, -casadi::inf});
        x_ub_ = casadi::DM({casadi::inf, casadi::inf, casadi::inf});
        u_lb_ = casadi::DM({-0.1, -0.1, -0.1});
        u_ub_ = casadi::DM({0.1, 0.1, 0.1});

        K_ = 10;
        r_ = 0.14;
        delta_t_ = 0.10;
        nx_ = 3;
        nu_ = 3;
    }

    void MpcNode::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        pose_ = msg;
    }

    void MpcNode::control() {
        if (!pose_) {
            RCLCPP_WARN(this->get_logger(), "pose not available");
            return;
        }

        casadi::DM x_current = casadi::DM::vertcat({pose_->x, pose_->y, pose_->theta});
        casadi::DM x0 = casadi::DM::zeros(nx_*(K_+1)+nu_*K_);
        casadi::Function S = MpcNode::make_nlp();

        // (u, x)
        std::pair<casadi::DM, casadi::DM> result = 
            compute_optimal_control(S, x_current, x0);
        
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
        RCLCPP_INFO(this->get_logger(), "%s", x_opt.get_str().c_str());

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
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<yasarobo2025_26::MpcNode>());
    rclcpp::shutdown();
    return 0;
}