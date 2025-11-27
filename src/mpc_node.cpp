#include <mpc_node.hpp>

namespace yasarobo2025_26{
    MpcNode::MpcNode(const rclcpp::NodeOptions & options): Node("mpc_node", options){
        
        subpose_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&MpcNode::poseCallback, this, std::placeholders::_1)
        );

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10
        );

        //Q
        double q_x, q_y, q_theta;
        this->declare_parameter("q_x", 1.0);
        this->declare_parameter("q_y", 1.0);
        this->declare_parameter("q_theta", 1.0);
        this->get_parameter("q_x", q_x);
        this->get_parameter("q_y", q_y);
        this->get_parameter("q_theta", q_theta);

        //R
        double r_1, r_2, r_3;
        this->declare_parameter("r_1", 1.0);
        this->declare_parameter("r_2", 1.0);
        this->declare_parameter("r_3", 1.0);
        this->get_parameter("r_1", r_1);
        this->get_parameter("r_2", r_2);
        this->get_parameter("r_3", r_3);

        //Qf
        double qf_x, qf_y, qf_theta;
        this->declare_parameter("qf_x", 1.0);
        this->declare_parameter("qf_y", 1.0);
        this->declare_parameter("qf_theta", 1.0);
        this->get_parameter("qf_x", qf_x);
        this->get_parameter("qf_y", qf_y);
        this->get_parameter("qf_theta", qf_theta);

        Q_ = casadi::SX::diag(casadi::SX::vertcat({q_x, q_y, q_theta}));
        R_ = casadi::SX::diag(casadi::SX::vertcat({r_1, r_2, r_3}));
        Qf_ = casadi::SX::diag(casadi::SX::vertcat({qf_x, qf_y, qf_theta}));
        x_ref_ = casadi::SX({0, 0, 0});
        u_ref_ = casadi::SX({0, 0, 0});

        x_lb_ = 
        x_ub_ =
        u_lb_ =
        u_ub_ = 
    }


    void MpcNode::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg){
        pose_ = msg;
    }


    casadi::SX MpcNode::inverseKinematics(casadi::SX v1, casadi::SX v2, casadi::SX v3) {
                casadi::SX out = casadi::SX::zeros(3);
                // twist.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
                // twist.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
                // twist.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
                out(1) = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
                out(2) = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
                out(3) = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
                return out;
            }


    casadi::SX MpcNode::compute_A(casadi::SX a, casadi::SX b, casadi::SX c){
        casadi::SX out = casadi::SX::zeros(3);
        out(0) = cos(pose_->theta) * a - sin(pose_->theta) * b;
        out(1) = sin(pose_->theta) * a + cos(pose_->theta) * b;
        out(2) = c;
        
        return out;
    }
    

    casadi::Function MpcNode::make_f(){
        // nx は状態変数の次元数, nu は制御入力の次元数, dt はサンプリングタイム
        double dt = 0.10;
        casadi::SX states = casadi::SX::sym("states", nx);
        casadi::SX controls = casadi::SX::sym("controls", nu);

        //状態変数の定義
        casadi::SX x_f = states(0);
        casadi::SX y_f = states(1);
        casadi::SX theta_f = states(2);

        //制御入力の定義
        casadi::SX v1 = controls(0);
        casadi::SX v2 = controls(1);
        casadi::SX v3 = controls(2);

    
        //状態方程式の定義
        casadi::SX cmd = inverseKinematics(v1, v2, v3);
        casadi::SX delta_x_f = dt * compute_A(cmd(0), cmd(1), cmd(2))(0);
        casadi::SX delta_y_f = dt * compute_A(cmd(0), cmd(1), cmd(2))(1);
        casadi::SX delta_theta_f = dt * compute_A(cmd(0), cmd(1), cmd(2))(2);
       

        casadi::Function f = casadi::Function("f", {states, controls}, {delta_x_f, delta_y_f, delta_theta_f});

        return f;
    }


    casadi::SX MpcNode::compute_stage_cost(casadi::SX states, casadi::SX controls){
        // ステージコストの計算
        casadi::SX state_error = states - x_ref_;
        casadi::SX control_error = controls - u_ref_;

        //QとRの定義
        casadi::SX Q = casadi::SX::diag(casadi::SX::vertcat({1, 1, 1}));
        casadi::SX R = casadi::SX::diag(casadi::SX::vertcat({1, 1, 1}));    

        //評価関数の定義
        casadi::SX stage_cost = state_error.T() * Q * state_error + control_error.T() * R * control_error;
        return stage_cost;
    }


    casadi::SX MpcNode::compute_terminal_cost(casadi::SX states){
        // ターミナルコストの計算
        casadi::SX state_error = states - x_ref_;

        //Qfの定義
        casadi::SX Qf = casadi::SX::diag(casadi::SX::vertcat({1, 1, 1}));

        //評価関数の定義
        casadi::SX terminal_cost = state_error.T() * Qf * state_error;
        return terminal_cost;
    }


    casadi::Function MpcNode::make_nlp(){
        //状態方程式
        casadi::Function f = make_f();

        //最適化変数の定義
        std::vector<casadi::SX> X(K_ + 1);
        std::vector<casadi::SX> U(K_);
        
        //シンボリック変数の作成
        for (int k = 0; k <= K_+ 1; ++k){
            X[k] = casadi::SX::sym("X_" + std::to_string(k), nx_);
        }
        for (int k = 0; k <= K_; ++k){
            U[k] = casadi::SX::sym("U_" + std::to_string(k), nu_);
        }

        std::vector<casadi::SX> G;
        casadi::SX J = 0;

        for (int k = 0; k < K_; ++k){
            //ステージコストの計算
            casadi::SX stage_cost = compute_stage_cost(X[k], U[k]);
            J = J + MpcNode::compute_stage_cost(X[k], U[k]);

            //状態方程式の制約
            casadi::SX eq = X[k + 1] - (X[k] + f(casadi::SXVector{X[k], U[k]})[0]);
            G.push_back(eq);
        }
        //ターミナルコストの計算
        J = J + MpcNode::compute_terminal_cost(X.size() - 1);
        
        //最適化問題の定義
        std::vector<casadi::SX> all_vars;
        all_vars.insert(all_vars.end(), X.begin(), X.end());
        all_vars.insert(all_vars.end(), U.begin(), U.end());
        casadi::SX x_var = casadi::SX::vertcat(all_vars);
        casadi::SX g_var = casadi::SX::vertcat(G);

        //NLPの構築
        casadi::SXDict nlp;
        nlp["x"] = x_var;
        nlp["f"] = J;
        nlp["g"] = g_var;

        //optionsの設定
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


    

}