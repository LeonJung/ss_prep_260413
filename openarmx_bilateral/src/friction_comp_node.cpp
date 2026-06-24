// friction_comp_node.cpp — Phase 3: add identified Coulomb friction to the
// gravity-comp effort, per robot. Sits between gravity_comp_node and the effort
// controller so the effort command interface keeps a SINGLE publisher:
//
//   gravity_comp_node --(grav_in: gravity torque)-->  [this node]
//   /joint_states     --(joint velocity)----------->  [this node]
//        out[j] = grav[j] + (enable ? scale * Fc[j]*tanh(0.1*k[j]*w[j]) : 0)
//   [this node] --(effort_out)--> left_forward_effort_controller/commands
//
// Friction is Coulomb-dominated (identified Fv~=0, so only the tanh term). tanh
// gives a smooth sign at w=0 (no chatter). enable:=false => pure passthrough
// (gravity only) for clean A/B testing. scale<1 avoids over-compensation /
// limit cycles; tune by feel.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FrictionCompNode : public rclcpp::Node {
public:
    FrictionCompNode() : Node("openarmx_friction_comp") {
        declare_parameter<std::string>("grav_in", "/follower/grav_only");
        declare_parameter<std::string>("states", "/follower/joint_states");
        declare_parameter<std::string>("effort_out",
            "/follower/left_forward_effort_controller/commands");
        declare_parameter<std::string>("joint_prefix", "openarmx_left_joint");
        declare_parameter<int>("n_joints", 7);
        declare_parameter<std::vector<double>>("fc", std::vector<double>{});  // Coulomb [Nm]
        declare_parameter<std::vector<double>>("k", std::vector<double>{});   // tanh sharpness
        declare_parameter<double>("scale", 0.7);          // conservative; tune by feel
        declare_parameter<bool>("enable", true);          // false => gravity passthrough (A/B OFF)
        declare_parameter<double>("tau_max", 3.0);        // per-joint friction clamp [Nm]
        declare_parameter<double>("vel_eps", 0.0);        // ignore |w| below this (0 = off)

        nj_ = get_parameter("n_joints").as_int();
        std::string jp = get_parameter("joint_prefix").as_string();
        for (int i = 0; i < nj_; ++i) names_.push_back(jp + std::to_string(i + 1));
        fc_ = get_parameter("fc").as_double_array();
        k_  = get_parameter("k").as_double_array();
        scale_ = get_parameter("scale").as_double();
        enable_ = get_parameter("enable").as_bool();
        tau_max_ = get_parameter("tau_max").as_double();
        vel_eps_ = get_parameter("vel_eps").as_double();
        vel_.assign(nj_, 0.0);

        bool ok = ((int)fc_.size() == nj_ && (int)k_.size() == nj_);
        if (!ok) {
            RCLCPP_WARN(get_logger(), "fc/k size != n_joints (%zu/%zu vs %d) -> friction DISABLED (passthrough)",
                        fc_.size(), k_.size(), nj_);
            enable_ = false;
        }
        RCLCPP_INFO(get_logger(), "friction_comp: enable=%s scale=%.2f n=%d  out=%s",
                    enable_ ? "ON" : "off(passthrough)", scale_, nj_,
                    get_parameter("effort_out").as_string().c_str());

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            get_parameter("effort_out").as_string(), 10);
        st_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("states").as_string(), 50,
            [this](sensor_msgs::msg::JointState::SharedPtr m) {
                for (int j = 0; j < nj_; ++j)
                    for (size_t kk = 0; kk < m->name.size(); ++kk)
                        if (m->name[kk] == names_[j]) {
                            if (kk < m->velocity.size()) vel_[j] = m->velocity[kk];
                            break;
                        }
            });
        grav_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("grav_in").as_string(), 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr m) { onGrav(m); });
    }

private:
    void onGrav(const std_msgs::msg::Float64MultiArray::SharedPtr& g) {
        std_msgs::msg::Float64MultiArray out;
        out.data.resize(g->data.size());
        for (size_t j = 0; j < g->data.size(); ++j) {
            double tau = g->data[j];
            if (enable_ && (int)j < nj_) {
                double w = vel_[j];
                if (std::abs(w) >= vel_eps_) {
                    double fr = scale_ * fc_[j] * std::tanh(0.1 * k_[j] * w);
                    fr = std::clamp(fr, -tau_max_, tau_max_);
                    tau += fr;
                }
            }
            out.data[j] = tau;
        }
        pub_->publish(out);
    }

    int nj_ = 7;
    bool enable_ = true;
    double scale_ = 0.7, tau_max_ = 3.0, vel_eps_ = 0.0;
    std::vector<std::string> names_;
    std::vector<double> fc_, k_, vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr st_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grav_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrictionCompNode>());
    rclcpp::shutdown();
    return 0;
}
