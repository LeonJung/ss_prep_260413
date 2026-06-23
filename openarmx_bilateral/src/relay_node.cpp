// relay_node.cpp — openarmx bilateral teleop cross-relay (topic-only; the
// ros2_control HW interface does the CAN/MIT, this just moves positions between
// arms). Reference: openarmx_teleop_bimanual one-way relay
// (teleop_bimanual_node.cpp leader->follower); here it is generalized to be
// bidirectional with a toggle.
//
//   bilateral:=false (default) -> UNILATERAL: leader pos -> follower
//       forward_position_controller (leader is gravity-float / hand-moved).
//   bilateral:=true            -> also follower pos -> leader
//       forward_position_controller; force feedback then emerges from the
//       leader HW MIT kp (set leader kp soft via the kp_joint param service).
//
// Defaults are LEFT-arm only (operator: 당분간 left만 테스트, no arm_side typing).
// All topics are overridable to match the two bringups' namespaces.

#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RelayNode : public rclcpp::Node {
public:
    RelayNode() : Node("openarmx_bilateral_relay") {
        // LEFT defaults (override arm_side / topics for right or different ns)
        declare_parameter<std::string>("arm_side", "left_arm");   // left_arm | right_arm
        std::string side = get_parameter("arm_side").as_string();
        std::string sp = (side == "right_arm") ? "right" : "left";

        // LEADER runs NON-namespaced so the official gravity_comp_node (absolute
        // topics) reaches it; FOLLOWER under /follower. Override if your bringup
        // namespacing differs.
        declare_parameter<std::string>("leader_states",   "/joint_states");
        declare_parameter<std::string>("follower_states", "/follower/joint_states");
        declare_parameter<std::string>("leader_cmd",
            "/" + sp + "_forward_position_controller/commands");
        declare_parameter<std::string>("follower_cmd",
            "/follower/" + sp + "_forward_position_controller/commands");
        declare_parameter<std::string>("joint_prefix", "openarmx_" + sp + "_joint");   // +1..7
        declare_parameter<std::string>("gripper_joint", "openarmx_" + sp + "_finger_joint1");
        declare_parameter<int>("n_joints", 7);
        declare_parameter<bool>("include_gripper", true);
        declare_parameter<double>("couple_sign", 1.0);   // +1 verified on HW (left pair)
        declare_parameter<bool>("bilateral", false);      // false = unilateral (step b)
        declare_parameter<int>("rate_hz", 200);

        nj_ = get_parameter("n_joints").as_int();
        std::string jp = get_parameter("joint_prefix").as_string();
        for (int i = 0; i < nj_; ++i) names_.push_back(jp + std::to_string(i + 1));
        gripper_name_ = get_parameter("gripper_joint").as_string();
        include_gripper_ = get_parameter("include_gripper").as_bool();
        couple_sign_ = get_parameter("couple_sign").as_double();
        bilateral_ = get_parameter("bilateral").as_bool();
        int rate = get_parameter("rate_hz").as_int();

        ql_.assign(nj_, 0.0); qf_.assign(nj_, 0.0);

        follower_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            get_parameter("follower_cmd").as_string(), 10);
        if (bilateral_)
            leader_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
                get_parameter("leader_cmd").as_string(), 10);
        leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("leader_states").as_string(), 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { grab(m, ql_, gl_, have_l_); });
        follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("follower_states").as_string(), 10,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { grab(m, qf_, gf_, have_f_); });

        timer_ = create_wall_timer(std::chrono::microseconds(1000000 / std::max(1, rate)),
                                   [this] { tick(); });
        RCLCPP_INFO(get_logger(), "openarmx_bilateral relay: side=%s mode=%s n=%d sign=%.0f gripper=%s",
                    side.c_str(), bilateral_ ? "BILATERAL" : "unilateral",
                    nj_, couple_sign_, include_gripper_ ? "yes" : "no");
    }

private:
    void grab(const sensor_msgs::msg::JointState::SharedPtr& m,
              std::vector<double>& q, double& grip, bool& have) {
        for (int j = 0; j < nj_; ++j)
            for (size_t k = 0; k < m->name.size(); ++k)
                if (m->name[k] == names_[j] && k < m->position.size()) { q[j] = m->position[k]; break; }
        for (size_t k = 0; k < m->name.size(); ++k)
            if (m->name[k] == gripper_name_ && k < m->position.size()) { grip = m->position[k]; break; }
        have = true;
    }
    std_msgs::msg::Float64MultiArray make(const std::vector<double>& src_arm, double src_grip) {
        std_msgs::msg::Float64MultiArray msg;
        for (int j = 0; j < nj_; ++j) msg.data.push_back(couple_sign_ * src_arm[j]);
        if (include_gripper_) msg.data.push_back(src_grip);   // gripper relayed directly
        return msg;
    }
    void tick() {
        if (have_l_) follower_pub_->publish(make(ql_, gl_));            // follower tracks leader
        if (bilateral_ && have_f_) leader_pub_->publish(make(qf_, gf_)); // leader feels follower
    }

    int nj_ = 7;
    bool include_gripper_ = true, bilateral_ = false, have_l_ = false, have_f_ = false;
    double couple_sign_ = -1.0, gl_ = 0.0, gf_ = 0.0;
    std::string gripper_name_;
    std::vector<std::string> names_;
    std::vector<double> ql_, qf_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leader_pub_, follower_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_, follower_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelayNode>());
    rclcpp::shutdown();
    return 0;
}
