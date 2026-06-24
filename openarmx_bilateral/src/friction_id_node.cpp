// friction_id_node.cpp — Phase 3 step 1: friction identification data collector.
//
// Drives ONE follower joint at a time at several CONSTANT speeds (both signs),
// shuttling within +-range of its start pose, while gravity comp is ON and the
// position hold is OFF (follower kp=0). At constant speed accel~=0, so
//     motor_effort = gravity + friction   =>   friction = effort - gravity_cmd.
// We log effort AND the gravity-comp command, so friction is recovered directly
// (no offline KDL). Offline we fit per joint:
//     tau_fric(w) = Fc*tanh(0.1*k*w) + Fv*w + Fo.
//
// Setup is done by friction_id.launch.py (velocity controller + effort+gravity +
// follower kp:=0). DO NOT run the relay / move the leader during ID.
//
// CSV columns (one row per follower joint-state sample, only the excited joint):
//   t, jexc, q, vel_act, vel_cmd, eff, grav, fric(=eff-grav), speed, dir
//
// SAFETY: shuttles within [q0-range, q0+range] of each joint's pose at start;
// flips direction at the bounds; commands 0 to all other joints. Start the arm
// in a roughly mid-range, collision-free pose. Ctrl+C any time -> publishes 0.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class FrictionIdNode : public rclcpp::Node {
public:
    FrictionIdNode() : Node("openarmx_friction_id") {
        declare_parameter<std::string>("joint_prefix", "openarmx_left_joint");
        declare_parameter<int>("n_joints", 7);
        declare_parameter<std::vector<int64_t>>("joints",
            std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7});
        declare_parameter<std::vector<double>>("speeds",
            std::vector<double>{0.1, 0.2, 0.3, 0.5, 0.8, 1.2});  // rad/s magnitudes
        declare_parameter<double>("range", 0.45);    // fallback half-range [rad] if no abs bounds
        declare_parameter<double>("dwell", 5.0);      // seconds per speed level
        // friction-test-only per-joint ABSOLUTE limits [rad] (NOT control limits).
        // If both arrays are length n_joints, shuttle stays inside [lo,hi]; else
        // fall back to +-range around the start pose. margin = cushion kept inside
        // the stated limit to absorb braking overshoot.
        declare_parameter<std::vector<double>>("joint_lo", std::vector<double>{});
        declare_parameter<std::vector<double>>("joint_hi", std::vector<double>{});
        declare_parameter<double>("margin", 0.087);   // ~5 deg
        declare_parameter<std::string>("vel_cmd",
            "/follower/left_forward_velocity_controller/commands");
        declare_parameter<std::string>("states", "/follower/joint_states");
        declare_parameter<std::string>("grav",
            "/follower/left_forward_effort_controller/commands");
        declare_parameter<std::string>("csv", "/tmp/friction_id.csv");
        declare_parameter<int>("rate_hz", 200);

        nj_ = get_parameter("n_joints").as_int();
        std::string jp = get_parameter("joint_prefix").as_string();
        for (int i = 0; i < nj_; ++i) names_.push_back(jp + std::to_string(i + 1));
        speeds_ = get_parameter("speeds").as_double_array();
        range_ = get_parameter("range").as_double();
        dwell_ = get_parameter("dwell").as_double();
        lo_ = get_parameter("joint_lo").as_double_array();
        hi_ = get_parameter("joint_hi").as_double_array();
        margin_ = get_parameter("margin").as_double();
        use_abs_ = ((int)lo_.size() == nj_ && (int)hi_.size() == nj_);
        RCLCPP_INFO(get_logger(), "bounds: %s",
                    use_abs_ ? "per-joint absolute [lo,hi]" : "fallback +-range around start");

        // SCHEDULE: ALWAYS the full 1..nj_ sweep, built directly here. The old
        // path (seq_ from the joints param + a drop-out-of-range guard) produced
        // a wrong [3..7] schedule in the field, so it is removed entirely.
        {   // diagnostic only: what the joints param actually held
            std::string raw;
            for (auto v : get_parameter("joints").as_integer_array())
                raw += std::to_string(v) + " ";
            RCLCPP_INFO(get_logger(), "joints param raw=[%s] (IGNORED; full 1..%d sweep)",
                        raw.c_str(), nj_);
        }
        seq_.clear();
        for (int j = 1; j <= nj_; ++j) seq_.push_back(j);   // 1,2,...,nj_  (no guard)
        if (speeds_.empty()) speeds_ = {0.1, 0.2, 0.3, 0.5, 0.8, 1.2};

        // diagnostic: show exactly what schedule/topics this node received
        std::string sched;
        for (int j : seq_) sched += std::to_string(j) + " ";
        RCLCPP_INFO(get_logger(),
            "DIAG n_joints=%d joints_param=%zu SCHEDULE=[%s] states=%s vel_cmd=%s grav=%s",
            nj_, get_parameter("joints").as_integer_array().size(), sched.c_str(),
            get_parameter("states").as_string().c_str(),
            get_parameter("vel_cmd").as_string().c_str(),
            get_parameter("grav").as_string().c_str());

        q_.assign(nj_, 0.0); v_.assign(nj_, 0.0); eff_.assign(nj_, 0.0); grav_.assign(nj_, 0.0);

        std::string path = get_parameter("csv").as_string();
        f_.open(path);
        if (f_.is_open()) f_ << "t,jexc,q,vel_act,vel_cmd,eff,grav,fric,speed,dir\n";
        RCLCPP_INFO(get_logger(), "friction ID -> %s : %zu joints x %zu speeds, range=%.2f dwell=%.1fs",
                    path.c_str(), seq_.size(), speeds_.size(), range_, dwell_);

        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            get_parameter("vel_cmd").as_string(), 10);
        grav_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("grav").as_string(), 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
                for (int j = 0; j < nj_ && j < (int)m->data.size(); ++j) grav_[j] = m->data[j];
            });
        st_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("states").as_string(), 50,
            [this](sensor_msgs::msg::JointState::SharedPtr m) { onState(m); });

        int rate = std::max(1, (int)get_parameter("rate_hz").as_int());
        timer_ = create_wall_timer(std::chrono::microseconds(1000000 / rate),
                                   [this] { tick(); });
        t0_ = now();
    }
    ~FrictionIdNode() override { if (f_.is_open()) f_.close(); }

private:
    void onState(const sensor_msgs::msg::JointState::SharedPtr& m) {
        for (int j = 0; j < nj_; ++j)
            for (size_t k = 0; k < m->name.size(); ++k)
                if (m->name[k] == names_[j]) {
                    if (k < m->position.size()) q_[j] = m->position[k];
                    if (k < m->velocity.size()) v_[j] = m->velocity[k];
                    if (k < m->effort.size())   eff_[j] = m->effort[k];
                    break;
                }
        have_state_ = true;
        if (started_ && !done_ && cur_ < (int)seq_.size()) {
            int j = seq_[cur_] - 1;
            if (j >= 0 && j < nj_ && f_.is_open()) {
                double t = (now() - t0_).seconds();
                double fr = eff_[j] - grav_[j];
                char buf[200];
                std::snprintf(buf, sizeof(buf),
                    "%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%d\n",
                    t, j + 1, q_[j], v_[j], dir_ * speeds_[spd_], eff_[j], grav_[j],
                    fr, speeds_[spd_], dir_);
                f_ << buf;
            }
        }
    }

    void publish(double cmd_for_excited) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(nj_, 0.0);
        if (cur_ < (int)seq_.size()) {
            int j = seq_[cur_] - 1;
            if (j >= 0 && j < nj_) msg.data[j] = cmd_for_excited;
        }
        cmd_pub_->publish(msg);
    }

    void tick() {
        if (!have_state_) return;
        if (!started_) {                       // capture start pose, begin
            q0_ = q_; started_ = true;
            cur_ = 0; spd_ = 0; dir_ = 1; seg_t0_ = now();
            RCLCPP_INFO(get_logger(), "start: joint %d @ speed %.2f", seq_[cur_], speeds_[spd_]);
            return;
        }
        if (done_) { publish(0.0); return; }

        int j = seq_[cur_] - 1;
        double lo, hi;
        if (use_abs_) { lo = lo_[j]; hi = hi_[j]; }
        else          { lo = q0_[j] - range_; hi = q0_[j] + range_; }
        if (hi < lo) std::swap(lo, hi);
        // keep a margin inside the stated limit (shrink if band is narrow)
        double m = std::min(margin_, 0.3 * (hi - lo));
        double Blo = lo + m, Bhi = hi - m;
        // anticipate braking distance so overshoot stays within the margin
        double brake = std::min(0.4 * (Bhi - Blo), 0.05 + 0.2 * speeds_[spd_]);
        // shuttle: flip before the bounds (and hard-steer back if already past)
        if (q_[j] >= Bhi - brake) dir_ = -1;
        else if (q_[j] <= Blo + brake) dir_ = 1;
        publish(dir_ * speeds_[spd_]);

        // advance speed level / joint after dwell
        if ((now() - seg_t0_).seconds() >= dwell_) {
            seg_t0_ = now();
            if (++spd_ >= (int)speeds_.size()) {
                spd_ = 0;
                publish(0.0);
                if (++cur_ >= (int)seq_.size()) {
                    done_ = true; publish(0.0);
                    RCLCPP_INFO(get_logger(), "friction ID DONE -> CSV written. Ctrl+C.");
                    return;
                }
                dir_ = 1;
                RCLCPP_INFO(get_logger(), "next: joint %d", seq_[cur_]);
            } else {
                RCLCPP_INFO(get_logger(), "joint %d speed %.2f", seq_[cur_], speeds_[spd_]);
            }
        }
    }

    int nj_ = 7;
    std::vector<std::string> names_;
    std::vector<int> seq_;
    std::vector<double> speeds_, q_, v_, eff_, grav_, q0_, lo_, hi_;
    double range_ = 0.45, dwell_ = 5.0, margin_ = 0.087;
    bool use_abs_ = false, have_state_ = false, started_ = false, done_ = false;
    int cur_ = 0, spd_ = 0, dir_ = 1;
    rclcpp::Time t0_, seg_t0_;
    std::ofstream f_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr grav_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr st_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrictionIdNode>());
    rclcpp::shutdown();
    return 0;
}
