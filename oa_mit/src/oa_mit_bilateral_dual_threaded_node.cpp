// oa_mit_bilateral_dual_threaded_node.cpp — Version B: single process, but ONE
// THREAD PER ARM (like enactic / ur10e_teleop). Each thread runs its own arm's
// CAN read/command loop at a steady rate; peer state is exchanged in-memory via
// a small mutex-protected snapshot. This decouples the two CAN buses' timing so
// each arm gets regular MIT setpoints -> low-inertia wrist (q3-q7) shouldn't jerk
// (the single-loop dual node serialized both buses -> suspected 팡팡 cause).
//
// Same control law/gains as oa_mit_bilateral_dual_node (per-joint kp profile,
// gravity FF, couple_sign). Compare against Version A (official ros2_control
// follower). couple via arm_side; leader=can0/can2 etc.

#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "oa_mit_arm.hpp"

// peer snapshot (one arm writes its state, the other reads it)
struct PeerState {
    std::mutex m;
    std::vector<double> qm, dqm;
    bool have = false;
};

class BilateralDualThreaded : public rclcpp::Node {
public:
    BilateralDualThreaded() : Node("oa_mit_bilateral_dual_threaded") {
        declare_parameter<std::string>("arm_side", "right_arm");
        declare_parameter<std::string>("leader_can", "auto");
        declare_parameter<std::string>("follower_can", "auto");
        declare_parameter<std::string>("urdf_path", "/tmp/v10_bimanual.urdf");
        declare_parameter<int>("control_rate_hz", 250);
        declare_parameter<double>("g_scale", 0.9);
        declare_parameter<double>("gy_override", 0.0);
        declare_parameter<double>("leader_gain", 0.0);
        declare_parameter<double>("follower_gain", 0.0);
        declare_parameter<double>("leader_kd_scale", 1.0);
        declare_parameter<double>("follower_kd_scale", 1.0);
        declare_parameter<bool>("vel_ff", false);
        declare_parameter<double>("couple_sign", -1.0);
        declare_parameter<bool>("verbose", false);
        declare_parameter<bool>("enable_leader", true);
        declare_parameter<bool>("enable_follower", true);

        side_ = get_parameter("arm_side").as_string();
        const bool is_left = (side_ == "left_arm");
        std::string lcan = get_parameter("leader_can").as_string();
        std::string fcan = get_parameter("follower_can").as_string();
        if (lcan == "auto") lcan = is_left ? "can1" : "can0";
        if (fcan == "auto") fcan = is_left ? "can3" : "can2";
        std::string urdf = get_parameter("urdf_path").as_string();
        rate_ = get_parameter("control_rate_hz").as_int();
        double gscale = get_parameter("g_scale").as_double();
        double gx = 0.0, gz = 0.0;
        double gy = is_left ? 9.81 : -9.81;
        double gy_ovr = get_parameter("gy_override").as_double();
        if (gy_ovr != 0.0) gy = gy_ovr;
        couple_sign_ = get_parameter("couple_sign").as_double();
        verbose_ = get_parameter("verbose").as_bool();
        en_leader_ = get_parameter("enable_leader").as_bool();
        en_follower_ = get_parameter("enable_follower").as_bool();

        std::string root = is_left ? "openarmx_left_link0" : "openarmx_right_link0";
        std::string leaf = is_left ? "openarmx_left_link7" : "openarmx_right_link7";
        const std::vector<double> tau_lim = {10, 10, 5, 5, 2, 2, 2};
        const std::vector<double> KP_BASE = {50, 50, 50, 50, 10, 10, 10};
        const std::vector<double> KD_BASE = {2.5, 2.5, 2.5, 2.5, 0.5, 0.5, 0.5};
        double lgain = get_parameter("leader_gain").as_double();
        double fgain = get_parameter("follower_gain").as_double();
        double lkds = get_parameter("leader_kd_scale").as_double();
        double fkds = get_parameter("follower_kd_scale").as_double();
        bool velff = get_parameter("vel_ff").as_bool();
        auto setup = [&](Arm& a, const std::string& nm, double gain, double kd_scale) {
            a.name = nm; a.g_scale = gscale; a.tau_limit = tau_lim;
            a.gain = gain; a.vel_ff = velff;
            a.kp_vec.assign(KP_BASE.size(), 0.0);
            a.kd_vec.assign(KD_BASE.size(), 0.0);
            for (size_t i = 0; i < KP_BASE.size(); ++i) {
                a.kp_vec[i] = KP_BASE[i] * gain;
                a.kd_vec[i] = KD_BASE[i] * gain * kd_scale;
            }
        };
        setup(leader_, "leader", lgain, lkds);
        setup(follower_, "follower", fgain, fkds);

        if (!en_leader_ && !en_follower_)
            throw std::runtime_error("both arms disabled");
        RCLCPP_INFO(get_logger(),
            "THREADED: leader=%s follower=%s side=%s rate=%d | lgain=%.2f fgain=%.2f "
            "vel_ff=%s sign=%.0f", lcan.c_str(), fcan.c_str(), side_.c_str(), rate_,
            lgain, fgain, velff ? "on" : "off", couple_sign_);
        if (en_leader_ && !leader_.init(lcan, urdf, root, leaf, gx, gy, gz, get_logger()))
            throw std::runtime_error("leader init failed");
        if (en_follower_ && !follower_.init(fcan, urdf, root, leaf, gx, gy, gz, get_logger()))
            throw std::runtime_error("follower init failed");

        running_.store(true);
        if (en_leader_)
            th_leader_ = std::thread([this] {
                arm_loop(leader_, leader_state_, follower_state_, en_follower_, "leader");
            });
        if (en_follower_)
            th_follower_ = std::thread([this] {
                arm_loop(follower_, follower_state_, leader_state_, en_leader_, "follower");
            });
        RCLCPP_INFO(get_logger(), "threads started");
    }

    ~BilateralDualThreaded() override {
        running_.store(false);
        if (th_leader_.joinable()) th_leader_.join();
        if (th_follower_.joinable()) th_follower_.join();
        leader_.relax_disable();
        follower_.relax_disable();
    }

private:
    // one arm's dedicated loop: read own CAN -> publish own snapshot -> read peer
    // snapshot -> gravity -> command own CAN, at a steady rate.
    void arm_loop(Arm& self, PeerState& own, PeerState& peer, bool peer_enabled,
                  const char* tag) {
        const auto period = std::chrono::microseconds(1000000 / std::max(1, rate_));
        auto next = std::chrono::steady_clock::now();
        auto t_prev = next; bool first = true;
        double rsum = 0, rmax = 0; long rn = 0;
        while (running_.load()) {
            auto now = std::chrono::steady_clock::now();
            if (!first) { double d = std::chrono::duration<double>(now - t_prev).count();
                          rsum += d; if (d > rmax) rmax = d; if (++rn >= 500) {
                              if (verbose_) RCLCPP_INFO(get_logger(),
                                  "[%s] %.0f Hz, max gap %.1f ms", tag, 500.0 / rsum, rmax * 1000.0);
                              rsum = 0; rmax = 0; rn = 0; } }
            first = false; t_prev = now;

            self.read();
            { std::lock_guard<std::mutex> lk(own.m);
              own.qm = self.qm; own.dqm = self.dqm; own.have = true; }
            std::vector<double> pqm, pdqm; bool got = false;
            { std::lock_guard<std::mutex> lk(peer.m);
              if (peer.have) { pqm = peer.qm; pdqm = peer.dqm; got = true; } }
            self.gravity();
            self.command(peer_enabled && got && self.gain > 0.0, couple_sign_, pqm, pdqm);

            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    Arm leader_, follower_;
    PeerState leader_state_, follower_state_;
    std::thread th_leader_, th_follower_;
    std::atomic<bool> running_{false};
    std::string side_;
    int rate_ = 250;
    double couple_sign_ = -1.0;
    bool verbose_ = false, en_leader_ = true, en_follower_ = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BilateralDualThreaded>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("oa_mit_bilateral_dual_threaded"), "fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
