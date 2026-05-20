#include "ur10e_teleop_unilateral_vive_cpp/vive_leader_node.hpp"

#include <chrono>
#include <cmath>

#include "ur10e_teleop_unilateral_vive_cpp/ur_ik.hpp"

namespace ur10e_teleop_unilateral_vive_cpp {

namespace {

// Quintic s(τ) ∈ [0,1] for τ ∈ [0,1]. Smooth start/stop (zero vel/acc
// at endpoints). Matches the homing curve used by the UR3e leader.
inline double quintic(double tau) {
  if (tau <= 0.0) return 0.0;
  if (tau >= 1.0) return 1.0;
  const double t3 = tau * tau * tau;
  return t3 * (10.0 - 15.0 * tau + 6.0 * tau * tau);
}

inline double now_sec(const rclcpp::Node* node) {
  return node->now().seconds();
}

}  // namespace

ViveLeaderNode::ViveLeaderNode(const Options& opts)
    : rclcpp::Node("vive_leader_node"), opts_(opts) {
  rt_cfg_.enabled      = opts.use_rt;
  rt_cfg_.priority     = opts.rt_priority;
  rt_cfg_.cpu_affinity = opts.rt_cpu;

  if (!opts.config_path.empty()) {
    if (!load_config(opts.config_path, cfg_)) {
      RCLCPP_WARN(get_logger(), "config load failed; using defaults");
    } else {
      RCLCPP_INFO(get_logger(), "config loaded: %s", opts.config_path.c_str());
    }
  }

  home_qpos_ = cfg_.leader_home;

  // Calibration (optional at load time — if missing, identity transform
  // is used and the operator can populate later via the calibrate.py
  // helper).
  if (!opts.calib_path.empty()) {
    if (calib_.load(opts.calib_path)) {
      RCLCPP_INFO(get_logger(), "calibration loaded: %s",
                  opts.calib_path.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
        "calibration load failed (%s) — using identity. "
        "Run script/calibrate.py to capture a calibration.",
        opts.calib_path.c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(),
        "no --calib path provided — using identity tracker→base transform");
  }

  // QoS — identical to UR3e leader so wire format is bit-compatible.
  rclcpp::QoS state_qos{10};
  rclcpp::QoS latched_qos{1};
  latched_qos.reliable().transient_local();

  state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/ur10e/leader/joint_state", state_qos);
  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos);

  peer_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/ur10e/follower/joint_state", state_qos,
      std::bind(&ViveLeaderNode::peer_cb, this, std::placeholders::_1));
  mode_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos,
      std::bind(&ViveLeaderNode::mode_cb, this, std::placeholders::_1));
  reset_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/ur10e/reset", latched_qos,
      std::bind(&ViveLeaderNode::reset_cb, this, std::placeholders::_1));

  publish_mode(MODE_PAUSED);
}

ViveLeaderNode::~ViveLeaderNode() { stop(); }

bool ViveLeaderNode::run() {
  if (running_.exchange(true)) return true;

  tracker_ = std::make_unique<ViveTracker>();
  ViveTracker::Config tc;
  tc.target_serial      = opts_.tracker_serial;
  tc.init_timeout_sec   = opts_.tracker_init_timeout_sec;
  if (!tracker_->init(tc)) {
    RCLCPP_ERROR(get_logger(),
        "ViveTracker.init() failed — is SteamVR running and the tracker on?");
    running_ = false;
    return false;
  }
  RCLCPP_INFO(get_logger(), "tracker ready (serial=%s)",
              tracker_->serial().c_str());

  if (cfg_.auto_home_on_start) {
    publish_mode(MODE_HOMING, now_sec(this), cfg_.homing_duration);
    RCLCPP_INFO(get_logger(), "auto_home_on_start → HOMING in %.1fs",
                cfg_.homing_duration);
  }

  control_thread_ = std::thread([this]() {
    if (rt_cfg_.enabled) init_rt_thread(rt_cfg_);
    control_loop();
  });
  return true;
}

void ViveLeaderNode::stop() {
  if (!running_.exchange(false)) return;
  if (control_thread_.joinable()) control_thread_.join();
  if (tracker_) {
    tracker_->shutdown();
    tracker_.reset();
  }
}

void ViveLeaderNode::control_loop() {
  using clk = std::chrono::steady_clock;
  const auto period =
      std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / opts_.control_rate_hz));
  auto next = clk::now() + period;

  // Virtual joint state — seeded at home.
  std::array<double, 6> q  = home_qpos_;
  std::array<double, 6> dq{};
  std::array<double, 6> q_prev = home_qpos_;
  std::array<double, 6> q_home_start = home_qpos_;

  int prev_state = MODE_PAUSED;
  int last_reset_counter = reset_counter_.load();
  const double startup_time = now_sec(this);
  const double RESET_STARTUP_GRACE = 2.0;

  RCLCPP_INFO(get_logger(),
              "control_loop entered  rate=%.0fHz  rt=%s",
              opts_.control_rate_hz, rt_cfg_.enabled ? "ON" : "OFF");

  while (running_) {
    const double t_now = now_sec(this);

    // ---- snapshot mode ----
    int cur_state;
    double m_t_start, m_duration;
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      cur_state  = mode_state_;
      m_t_start  = mode_t_start_;
      m_duration = mode_duration_;
    }

    // ---- reset → HOMING (only from ACTIVE, with startup grace) ----
    int cur_reset = reset_counter_.load();
    if (cur_reset != last_reset_counter) {
      last_reset_counter = cur_reset;
      if (t_now - startup_time < RESET_STARTUP_GRACE) {
        RCLCPP_INFO(get_logger(),
            "reset ignored during startup grace (counter=%d)", cur_reset);
      } else if (cur_state == MODE_ACTIVE) {
        publish_mode(MODE_HOMING, t_now, cfg_.homing_duration);
      }
    }

    // ---- state transition bookkeeping ----
    if (cur_state != prev_state) {
      RCLCPP_INFO(get_logger(), "state %d → %d", prev_state, cur_state);
      if (cur_state == MODE_HOMING) {
        q_home_start = q;
      }
      prev_state = cur_state;
    }

    // ---- compute virtual joint state per mode ----
    std::array<double, 6> q_target = q;

    switch (cur_state) {
      case MODE_ACTIVE:
      case MODE_FREEDRIVE: {
        // Track Vive tracker via IK.
        Eigen::Matrix4d T_tracker;
        if (tracker_->poll(T_tracker)) {
          const Eigen::Matrix4d T_ee = calib_.apply(T_tracker);
          std::array<double, 6> q_sol;
          if (ur_ik_solve(T_ee, q, opts_.robot_type, q_sol)) {
            q_target = q_sol;
          } else {
            // non-convergence — accept the best-effort q_sol but log
            // sparingly (every 500 cycles = once per second at 500 Hz)
            q_target = q_sol;
            static int warn_div = 0;
            if ((warn_div++ % 500) == 0) {
              RCLCPP_WARN(get_logger(),
                "IK did not converge (this cycle's q_sol applied anyway)");
            }
          }
        } else {
          // tracker lost — hold last q
          q_target = q;
        }
        break;
      }
      case MODE_HOMING: {
        const double tau = (m_duration > 0.0)
            ? (t_now - m_t_start) / m_duration : 1.0;
        const double s = quintic(tau);
        for (int i = 0; i < 6; ++i) {
          q_target[i] = q_home_start[i] + s * (home_qpos_[i] - q_home_start[i]);
        }
        if (tau >= 1.0) {
          publish_mode(MODE_PAUSED);
        }
        break;
      }
      case MODE_PAUSED:
      default:
        q_target = q;  // freeze
        break;
    }

    // ---- finite-difference dq with LPF ----
    const double dt = 1.0 / opts_.control_rate_hz;
    const double a  = opts_.dq_filter_alpha;
    for (int i = 0; i < 6; ++i) {
      const double dq_raw = (q_target[i] - q_prev[i]) / dt;
      dq[i] = a * dq_raw + (1.0 - a) * dq[i];
    }
    q_prev = q;
    q = q_target;

    publish_state(q, dq);

    // ---- pacing ----
    std::this_thread::sleep_until(next);
    next += period;
  }
}

void ViveLeaderNode::peer_cb(
    const sensor_msgs::msg::JointState::SharedPtr /*msg*/) {
  // Unilateral: leader does not consume follower state.
  // Subscription kept for symmetry with the UR3e leader.
}

void ViveLeaderNode::mode_cb(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() < 3) return;
  std::lock_guard<std::mutex> lk(mode_mtx_);
  mode_state_   = static_cast<int>(msg->data[0]);
  mode_t_start_ = msg->data[1];
  mode_duration_ = msg->data[2];
}

void ViveLeaderNode::reset_cb(
    const std_msgs::msg::Int32::SharedPtr msg) {
  reset_counter_.store(msg->data);
}

void ViveLeaderNode::publish_state(const std::array<double, 6>& q,
                                    const std::array<double, 6>& dq) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.position.assign(q.begin(), q.end());
  msg.velocity.assign(dq.begin(), dq.end());
  state_pub_->publish(msg);
}

void ViveLeaderNode::publish_mode(int mode, double t_start, double duration) {
  // Update local cache so our own loop sees the change.
  {
    std::lock_guard<std::mutex> lk(mode_mtx_);
    mode_state_   = mode;
    mode_t_start_ = t_start;
    mode_duration_ = duration;
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

}  // namespace ur10e_teleop_unilateral_vive_cpp
