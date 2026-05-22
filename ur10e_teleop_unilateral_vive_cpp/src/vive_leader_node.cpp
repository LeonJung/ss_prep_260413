#include "ur10e_teleop_unilateral_vive_cpp/vive_leader_node.hpp"

#include <chrono>
#include <cmath>

#include "ur10e_teleop_unilateral_vive_cpp/ur_ik.hpp"
#include "ur10e_teleop_unilateral_vive_cpp/ur_jacobian.hpp"

namespace ur10e_teleop_unilateral_vive_cpp {

namespace {

// Quintic smoothstep s(τ) ∈ [0,1] for τ ∈ [0,1].
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

  // Build the per-arm runtime list from the configured ArmOptions.
  // An arm with empty tracker_serial is skipped (single-arm fallback).
  auto add_arm = [&](const ArmOptions& a, const char* side_label,
                     const Vec6& side_home) {
    if (a.tracker_serial.empty() && a.calib_path.empty() &&
        a.topic_prefix.empty()) {
      // fully unconfigured — skip
      return;
    }
    if (a.tracker_serial.empty()) {
      RCLCPP_WARN(get_logger(),
        "%s side has empty tracker_serial — skipping (set --%s-serial)",
        side_label, side_label);
      return;
    }
    Arm arm;
    arm.opts = a;
    arm.home_qpos = side_home;
    arm.q = arm.home_qpos;
    arm.q_prev = arm.home_qpos;
    arm.q_home_start = arm.home_qpos;

    if (!a.calib_path.empty()) {
      if (arm.calib.load(a.calib_path)) {
        RCLCPP_INFO(get_logger(), "%s calib loaded: %s",
                    side_label, a.calib_path.c_str());
        arm.tare_pending = false;  // explicit calibration overrides auto-tare
      } else {
        RCLCPP_WARN(get_logger(),
          "%s calib load failed (%s) — will auto-tare on first ACTIVE poll",
          side_label, a.calib_path.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(),
        "%s side: no calib path — will auto-tare on first ACTIVE poll",
        side_label);
    }

    rclcpp::QoS state_qos{10};
    arm.state_pub = create_publisher<sensor_msgs::msg::JointState>(
        a.topic_prefix + "/leader/joint_state", state_qos);

    arms_.push_back(std::move(arm));
  };

  // home_qpos is in the FK robot's joint space — since robot_type
  // defaults to ur10e (the actual follower), follower_home_* is the
  // right reference, not leader_home_* (which was UR3e-space carried
  // over from the parent unilateral package).
  add_arm(opts.left,  "left",  cfg_.follower_home_left);
  add_arm(opts.right, "right", cfg_.follower_home_right);

  if (arms_.empty()) {
    RCLCPP_ERROR(get_logger(),
      "no arms configured — set at least one of --left-serial / --right-serial");
    throw std::runtime_error("ViveLeaderNode: no arms configured");
  }

  // Shared mode / reset
  rclcpp::QoS latched_qos{1};
  latched_qos.reliable().transient_local();

  mode_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur10e/mode", latched_qos);
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

  // Init each arm's tracker. OpenVR allows multiple background apps;
  // ViveTracker calls VR_Init internally, but the first call wins —
  // subsequent inits short-circuit (the runtime is process-wide).
  for (auto& arm : arms_) {
    arm.tracker = std::make_unique<ViveTracker>();
    ViveTracker::Config tc;
    tc.target_serial    = arm.opts.tracker_serial;
    tc.init_timeout_sec = opts_.tracker_init_timeout_sec;
    if (!arm.tracker->init(tc)) {
      RCLCPP_ERROR(get_logger(),
          "tracker init failed for serial='%s' (topic %s)",
          arm.opts.tracker_serial.c_str(),
          arm.opts.topic_prefix.c_str());
      running_ = false;
      return false;
    }
    RCLCPP_INFO(get_logger(), "tracker ready: %s → %s",
                arm.tracker->serial().c_str(),
                arm.opts.topic_prefix.c_str());
  }

  // Startup tare: capture each tracker's CURRENT pose right now and
  // shift its calibration translation so this pose maps to UR home EE.
  // The calibration's rotation (axis alignment from vive_calibrate)
  // is preserved — only the origin is re-anchored to "wherever Vive
  // is when the node starts", which is what the operator expects.
  //
  // Falls back gracefully if a tracker is briefly invalid: leaves
  // tare_pending = true so the first valid ACTIVE-mode poll completes
  // the tare instead.
  for (auto& arm : arms_) {
    Eigen::Matrix4d T_now;
    bool got = false;
    for (int tries = 0; tries < 50; ++tries) {   // ≈ 1 s @ 20 ms
      if (arm.tracker->poll(T_now)) { got = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!got) {
      RCLCPP_WARN(get_logger(),
          "%s: startup tracker poll timed out — will tare on first ACTIVE poll",
          arm.opts.topic_prefix.c_str());
      arm.tare_pending = true;
      continue;
    }
    Eigen::Matrix4d T_ur_home;
    forward_kinematics(arm.home_qpos, opts_.robot_type, T_ur_home);
    const Eigen::Matrix4d T_now_ur = arm.calib.apply(T_now);
    Eigen::Matrix4d C = arm.calib.transform();
    C.block<3, 1>(0, 3) +=
        (T_ur_home.block<3, 1>(0, 3) - T_now_ur.block<3, 1>(0, 3));
    arm.calib = Calibration(C);

    // Cache the tracker's startup pose for delta-based pose tracking
    // (position AND orientation). At startup, the delta is identity,
    // so T_ee = T_ur_home — no jump regardless of how the tracker is
    // oriented when the node comes up.
    arm.T_tracker_startup = T_now;
    arm.has_startup_pose = true;

    arm.tare_pending = false;
    RCLCPP_INFO(get_logger(),
        "%s: startup tare — current tracker pose ↦ UR home EE "
        "(rotation kept; only translation shifted)",
        arm.opts.topic_prefix.c_str());
  }

  if (cfg_.auto_home_on_start) {
    publish_mode(MODE_HOMING, now_sec(this), cfg_.homing_duration);
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
  for (auto& arm : arms_) {
    if (arm.tracker) {
      arm.tracker->shutdown();
      arm.tracker.reset();
    }
  }
  arms_.clear();
}

void ViveLeaderNode::control_loop() {
  using clk = std::chrono::steady_clock;
  const auto period =
      std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / opts_.control_rate_hz));
  auto next = clk::now() + period;

  int prev_state = MODE_PAUSED;
  int last_reset_counter = reset_counter_.load();
  const double startup_time = now_sec(this);
  const double RESET_STARTUP_GRACE = 2.0;

  RCLCPP_INFO(get_logger(),
              "control_loop entered  arms=%zu  rate=%.0fHz  rt=%s",
              arms_.size(), opts_.control_rate_hz,
              rt_cfg_.enabled ? "ON" : "OFF");

  while (running_) {
    const double t_now = now_sec(this);

    int cur_state;
    double m_t_start, m_duration;
    {
      std::lock_guard<std::mutex> lk(mode_mtx_);
      cur_state  = mode_state_;
      m_t_start  = mode_t_start_;
      m_duration = mode_duration_;
    }

    // reset → HOMING
    int cur_reset = reset_counter_.load();
    if (cur_reset != last_reset_counter) {
      last_reset_counter = cur_reset;
      if (t_now - startup_time >= RESET_STARTUP_GRACE
          && cur_state == MODE_ACTIVE) {
        publish_mode(MODE_HOMING, t_now, cfg_.homing_duration);
      }
    }

    if (cur_state != prev_state) {
      RCLCPP_INFO(get_logger(), "state %d → %d", prev_state, cur_state);
      if (cur_state == MODE_HOMING) {
        for (auto& a : arms_) a.q_home_start = a.q;
      }
      // Re-tare on every entry into ACTIVE / FREEDRIVE so the operator's
      // current tracker pose becomes the new "zero" — both position and
      // orientation (via T_tracker_startup for delta tracking). Keeps
      // the calibration's axis alignment (rotation part of calib)
      // untouched; only the origin moves.
      if (cur_state == MODE_ACTIVE || cur_state == MODE_FREEDRIVE) {
        for (auto& a : arms_) a.tare_pending = true;
      }
      prev_state = cur_state;
    }

    // Per-arm update
    for (auto& arm : arms_) {
      tick_arm(arm, cur_state, t_now, m_t_start, m_duration);
      publish_arm_state(arm);
    }

    if (cur_state == MODE_HOMING && m_duration > 0.0
        && (t_now - m_t_start) >= m_duration) {
      publish_mode(MODE_PAUSED);
    }

    std::this_thread::sleep_until(next);
    next += period;
  }
}

void ViveLeaderNode::tick_arm(Arm& arm, int cur_state, double t_now,
                               double m_t_start, double m_duration) {
  std::array<double, 6> q_target = arm.q;

  switch (cur_state) {
    case MODE_ACTIVE:
    case MODE_FREEDRIVE: {
      Eigen::Matrix4d T_tracker;
      if (arm.tracker && arm.tracker->poll(T_tracker)) {
        // Tare on first valid ACTIVE/FREEDRIVE poll after entry. The
        // state-transition handler in control_loop sets tare_pending=true
        // each time we enter one of these modes, so this fires once per
        // entry (also used as a fallback if the startup tare in run()
        // was skipped due to a briefly invalid poll).
        if (arm.tare_pending) {
          Eigen::Matrix4d T_ur_home_local;
          forward_kinematics(arm.home_qpos, opts_.robot_type, T_ur_home_local);
          const Eigen::Matrix4d T_now_ur = arm.calib.apply(T_tracker);
          Eigen::Matrix4d C = arm.calib.transform();
          C.block<3, 1>(0, 3) +=
              (T_ur_home_local.block<3, 1>(0, 3) - T_now_ur.block<3, 1>(0, 3));
          arm.calib = Calibration(C);
          arm.T_tracker_startup = T_tracker;
          arm.has_startup_pose = true;
          arm.tare_pending = false;
          RCLCPP_INFO(get_logger(),
              "%s: tare on ACTIVE entry — current tracker pose ↦ UR home EE",
              arm.opts.topic_prefix.c_str());
        }

        // Build IK target via delta-based pose tracking. At startup the
        // tracker pose is captured; subsequent samples are expressed as
        // deltas (translation + rotation) relative to that startup pose.
        // The cali's rotation maps the delta from SteamVR frame to UR
        // base frame (with rotation conjugation for the orientation
        // part), then we apply the delta on top of T_ur_home. This
        // gives BOTH:
        //   - no jump at startup (identity delta → T_ee = T_ur_home)
        //   - faithful axis-aligned position + orientation tracking
        //     once the operator starts moving.
        Eigen::Matrix4d T_ur_home;
        forward_kinematics(arm.home_qpos, opts_.robot_type, T_ur_home);
        Eigen::Matrix4d T_ee = T_ur_home;
        if (arm.has_startup_pose) {
          const Eigen::Matrix3d R_cali = arm.calib.transform().block<3,3>(0,0);
          // tracker delta in SteamVR frame
          const Eigen::Matrix3d dR_tracker =
              T_tracker.block<3,3>(0,0)
              * arm.T_tracker_startup.block<3,3>(0,0).transpose();
          const Eigen::Vector3d dp_tracker =
              T_tracker.block<3,1>(0,3)
              - arm.T_tracker_startup.block<3,1>(0,3);
          // express delta in UR base frame
          const Eigen::Matrix3d dR_ur     = R_cali * dR_tracker * R_cali.transpose();
          const Eigen::Vector3d dp_ur     = R_cali * dp_tracker;
          // compose with home pose
          T_ee.block<3,3>(0,0) = dR_ur * T_ur_home.block<3,3>(0,0);
          T_ee.block<3,1>(0,3) = T_ur_home.block<3,1>(0,3) + dp_ur;
        }

        std::array<double, 6> q_sol;
        if (ur_ik_solve(T_ee, arm.q, opts_.robot_type, q_sol)) {
          q_target = q_sol;
        } else {
          q_target = q_sol;  // best-effort
          static int warn_div = 0;
          if ((warn_div++ % 500) == 0) {
            RCLCPP_WARN(get_logger(),
                "%s: IK did not converge",
                arm.opts.topic_prefix.c_str());
          }
        }
      } else {
        q_target = arm.q;  // tracker lost — hold
        static int lost_div = 0;
        if ((lost_div++ % 500) == 0) {
          RCLCPP_WARN(get_logger(),
              "%s: tracker poll returned false (tare_pending=%s) — "
              "check SteamVR shows the tracker as tracking (green) and "
              "that base stations are visible to it",
              arm.opts.topic_prefix.c_str(),
              arm.tare_pending ? "yes" : "no");
        }
      }
      break;
    }
    case MODE_HOMING: {
      const double tau = (m_duration > 0.0)
          ? (t_now - m_t_start) / m_duration : 1.0;
      const double s = quintic(tau);
      for (int i = 0; i < 6; ++i) {
        q_target[i] =
            arm.q_home_start[i] + s * (arm.home_qpos[i] - arm.q_home_start[i]);
      }
      break;
    }
    case MODE_PAUSED:
    default:
      q_target = arm.q;  // freeze
      break;
  }

  // LPF on q (smooths IK output jitter / tracker noise before the
  // follower's PD sees it). dq is computed from the FILTERED q so
  // it's automatically consistent — no separate raw vs filtered
  // estimates fighting each other.
  const double dt   = 1.0 / opts_.control_rate_hz;
  const double aq   = opts_.q_filter_alpha;
  const double adq  = opts_.dq_filter_alpha;
  std::array<double, 6> q_new{};
  for (int i = 0; i < 6; ++i) {
    q_new[i] = aq * q_target[i] + (1.0 - aq) * arm.q[i];
  }
  for (int i = 0; i < 6; ++i) {
    const double dq_raw = (q_new[i] - arm.q[i]) / dt;
    arm.dq[i] = adq * dq_raw + (1.0 - adq) * arm.dq[i];
  }
  arm.q_prev = arm.q;
  arm.q = q_new;
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

void ViveLeaderNode::publish_arm_state(Arm& arm) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.position.assign(arm.q.begin(), arm.q.end());
  msg.velocity.assign(arm.dq.begin(), arm.dq.end());
  arm.state_pub->publish(msg);
}

void ViveLeaderNode::publish_mode(int mode, double t_start, double duration) {
  {
    std::lock_guard<std::mutex> lk(mode_mtx_);
    mode_state_    = mode;
    mode_t_start_  = t_start;
    mode_duration_ = duration;
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {static_cast<double>(mode), t_start, duration};
  mode_pub_->publish(msg);
}

}  // namespace ur10e_teleop_unilateral_vive_cpp
