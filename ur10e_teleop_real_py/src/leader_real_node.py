#!/usr/bin/env python3
"""
leader_real_node.py — ROS2 node for the Leader arm (real hardware / dummy).

Control parameters loaded from YAML config:
  --config dummy.yaml    → self-computed friction/gravity (for testing)
  --config real_ur.yaml  → UR firmware handles friction/gravity

Topics:
  /ur10e/leader/joint_state    (sensor_msgs/JointState)    — published
  /ur10e/follower/joint_state  (sensor_msgs/JointState)    — subscribed (peer)
  /ur10e/mode                  (std_msgs/Float64MultiArray) — pub/sub
  /ur10e/reset                 (std_msgs/Int32)             — subscribed
"""

import argparse
import os
import sys
import threading
import time
import traceback

import numpy as np
import yaml

try:
    from pynput import keyboard as pk
    PYNPUT_AVAILABLE = True
except Exception:
    pk = None
    PYNPUT_AVAILABLE = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

sys.path.insert(0, os.path.dirname(__file__))
# Also add share/src path for when running from lib/ directory
try:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(
        get_package_share_directory('ur10e_teleop_real_py'), 'src'))
except Exception:
    pass
from dummy_control import DummyControl
from environment_sensing_data_emulator import EnvironmentEmulator

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MODE_ACTIVE = 0
MODE_PAUSED = 1
MODE_HOMING = 2
MODE_FREEDRIVE = 3   # tau=0 (gravity-comp only) — for manual positioning / calibration

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
N = 6
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
HOMING_DURATION = 3.0
STEP_Q = 0.01
REPEAT_INTERVAL = 0.05


def _state_name(s: int) -> str:
    return {MODE_ACTIVE: 'ACTIVE', MODE_PAUSED: 'PAUSED',
            MODE_HOMING: 'HOMING', MODE_FREEDRIVE: 'FREEDRIVE'}.get(s, '??')


def _quintic(alpha: float) -> float:
    a = np.clip(alpha, 0.0, 1.0)
    return 10 * a**3 - 15 * a**4 + 6 * a**5


# ---------------------------------------------------------------------------
# Keyboard helpers (optional — disabled in SSH/headless environments)
# ---------------------------------------------------------------------------
if PYNPUT_AVAILABLE:
    def _normalize_key(key):
        if isinstance(key, pk.Key):
            return key
        if isinstance(key, pk.KeyCode):
            ch = getattr(key, 'char', None)
            if ch is not None:
                return ch.lower() if ch.isalpha() else ch
        return None

    class KeyboardHandler:
        def __init__(self):
            self._held_keys = set()
            self._lock = threading.Lock()
            self._last_repeat_t = 0.0
            self._listener = pk.Listener(
                on_press=self._on_press, on_release=self._on_release)
            self._listener.daemon = True
            self._listener.start()

        def _on_press(self, key):
            k = _normalize_key(key)
            if k is not None:
                with self._lock:
                    self._held_keys.add(k)

        def _on_release(self, key):
            k = _normalize_key(key)
            if k is not None:
                with self._lock:
                    self._held_keys.discard(k)

        def apply_held_keys(self, q_user: np.ndarray, now: float) -> bool:
            if now - self._last_repeat_t < REPEAT_INTERVAL:
                return False
            self._last_repeat_t = now
            with self._lock:
                keys_now = set(self._held_keys)
            if not keys_now:
                return False
            moved = False
            if pk.Key.up    in keys_now: q_user[0] -= STEP_Q; moved = True
            if pk.Key.down  in keys_now: q_user[0] += STEP_Q; moved = True
            if pk.Key.left  in keys_now: q_user[1] -= STEP_Q; moved = True
            if pk.Key.right in keys_now: q_user[1] += STEP_Q; moved = True
            if '[' in keys_now: q_user[2] -= STEP_Q; moved = True
            if ']' in keys_now: q_user[2] += STEP_Q; moved = True
            if 'r' in keys_now: q_user[3] -= STEP_Q; moved = True
            if 'f' in keys_now: q_user[3] += STEP_Q; moved = True
            if 't' in keys_now: q_user[4] -= STEP_Q; moved = True
            if 'g' in keys_now: q_user[4] += STEP_Q; moved = True
            if 'y' in keys_now: q_user[5] -= STEP_Q; moved = True
            if 'h' in keys_now: q_user[5] += STEP_Q; moved = True
            return moved


# ---------------------------------------------------------------------------
# LeaderReal — ROS2 Node
# ---------------------------------------------------------------------------
class LeaderReal(Node):

    def __init__(self, robot: str = 'ur10e', config_path: str = None,
                 client: str = 'dummy', robot_ip: str = '127.0.0.1',
                 robot_port: int = 30004):
        super().__init__('leader_real_node')
        self._load_config(config_path)
        self._init_robot(robot, client, robot_ip, robot_port)
        self._init_pubsub()
        self._init_threads()

    def _load_config(self, config_path: str):
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                self.cfg = yaml.safe_load(f)
            self.get_logger().info(f'Config loaded: {config_path}')
        else:
            self.get_logger().warn('No config file — using hardcoded defaults')
            self.cfg = {
                'friction_comp': False,
                'gravity_comp_internal': False,
                'torque_limit': [330, 330, 150, 56, 56, 56],
                'leader': {
                    'KP_USER': [40, 40, 30, 15, 15, 8],
                    'KD_USER': [6, 6, 4, 2, 2, 1],
                    'KP_HOLD': [200, 200, 150, 60, 60, 30],
                    'KD_HOLD': [20, 20, 12, 6, 6, 3],
                    'KP_BI': [400, 400, 300, 100, 100, 50],
                    'KD_BI': [25, 25, 18, 8, 8, 4],
                    'OVERFORCE_USER': [20, 20, 15, 8, 8, 4],
                    'OVERFORCE_CONSTRAINT': [80, 80, 50, 25, 25, 15],
                },
                'friction': {'Fc': [0]*6, 'Fv': [0]*6, 'k': [50]*6},
                'timestep': 0.002,
            }

        lcfg = self.cfg['leader']
        self.KP_USER = np.array(lcfg['KP_USER'])
        self.KD_USER = np.array(lcfg['KD_USER'])
        self.KP_HOLD = np.array(lcfg['KP_HOLD'])
        self.KD_HOLD = np.array(lcfg['KD_HOLD'])
        self.KP_BI   = np.array(lcfg['KP_BI'])
        self.KD_BI   = np.array(lcfg['KD_BI'])
        self.TAU_BI_DEADBAND = np.array(lcfg.get('TAU_BI_DEADBAND', [0.0]*N))
        self.OVERFORCE_USER       = np.array(lcfg['OVERFORCE_USER'])
        self.OVERFORCE_CONSTRAINT = np.array(lcfg['OVERFORCE_CONSTRAINT'])

        # Leader-specific torque limit if present, else fall back to shared
        tlim = self.cfg.get('leader_torque_limit',
                             self.cfg.get('torque_limit', [25.0]*N))
        self.act_lo = -np.array(tlim)
        self.act_hi =  np.array(tlim)

        self.friction_comp = self.cfg['friction_comp']
        self.gravity_comp_internal = self.cfg['gravity_comp_internal']

        frc = self.cfg['friction']
        self.Fc = np.array(frc['Fc'])
        self.Fv = np.array(frc['Fv'])
        self.Fk = np.array(frc['k'])

        self.timestep = self.cfg.get('timestep', 0.002)

        # Per-robot HOME_QPOS (from config or global default)
        self.HOME_QPOS = np.array(
            self.cfg.get('leader_home', HOME_QPOS.tolist()))

        # Peer HOME for bilateral mirroring
        self.PEER_HOME = np.array(
            self.cfg.get('follower_home', HOME_QPOS.tolist()))

        # Joint mirror sign (for reversed robot installation)
        mirror_cfg = self.cfg.get('joint_mirror', {})
        self.MIRROR_SIGN = np.array(mirror_cfg.get('sign', [1]*N))

    def _mirror_peer_to_local(self, peer_q: np.ndarray) -> np.ndarray:
        """Convert peer joint angles to local coordinate frame."""
        delta = peer_q - self.PEER_HOME
        return self.HOME_QPOS + self.MIRROR_SIGN * delta

    def _init_robot(self, robot: str, client: str, robot_ip: str,
                    robot_port: int = 30004):
        if client == 'rtde':
            from control import URControl
            self.robot = URControl(
                robot_ip=robot_ip, robot_name=robot,
                timestep=self.timestep, port=robot_port)
        else:
            self.emulator = EnvironmentEmulator(robot)
            self.robot = DummyControl(
                robot_name=robot, timestep=self.timestep,
                emulator=self.emulator)
        self.robot.connect()

    def _init_pubsub(self):
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.state_pub = self.create_publisher(
            JointState, '/ur10e/leader/joint_state', 10)
        self.create_subscription(
            JointState, '/ur10e/follower/joint_state', self._peer_cb, 10)
        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)
        self.create_subscription(
            Float64MultiArray, '/ur10e/mode', self._mode_cb, qos_latched)
        self.create_subscription(
            Int32, '/ur10e/reset', self._reset_cb, qos_latched)

        self._state_msg = JointState()
        self._state_msg.name = list(JOINT_NAMES)
        self._mode_msg = Float64MultiArray()

        q, _ = self.robot.read_joint_state()
        self._publish_state(q, np.zeros(N), np.zeros(N))
        self._publish_mode(MODE_ACTIVE)

    def _init_threads(self):
        self._pub_tick = 0
        self._PUB_EVERY = 2

        self._peer_q = None
        self._peer_dq = None
        self._peer_lock = threading.Lock()
        self._mode_state = MODE_ACTIVE
        self._mode_t_start = 0.0
        self._mode_duration = 0.0
        self._mode_lock = threading.Lock()
        self._reset_counter = 0

        if PYNPUT_AVAILABLE:
            self.keyboard = KeyboardHandler()
        else:
            self.keyboard = None
            self.get_logger().warn(
                'pynput unavailable (no display) — keyboard disabled')

        self.stop_event = threading.Event()
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='leader-control')

    # ---- Friction compensation (self-computed) -----------------------------

    def _compute_friction_comp(self, dq: np.ndarray) -> np.ndarray:
        return self.Fc * np.tanh(self.Fk * dq) + self.Fv * dq

    # ---- ROS2 callbacks ----------------------------------------------------

    def _peer_cb(self, msg: JointState):
        with self._peer_lock:
            self._peer_q = np.array(msg.position[:N])
            self._peer_dq = np.array(msg.velocity[:N]) if len(msg.velocity) >= N else np.zeros(N)

    def _mode_cb(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 3:
            with self._mode_lock:
                self._mode_state = int(d[0])
                self._mode_t_start = d[1]
                self._mode_duration = d[2]

    def _reset_cb(self, msg: Int32):
        self._reset_counter = msg.data

    def _publish_state(self, q, dq, tau_contact):
        msg = self._state_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = q.tolist()
        msg.velocity = dq.tolist()
        msg.effort = tau_contact.tolist()
        self.state_pub.publish(msg)

    def _publish_mode(self, state, t_start=0.0, duration=0.0):
        msg = self._mode_msg
        msg.data = [float(state), float(t_start), float(duration)]
        self.mode_pub.publish(msg)

    # ---- Control thread ----------------------------------------------------

    def _control_loop(self):
        prev_state = MODE_ACTIVE
        q_hold = self.HOME_QPOS.copy()
        q_home_start = self.HOME_QPOS.copy()
        # Initialize q_user to actual robot position (avoids over-force at startup)
        q_init, _ = self.robot.read_joint_state()
        q_user = q_init.copy()
        self.get_logger().info(
            f'q_user initialized to actual: {q_init.round(4).tolist()}')
        overforce_cooldown = 0.0
        last_reset_counter = self._reset_counter
        # Ignore reset topic during startup grace period (latched replay protection)
        startup_time = time.time()
        RESET_STARTUP_GRACE = 2.0
        # Bilateral soft-start timer (reset on each PAUSED→ACTIVE transition)
        active_t_start = startup_time
        # 1Hz diagnostic log
        last_diag_t = 0.0
        q_prev_diag = q_init.copy()

        # Auto-homing on start
        if self.cfg.get('auto_home_on_start', False):
            homing_dur = self.cfg.get('homing_duration', 5.0)
            self.get_logger().info(
                f'Auto-homing to HOME in {homing_dur}s...')
            self._publish_mode(MODE_HOMING, time.time(), homing_dur)

        try:
            while not self.stop_event.is_set():
                step_start = time.time()
                now = step_start

                q, dq = self.robot.read_joint_state()
                tau_contact = self.robot.read_contact_forces()

                if self.gravity_comp_internal:
                    tau_grav = np.zeros(N)
                else:
                    tau_grav = self.robot.read_gravity_compensation()

                with self._mode_lock:
                    cur_state = self._mode_state
                    h_t_start = self._mode_t_start
                    h_duration = self._mode_duration

                # Reset requests → trigger smooth HOMING (not instant q_user jump)
                cur_reset = self._reset_counter
                if cur_reset != last_reset_counter:
                    last_reset_counter = cur_reset
                    if now - startup_time < RESET_STARTUP_GRACE:
                        # Ignore latched replay during startup grace period
                        self.get_logger().info(
                            f'reset ignored during startup grace '
                            f'(counter={cur_reset})')
                    elif cur_state == MODE_ACTIVE:
                        homing_dur = self.cfg.get('homing_duration', 5.0)
                        self._publish_mode(MODE_HOMING, now, homing_dur)
                        self.get_logger().info(
                            f'reset request → HOMING ({homing_dur}s)')

                # State transitions
                if cur_state != prev_state:
                    self.get_logger().info(
                        f'state  {_state_name(prev_state)} → '
                        f'{_state_name(cur_state)}')
                    if cur_state == MODE_PAUSED:
                        q_hold = self.HOME_QPOS.copy() if prev_state == MODE_HOMING else q.copy()
                    elif cur_state == MODE_HOMING:
                        q_home_start = q.copy()
                    elif cur_state == MODE_ACTIVE:
                        q_user = q.copy()
                        active_t_start = now   # for bilateral soft-start

                # Keyboard (joint-space, disabled in SSH/headless)
                if cur_state == MODE_ACTIVE and self.keyboard is not None:
                    self.keyboard.apply_held_keys(q_user, now)

                # Read peer (follower) state + mirror to local frame
                with self._peer_lock:
                    raw_peer_q = self._peer_q
                    raw_peer_dq = self._peer_dq
                bilateral_active = raw_peer_q is not None
                if bilateral_active:
                    peer_q = self._mirror_peer_to_local(raw_peer_q)
                    peer_dq = self.MIRROR_SIGN * raw_peer_dq

                # Control law
                if cur_state == MODE_ACTIVE:
                    tau_user = (self.KP_USER * (q_user - q)
                                - self.KD_USER * dq)
                    if bilateral_active:
                        # Soft-start ramp: tau_bi grows 0 → 1 over 0.5 s to
                        # avoid sudden yank if leader/follower aren't exactly
                        # synchronized at the moment ACTIVE is entered.
                        ACTIVE_RAMP = 0.5
                        ramp = min(1.0, (now - active_t_start) / ACTIVE_RAMP)
                        tau_bi_raw = (self.KP_BI * (peer_q - q)
                                      + self.KD_BI * (peer_dq - dq))
                        # Continuous deadband: zero below threshold, smooth
                        # onset above (no discontinuity).
                        mag = np.abs(tau_bi_raw)
                        excess = np.maximum(0.0, mag - self.TAU_BI_DEADBAND)
                        tau_bi = ramp * np.sign(tau_bi_raw) * excess
                    else:
                        tau_bi = np.zeros(N)
                    tau = tau_user + tau_bi + tau_grav

                elif cur_state == MODE_PAUSED:
                    tau_user = np.zeros(N)
                    tau = (self.KP_HOLD * (q_hold - q)
                           - self.KD_HOLD * dq + tau_grav)

                elif cur_state == MODE_HOMING:
                    tau_user = np.zeros(N)
                    dur = h_duration if h_duration > 0 else HOMING_DURATION
                    t = max(0.0, now - h_t_start)
                    alpha = min(t / dur, 1.0)
                    ease = _quintic(alpha)
                    q_des = (1.0 - ease) * q_home_start + ease * self.HOME_QPOS
                    tau = (self.KP_HOLD * (q_des - q)
                           - self.KD_HOLD * dq + tau_grav)
                    # Collision detection during homing (soft threshold)
                    homing_thresh_frac = self.cfg.get(
                        'homing_contact_threshold', 0.3)
                    abs_c = np.abs(tau_contact)
                    if np.any(abs_c > self.OVERFORCE_CONSTRAINT * homing_thresh_frac):
                        i = int(np.argmax(abs_c))
                        self.get_logger().warn(
                            f'HOMING stopped — contact on joint[{i}] = '
                            f'{tau_contact[i]:+.1f} Nm')
                        self._publish_mode(MODE_PAUSED)
                    elif alpha >= 1.0:
                        q_user = self.HOME_QPOS.copy()
                        self._publish_mode(MODE_PAUSED)
                        self.get_logger().info('HOMING complete → PAUSED')

                elif cur_state == MODE_FREEDRIVE:
                    # Zero-torque (firmware gravity comp only). User can move
                    # the arm freely by hand — used for calibration/positioning.
                    tau_user = np.zeros(N)
                    tau = tau_grav

                else:
                    tau_user = np.zeros(N)
                    tau = tau_grav

                # Friction compensation (self-computed when firmware doesn't)
                if not self.friction_comp:
                    tau += self._compute_friction_comp(dq)

                tau = np.clip(tau, self.act_lo, self.act_hi)
                self.robot.write_torque(tau)

                # Over-force detection
                if cur_state == MODE_ACTIVE and now > overforce_cooldown:
                    tau_user_spring = self.KP_USER * (q_user - q)
                    over_user = np.abs(tau_user_spring) > self.OVERFORCE_USER
                    over_const = np.abs(tau_contact) > self.OVERFORCE_CONSTRAINT
                    if np.any(over_user) or np.any(over_const):
                        if np.any(over_user):
                            i = int(np.argmax(np.abs(tau_user_spring)))
                            self.get_logger().warn(
                                f'OVER-FORCE  user on joint[{i}] = '
                                f'{tau_user_spring[i]:+.1f} Nm  →  PAUSED')
                        self._publish_mode(MODE_PAUSED)
                        overforce_cooldown = now + 1.0

                # Publish state (throttled)
                self._pub_tick += 1
                if self._pub_tick >= self._PUB_EVERY:
                    self._pub_tick = 0
                    self._publish_state(q, dq, tau_contact)

                # 1Hz diagnostic log
                if now - last_diag_t >= 1.0:
                    last_diag_t = now
                    dq_sample = q - q_prev_diag
                    q_prev_diag = q.copy()
                    # Gather URControl stats if available
                    status_str = ''
                    if hasattr(self.robot, 'read_status'):
                        st = self.robot.read_status()
                        status_str = (
                            f' robot_mode={st["robot_mode"]}'
                            f' safety={st["safety_mode"]}'
                            f' recv={st["recv_count"]}'
                            f' writes={st["write_count"]}')
                    tau_max = float(np.abs(tau).max())
                    self.get_logger().info(
                        f'[DIAG] mode={_state_name(cur_state)} '
                        f'|tau|max={tau_max:.2f}Nm '
                        f'|dq_1s|max={float(np.abs(dq_sample).max()):.4f}rad'
                        f'{status_str}')

                prev_state = cur_state

                elapsed = time.time() - step_start
                sleep_t = self.timestep - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except Exception:
            traceback.print_exc()
            self.stop_event.set()

    # ---- Main loop ---------------------------------------------------------

    def run(self):
        self._control_thread.start()
        self.get_logger().info(
            f'Running. friction_comp={self.friction_comp}, '
            f'gravity_comp_internal={self.gravity_comp_internal}')
        try:
            while not self.stop_event.is_set():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            self._control_thread.join(timeout=2.0)
            self.robot.disconnect()


def main():
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['ur10e', 'ur3e'], default='ur10e')
    parser.add_argument('--config', default=None,
                        help='Path to YAML config (e.g. config/dummy.yaml)')
    parser.add_argument('--client', choices=['dummy', 'rtde'], default='dummy',
                        help='Client type: dummy (local dynamics) or rtde (RTDE protocol)')
    parser.add_argument('--robot-ip', default='127.0.0.1',
                        help='Robot IP for RTDE client (default: 127.0.0.1)')
    parser.add_argument('--robot-port', type=int, default=30004,
                        help='Robot RTDE port (default: 30004)')
    args = parser.parse_args()

    config_path = args.config
    if config_path and not os.path.isabs(config_path):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('ur10e_teleop_real_py')
            candidate = os.path.join(pkg, 'config', config_path)
            if os.path.exists(candidate):
                config_path = candidate
        except Exception:
            pass

    node = LeaderReal(args.robot, config_path=config_path,
                      client=args.client, robot_ip=args.robot_ip,
                      robot_port=args.robot_port)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
