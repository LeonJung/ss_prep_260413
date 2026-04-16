#!/usr/bin/env python3
"""
follower_real_node.py — ROS2 node for the Follower arm (real hardware / dummy).

Control parameters loaded from YAML config:
  --config dummy.yaml    → self-computed friction/gravity (for testing)
  --config real_ur.yaml  → UR firmware handles friction/gravity

Topics:
  /ur10e/follower/joint_state  (sensor_msgs/JointState)    — published
  /ur10e/leader/joint_state    (sensor_msgs/JointState)    — subscribed (peer)
  /ur10e/mode                  (std_msgs/Float64MultiArray) — pub/sub
"""

import argparse
import os
import sys
import threading
import time
import traceback

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(__file__))
try:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(
        get_package_share_directory('ur10e_teleop_real'), 'src'))
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
HOMING_DURATION_FALLBACK = 3.0


def _state_name(s: int) -> str:
    return {MODE_ACTIVE: 'ACTIVE', MODE_PAUSED: 'PAUSED',
            MODE_HOMING: 'HOMING', MODE_FREEDRIVE: 'FREEDRIVE'}.get(s, '??')


def _quintic(alpha: float) -> float:
    """Quintic spline: zero velocity and acceleration at endpoints."""
    a = np.clip(alpha, 0.0, 1.0)
    return 10 * a**3 - 15 * a**4 + 6 * a**5


# ---------------------------------------------------------------------------
# FollowerReal — ROS2 Node
# ---------------------------------------------------------------------------
class FollowerReal(Node):

    def __init__(self, robot_name: str = 'ur10e', config_path: str = None,
                 client: str = 'dummy', robot_ip: str = '127.0.0.1',
                 robot_port: int = 30004):
        super().__init__('follower_real_node')
        self._load_config(config_path)
        self._init_robot(robot_name, client, robot_ip, robot_port)
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
                'follower': {
                    'KP_TRACK': [100, 100, 80, 40, 40, 20],
                    'KD_TRACK': [10, 10, 8, 4, 4, 2],
                    'KP_HOLD': [200, 200, 150, 60, 60, 30],
                    'KD_HOLD': [20, 20, 12, 6, 6, 3],
                    'OVERFORCE_CONSTRAINT': [80, 80, 50, 25, 25, 15],
                },
                'friction': {'Fc': [0]*6, 'Fv': [0]*6, 'k': [50]*6},
                'timestep': 0.002,
            }

        fcfg = self.cfg['follower']
        self.KP_TRACK = np.array(fcfg['KP_TRACK'])
        self.KD_TRACK = np.array(fcfg['KD_TRACK'])
        self.KP_HOLD  = np.array(fcfg['KP_HOLD'])
        self.KD_HOLD  = np.array(fcfg['KD_HOLD'])
        self.OVERFORCE_CONSTRAINT = np.array(fcfg['OVERFORCE_CONSTRAINT'])

        # Follower-specific torque limit if present, else fall back to shared
        tlim = self.cfg.get('follower_torque_limit',
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

        # Per-robot HOME_QPOS
        self.HOME_QPOS = np.array(
            self.cfg.get('follower_home', HOME_QPOS.tolist()))
        # Peer HOME for bilateral mirroring
        self.PEER_HOME = np.array(
            self.cfg.get('leader_home', HOME_QPOS.tolist()))
        # Joint mirror sign
        mirror_cfg = self.cfg.get('joint_mirror', {})
        self.MIRROR_SIGN = np.array(mirror_cfg.get('sign', [1]*N))

    def _mirror_peer_to_local(self, peer_q: np.ndarray) -> np.ndarray:
        """Convert peer (leader) joint angles to follower coordinate frame."""
        delta = peer_q - self.PEER_HOME
        return self.HOME_QPOS + self.MIRROR_SIGN * delta

    def _init_robot(self, robot_name: str, client: str, robot_ip: str,
                    robot_port: int = 30004):
        if client == 'rtde':
            from control import URControl
            self.robot = URControl(
                robot_ip=robot_ip, robot_name=robot_name,
                timestep=self.timestep, port=robot_port)
        else:
            self.emulator = EnvironmentEmulator(robot_name)
            self.robot = DummyControl(
                robot_name=robot_name, timestep=self.timestep,
                emulator=self.emulator)
        self.robot.connect()

    def _init_pubsub(self):
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.state_pub = self.create_publisher(
            JointState, '/ur10e/follower/joint_state', 10)
        self.create_subscription(
            JointState, '/ur10e/leader/joint_state', self._peer_cb, 10)
        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)
        self.create_subscription(
            Float64MultiArray, '/ur10e/mode', self._mode_cb, qos_latched)

        self._state_msg = JointState()
        self._state_msg.name = list(JOINT_NAMES)
        self._mode_msg = Float64MultiArray()

        q, _ = self.robot.read_joint_state()
        self._publish_state(q, np.zeros(N), np.zeros(N))

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

        self.stop_event = threading.Event()
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='follower-control')

    # ---- Friction compensation (self-computed) -----------------------------

    def _compute_friction_comp(self, dq: np.ndarray) -> np.ndarray:
        """tanh friction model: tau = Fc * tanh(k * dq) + Fv * dq"""
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
        # Initialize to actual robot position (avoids over-force at startup)
        q_init, _ = self.robot.read_joint_state()
        q_target_init = q_init.copy()
        self.get_logger().info(
            f'q_target initialized to actual: {q_init.round(4).tolist()}')
        overforce_cooldown = 0.0

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

                if cur_state != prev_state:
                    self.get_logger().info(
                        f'state  {_state_name(prev_state)} → '
                        f'{_state_name(cur_state)}')
                    if cur_state == MODE_PAUSED:
                        q_hold = self.HOME_QPOS.copy() if prev_state == MODE_HOMING else q.copy()
                    elif cur_state == MODE_HOMING:
                        q_home_start = q.copy()

                # ---- Control law ----
                if cur_state == MODE_ACTIVE:
                    # Mirror peer state to local frame
                    with self._peer_lock:
                        raw_peer_q = self._peer_q
                        raw_peer_dq = self._peer_dq

                    if raw_peer_q is not None:
                        peer_q = self._mirror_peer_to_local(raw_peer_q)
                        peer_dq = self.MIRROR_SIGN * raw_peer_dq
                        tau = (self.KP_TRACK * (peer_q - q)
                               + self.KD_TRACK * (peer_dq - dq)
                               + tau_grav)
                    else:
                        tau = (self.KP_TRACK * (q_target_init - q)
                               - self.KD_TRACK * dq + tau_grav)

                elif cur_state == MODE_PAUSED:
                    tau = self.KP_HOLD * (q_hold - q) - self.KD_HOLD * dq + tau_grav

                elif cur_state == MODE_HOMING:
                    dur = h_duration if h_duration > 0 else HOMING_DURATION_FALLBACK
                    t = max(0.0, now - h_t_start)
                    alpha = min(t / dur, 1.0)
                    ease = _quintic(alpha)
                    q_des = (1.0 - ease) * q_home_start + ease * self.HOME_QPOS
                    tau = self.KP_HOLD * (q_des - q) - self.KD_HOLD * dq + tau_grav
                    # Collision detection during homing
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
                        q_target_init = self.HOME_QPOS.copy()
                        self._publish_mode(MODE_PAUSED)
                        self.get_logger().info('HOMING complete → PAUSED')

                elif cur_state == MODE_FREEDRIVE:
                    # Zero-torque (firmware gravity comp only). User can move
                    # the arm freely by hand — used for calibration/positioning.
                    tau = tau_grav

                else:
                    tau = tau_grav

                # Friction compensation (self-computed when firmware doesn't do it)
                if not self.friction_comp:
                    tau += self._compute_friction_comp(dq)

                tau = np.clip(tau, self.act_lo, self.act_hi)
                self.robot.write_torque(tau)

                # Over-force detection
                if cur_state == MODE_ACTIVE and now > overforce_cooldown:
                    abs_c = np.abs(tau_contact)
                    if np.any(abs_c > self.OVERFORCE_CONSTRAINT):
                        i = int(np.argmax(abs_c))
                        self.get_logger().warn(
                            f'OVER-FORCE  contact on joint[{i}] = '
                            f'{tau_contact[i]:+.1f} Nm  →  PAUSED')
                        self._publish_mode(MODE_PAUSED)
                        overforce_cooldown = now + 1.0

                # Publish state (throttled)
                self._pub_tick += 1
                if self._pub_tick >= self._PUB_EVERY:
                    self._pub_tick = 0
                    self._publish_state(q, dq, tau_contact)

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
            pkg = get_package_share_directory('ur10e_teleop_real')
            candidate = os.path.join(pkg, 'config', config_path)
            if os.path.exists(candidate):
                config_path = candidate
        except Exception:
            pass

    node = FollowerReal(config_path=config_path,
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
