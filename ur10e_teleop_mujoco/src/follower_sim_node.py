#!/usr/bin/env python3
"""
follower_sim_node.py — ROS2 node version of follower_sim.py.

Identical physics and control logic, but all /dev/shm shared-memory
communication is replaced by ROS2 topic pub/sub:

  /ur10e/follower/joint_state  (sensor_msgs/JointState)    — published
  /ur10e/follower/cmd          (std_msgs/Float64MultiArray) — subscribed
  /ur10e/mode                  (std_msgs/Float64MultiArray) — pub/sub

No dependency on shm_manager.py or /dev/shm.
"""

import argparse
import os
import threading
import time
import traceback

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ---------------------------------------------------------------------------
# Constants (no shm_manager dependency)
# ---------------------------------------------------------------------------
MODE_ACTIVE = 0
MODE_PAUSED = 1
MODE_HOMING = 2

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
N = 6

HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

KP_FALLBACK = np.array([100.0, 100.0, 80.0, 40.0, 40.0, 20.0])
KD_FALLBACK = np.array([ 10.0,  10.0,  8.0,  4.0,  4.0,  2.0])

KP_HOLD = np.array([200.0, 200.0, 150.0, 60.0, 60.0, 30.0])
KD_HOLD = np.array([ 20.0,  20.0,  12.0,  6.0,  6.0,  3.0])

OVERFORCE_CONSTRAINT = np.array([80.0, 80.0, 50.0, 25.0, 25.0, 15.0])

HOMING_DURATION_FALLBACK = 3.0

RENDER_EVERY = 16


def _state_name(s: int) -> str:
    return {MODE_ACTIVE: 'ACTIVE', MODE_PAUSED: 'PAUSED',
            MODE_HOMING: 'HOMING'}.get(s, '??')


# ---------------------------------------------------------------------------
# Exchange buffer (lock-protected thread communication)
# ---------------------------------------------------------------------------
class ExchangeBuffer:
    """Lock-protected numpy arrays shared between MuJoCo and control threads."""

    def __init__(self, q0: np.ndarray):
        self.lock = threading.Lock()
        self.q = q0.copy()
        self.dq = np.zeros(N)
        self.tau_grav = np.zeros(N)
        self.tau_contact = np.zeros(N)
        self.tau = np.zeros(N)


# ---------------------------------------------------------------------------
# FollowerSim — ROS2 Node
# ---------------------------------------------------------------------------
class FollowerSim(Node):
    """MuJoCo simulation for the UR10E Follower arm (ROS2 node)."""

    def __init__(self, model_path: str):
        super().__init__('follower_sim_node')
        self._init_mujoco(model_path)
        self._init_pubsub()
        self._init_threads()

    # ---- Sub-init methods --------------------------------------------------

    def _init_mujoco(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:N] = HOME_QPOS
        mujoco.mj_forward(self.model, self.data)

        joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                     for n in JOINT_NAMES]
        self.qpos_ids = [int(self.model.jnt_qposadr[j]) for j in joint_ids]
        self.qvel_ids = [int(self.model.jnt_dofadr[j])  for j in joint_ids]

        act_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                     f'a_{n.replace("_joint","")}')
                   for n in JOINT_NAMES]
        self.act_lo = np.array([self.model.actuator_ctrlrange[a, 0]
                                for a in act_ids])
        self.act_hi = np.array([self.model.actuator_ctrlrange[a, 1]
                                for a in act_ids])

    def _init_pubsub(self):
        """Create ROS2 publishers and subscribers (pure pub/sub only)."""
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        # State publisher
        self.state_pub = self.create_publisher(
            JointState, '/ur10e/follower/joint_state', 10)

        # Subscribe to leader's joint state (bilateral coupling)
        self.create_subscription(
            JointState, '/ur10e/leader/joint_state', self._peer_cb, 10)

        # Mode pub/sub (latched for late-joining nodes)
        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)
        self.create_subscription(
            Float64MultiArray, '/ur10e/mode', self._mode_cb, qos_latched)

        # Pre-allocate reusable message objects (avoid per-iteration allocation)
        self._state_msg = JointState()
        self._state_msg.name = list(JOINT_NAMES)
        self._mode_msg = Float64MultiArray()

        # Publish initial state
        self._publish_state(
            self.data.qpos[self.qpos_ids].copy(),
            np.zeros(N), np.zeros(N))

    def _init_threads(self):
        """Create cached subscription data, exchange buffer, control thread."""
        # Publish throttle: publish state every N control iterations
        self._pub_tick = 0
        self._PUB_EVERY = 2   # 500 Hz control / 2 = 250 Hz publish

        # Cached peer (leader) joint state for bilateral coupling
        self._peer_q = None
        self._peer_dq = None
        self._peer_lock = threading.Lock()
        self._mode_state = MODE_ACTIVE
        self._mode_t_start = 0.0
        self._mode_duration = 0.0
        self._mode_lock = threading.Lock()

        # Exchange buffer for MuJoCo ↔ control thread
        q0 = self.data.qpos[self.qpos_ids].copy()
        self.buf = ExchangeBuffer(q0)

        mujoco.mj_forward(self.model, self.data)
        with self.buf.lock:
            self.buf.tau_grav[:] = self.data.qfrc_bias[self.qvel_ids]

        self.stop_event = threading.Event()
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name='follower-control',
        )

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

    # ---- ROS2 publish helpers ----------------------------------------------

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
        q_hold = HOME_QPOS.copy()
        q_home_start = HOME_QPOS.copy()
        q_target_init = HOME_QPOS.copy()
        overforce_cooldown = 0.0

        try:
            while not self.stop_event.is_set():
                step_start = time.time()
                now = step_start

                # ---- Read state from MuJoCo thread ----
                with self.buf.lock:
                    q = self.buf.q.copy()
                    dq = self.buf.dq.copy()
                    tau_grav = self.buf.tau_grav.copy()
                    tau_contact = self.buf.tau_contact.copy()

                # ---- Read mode from ROS2 topic ----
                with self._mode_lock:
                    cur_state = self._mode_state
                    h_t_start = self._mode_t_start
                    h_duration = self._mode_duration

                if cur_state != prev_state:
                    self.get_logger().info(
                        f'state  {_state_name(prev_state)} → '
                        f'{_state_name(cur_state)}')
                    if cur_state == MODE_PAUSED:
                        if prev_state == MODE_HOMING:
                            q_hold = HOME_QPOS.copy()
                        else:
                            q_hold = q.copy()
                    elif cur_state == MODE_HOMING:
                        q_home_start = q.copy()

                # ---- State-specific control law ----
                if cur_state == MODE_ACTIVE:
                    with self._peer_lock:
                        peer_q = self._peer_q
                        peer_dq = self._peer_dq

                    if peer_q is not None:
                        # Bilateral PD: track leader joint state
                        tau = (KP_FALLBACK * (peer_q - q)
                               + KD_FALLBACK * (peer_dq - dq)
                               + tau_grav)
                    else:
                        # No leader data yet — hold home
                        tau = (KP_FALLBACK * (q_target_init - q)
                               + KD_FALLBACK * (np.zeros(N) - dq)
                               + tau_grav)

                elif cur_state == MODE_PAUSED:
                    tau = KP_HOLD * (q_hold - q) - KD_HOLD * dq + tau_grav

                elif cur_state == MODE_HOMING:
                    dur = h_duration if h_duration > 0 else HOMING_DURATION_FALLBACK
                    t = max(0.0, now - h_t_start)
                    alpha = min(t / dur, 1.0)
                    ease = 3.0 * alpha ** 2 - 2.0 * alpha ** 3
                    q_des = (1.0 - ease) * q_home_start + ease * HOME_QPOS
                    tau = KP_HOLD * (q_des - q) - KD_HOLD * dq + tau_grav

                else:
                    tau = tau_grav

                tau = np.clip(tau, self.act_lo, self.act_hi)

                with self.buf.lock:
                    self.buf.tau[:] = tau

                # ---- Over-force detection (ACTIVE only) ----
                if cur_state == MODE_ACTIVE and now > overforce_cooldown:
                    abs_c = np.abs(tau_contact)
                    if np.any(abs_c > OVERFORCE_CONSTRAINT):
                        i = int(np.argmax(abs_c))
                        self.get_logger().warn(
                            f'OVER-FORCE  contact on joint[{i}] = '
                            f'{tau_contact[i]:+.1f} Nm  →  PAUSED')
                        self._publish_mode(MODE_PAUSED)
                        overforce_cooldown = now + 1.0

                # ---- Publish state to ROS2 (throttled) ----
                self._pub_tick += 1
                if self._pub_tick >= self._PUB_EVERY:
                    self._pub_tick = 0
                    self._publish_state(q, dq, tau_contact)

                prev_state = cur_state

                elapsed = time.time() - step_start
                sleep_t = self.model.opt.timestep - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except Exception:
            traceback.print_exc()
            self.stop_event.set()

    # ---- MuJoCo thread (physics + render) ----------------------------------

    def run(self):
        self._control_thread.start()

        render_tick = 0

        with mujoco.viewer.launch_passive(self.model, self.data,
                                           show_left_ui=False,
                                           show_right_ui=False) as viewer:
            viewer.cam.azimuth = 225.0
            viewer.cam.elevation = -20.0
            viewer.cam.distance = 2.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.4]

            self.get_logger().info(
                'Running.  Waiting for bilateral_control commands...')

            try:
                while viewer.is_running() and not self.stop_event.is_set():
                    step_start = time.time()

                    with self.buf.lock:
                        self.data.ctrl[:N] = self.buf.tau

                    mujoco.mj_step(self.model, self.data)

                    with self.buf.lock:
                        self.buf.q[:] = self.data.qpos[self.qpos_ids]
                        self.buf.dq[:] = self.data.qvel[self.qvel_ids]
                        self.buf.tau_grav[:] = \
                            self.data.qfrc_bias[self.qvel_ids]
                        self.buf.tau_contact[:] = \
                            self.data.qfrc_constraint[self.qvel_ids]

                    render_tick += 1
                    if render_tick >= RENDER_EVERY:
                        render_tick = 0
                        viewer.sync()

                    elapsed = time.time() - step_start
                    sleep_t = self.model.opt.timestep - elapsed
                    if sleep_t > 0:
                        time.sleep(sleep_t)
            finally:
                self.stop_event.set()
                self._control_thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    rclpy.init()

    from ament_index_python.packages import get_package_share_directory
    share_dir = get_package_share_directory('ur10e_teleop_mujoco')

    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.path.join(
        share_dir, 'xml', 'ur10e_follower_scene.xml'))
    args = parser.parse_args()

    node = FollowerSim(args.model)

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
