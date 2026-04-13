#!/usr/bin/env python3
"""
leader_sim_node.py — ROS2 node version of leader_sim.py.

Identical physics, control, keyboard, and IK logic, but all /dev/shm
shared-memory communication is replaced by ROS2 topic pub/sub:

  /ur10e/leader/joint_state  (sensor_msgs/JointState)    — published
  /ur10e/leader/cmd          (std_msgs/Float64MultiArray) — subscribed
  /ur10e/mode                (std_msgs/Float64MultiArray) — pub/sub
  /ur10e/reset               (std_msgs/Int32)             — subscribed

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

from pynput import keyboard as pk
import Xlib.display
import Xlib.X

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

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

# ---------------------------------------------------------------------------
# Per-robot configuration
# ---------------------------------------------------------------------------
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

ROBOT_CONFIG = {
    'ur10e': {
        'model_file': 'ur10e.xml',
        'KP_USER':   np.array([40.0, 40.0, 30.0, 15.0, 15.0,  8.0]),
        'KD_USER':   np.array([ 6.0,  6.0,  4.0,  2.0,  2.0,  1.0]),
        'KP_HOLD':   np.array([200.0, 200.0, 150.0, 60.0, 60.0, 30.0]),
        'KD_HOLD':   np.array([ 20.0,  20.0,  12.0,  6.0,  6.0,  3.0]),
        'KP_BI':     np.array([400.0, 400.0, 300.0, 100.0, 100.0, 50.0]),
        'KD_BI':     np.array([ 25.0,  25.0,  18.0,   8.0,   8.0,  4.0]),
        'OVERFORCE_USER':       np.array([20.0, 20.0, 15.0,  8.0,  8.0,  4.0]),
        'OVERFORCE_CONSTRAINT': np.array([80.0, 80.0, 50.0, 25.0, 25.0, 15.0]),
    },
    'ur3e': {
        'model_file': 'ur3e.xml',
        'KP_USER':   np.array([21.0, 21.0, 15.0,  7.5,  7.5,  4.5]),
        'KD_USER':   np.array([ 3.3,  3.3,  2.1,  1.05, 1.05, 0.6]),
        'KP_HOLD':   np.array([35.0, 35.0, 25.0, 10.0, 10.0,  5.0]),
        'KD_HOLD':   np.array([ 3.5,  3.5,  2.0,  1.0,  1.0,  0.5]),
        'KP_BI':     np.array([70.0, 70.0, 50.0, 15.0, 15.0,  9.0]),
        'KD_BI':     np.array([ 4.5,  4.5,  3.0,  1.2,  1.2,  0.7]),
        'OVERFORCE_USER':       np.array([10.5, 10.5, 7.5, 4.0, 4.0, 2.25]),
        'OVERFORCE_CONSTRAINT': np.array([13.0, 13.0, 8.0, 4.0, 4.0, 2.5]),
    },
}

HOMING_DURATION = 3.0
STEP_M = 0.01
STEP_R = 0.05
REPEAT_INTERVAL = 0.05
RENDER_EVERY = 16

ROBOT_BODIES = [
    'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link',
    'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'flange',
]


def _state_name(s: int) -> str:
    return {MODE_ACTIVE: 'ACTIVE', MODE_PAUSED: 'PAUSED',
            MODE_HOMING: 'HOMING'}.get(s, '??')


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------
def rpy_to_quat(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r / 2.0), np.sin(r / 2.0)
    cp, sp = np.cos(p / 2.0), np.sin(p / 2.0)
    cy, sy = np.cos(y / 2.0), np.sin(y / 2.0)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def mat_to_rpy(R):
    p = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    if np.cos(p) > 1e-6:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:
        r = 0.0
        y = np.arctan2(-R[0, 1], R[1, 1])
    return np.array([r, p, y])


# ---------------------------------------------------------------------------
# Damped-least-squares Jacobian IK
# ---------------------------------------------------------------------------
def solve_ik(model, ik_data, eef_site_id, target_pos, target_quat,
             q_seed, qpos_ids, qvel_ids,
             n_iters=100, tol=1e-4, lam=0.15, step_scale=0.5):
    ik_data.qpos[:] = 0.0
    ik_data.qvel[:] = 0.0
    ik_data.qpos[qpos_ids] = q_seed
    mujoco.mj_forward(model, ik_data)

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    neg_cur_quat = np.zeros(4)
    err_quat     = np.zeros(4)
    rot_err      = np.zeros(3)

    for _ in range(n_iters):
        cur_pos  = ik_data.site_xpos[eef_site_id].copy()
        cur_mat  = ik_data.site_xmat[eef_site_id].copy()
        cur_quat = np.zeros(4)
        mujoco.mju_mat2Quat(cur_quat, cur_mat)

        pos_err = target_pos - cur_pos

        mujoco.mju_negQuat(neg_cur_quat, cur_quat)
        mujoco.mju_mulQuat(err_quat, target_quat, neg_cur_quat)
        mujoco.mju_quat2Vel(rot_err, err_quat, 1.0)

        err = np.concatenate([pos_err, rot_err])
        if np.linalg.norm(err) < tol:
            break

        mujoco.mj_jacSite(model, ik_data, jacp, jacr, eef_site_id)
        J = np.vstack([jacp[:, qvel_ids], jacr[:, qvel_ids]])

        JJt = J @ J.T + (lam ** 2) * np.eye(6)
        delta_q = J.T @ np.linalg.solve(JJt, err)

        ik_data.qpos[qpos_ids] += step_scale * delta_q
        mujoco.mj_forward(model, ik_data)

    return np.array(ik_data.qpos[qpos_ids])


# ---------------------------------------------------------------------------
# Helper: normalise pynput key
# ---------------------------------------------------------------------------
def _normalize_key(key):
    if isinstance(key, pk.Key):
        return key
    if isinstance(key, pk.KeyCode):
        ch = getattr(key, 'char', None)
        if ch is not None:
            return ch.lower() if ch.isalpha() else ch
    return None


def _make_focus_checker(robot_name: str = 'ur10e'):
    robot_token = robot_name.lower()
    try:
        disp = Xlib.display.Display()
        root = disp.screen().root
        atom_active = disp.intern_atom('_NET_ACTIVE_WINDOW')
    except Exception:
        return lambda: True

    def check():
        try:
            prop = root.get_full_property(atom_active, Xlib.X.AnyPropertyType)
            if not prop or len(prop.value) == 0:
                return False
            wid = int(prop.value[0])
            w = disp.create_resource_object('window', wid)
            name = (w.get_wm_name() or '')
            name = name.lower() if isinstance(name, str) else ''
            return robot_token in name and 'follower' not in name
        except Exception:
            return False
    return check


# ---------------------------------------------------------------------------
# Exchange buffer
# ---------------------------------------------------------------------------
class ExchangeBuffer:
    def __init__(self, q0: np.ndarray, eef_xyz0: np.ndarray,
                 eef_mat0: np.ndarray):
        self.lock = threading.Lock()
        self.q = q0.copy()
        self.dq = np.zeros(N)
        self.tau_grav = np.zeros(N)
        self.tau_contact = np.zeros(N)
        self.drag_active = False
        self.eef_xyz = eef_xyz0.copy()
        self.eef_mat = eef_mat0.copy()
        self.tau = np.zeros(N)


# ---------------------------------------------------------------------------
# Control state (shared between key callback and control thread)
# ---------------------------------------------------------------------------
class ControlState:
    def __init__(self, xyz0, rpy0, q0):
        self.lock       = threading.Lock()
        self.target_xyz = xyz0.copy()
        self.target_rpy = rpy0.copy()
        self.q_target   = q0.copy()
        self.dirty      = True


# ---------------------------------------------------------------------------
# KeyboardHandler
# ---------------------------------------------------------------------------
class KeyboardHandler:
    def __init__(self, robot_name: str):
        self._held_keys = set()
        self._lock = threading.Lock()
        self._last_repeat_t = 0.0
        self._is_focused = _make_focus_checker(robot_name)

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

    def is_focused(self) -> bool:
        return self._is_focused()

    def apply_held_keys(self, ctrl: ControlState, now: float) -> None:
        if now - self._last_repeat_t < REPEAT_INTERVAL:
            return
        if not self._is_focused():
            return
        self._last_repeat_t = now

        with self._lock:
            keys_now = set(self._held_keys)
        if not keys_now:
            return

        moved = False
        with ctrl.lock:
            if pk.Key.up    in keys_now:
                ctrl.target_xyz[0] -= STEP_M; moved = True
            if pk.Key.down  in keys_now:
                ctrl.target_xyz[0] += STEP_M; moved = True
            if pk.Key.left  in keys_now:
                ctrl.target_xyz[1] -= STEP_M; moved = True
            if pk.Key.right in keys_now:
                ctrl.target_xyz[1] += STEP_M; moved = True
            if '[' in keys_now:
                ctrl.target_xyz[2] -= STEP_M; moved = True
            if ']' in keys_now:
                ctrl.target_xyz[2] += STEP_M; moved = True
            if 'r' in keys_now:
                ctrl.target_rpy[0] -= STEP_R; moved = True
            if 'f' in keys_now:
                ctrl.target_rpy[0] += STEP_R; moved = True
            if 't' in keys_now:
                ctrl.target_rpy[1] -= STEP_R; moved = True
            if 'g' in keys_now:
                ctrl.target_rpy[1] += STEP_R; moved = True
            if 'y' in keys_now:
                ctrl.target_rpy[2] -= STEP_R; moved = True
            if 'h' in keys_now:
                ctrl.target_rpy[2] += STEP_R; moved = True
            if moved:
                ctrl.dirty = True


# ---------------------------------------------------------------------------
# LeaderSim — ROS2 Node
# ---------------------------------------------------------------------------
class LeaderSim(Node):
    """MuJoCo simulation for the Leader arm (ROS2 node)."""

    def __init__(self, robot: str, model_path: str):
        super().__init__('leader_sim_node')
        self._init_params(robot)
        self._init_mujoco(model_path)
        self._init_pubsub()
        self._init_threads(robot)

    # ---- Sub-init methods --------------------------------------------------

    def _init_params(self, robot: str):
        cfg = ROBOT_CONFIG[robot]
        self.KP_USER = cfg['KP_USER']
        self.KD_USER = cfg['KD_USER']
        self.KP_HOLD = cfg['KP_HOLD']
        self.KD_HOLD = cfg['KD_HOLD']
        self.KP_BI   = cfg['KP_BI']
        self.KD_BI   = cfg['KD_BI']
        self.OVERFORCE_USER       = cfg['OVERFORCE_USER']
        self.OVERFORCE_CONSTRAINT = cfg['OVERFORCE_CONSTRAINT']

    def _init_mujoco(self, model_path: str):
        self.get_logger().info(f'model      : {model_path}')

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

        self.eef_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, 'eef')
        if self.eef_site_id < 0:
            raise RuntimeError("EEF site 'eef' not found in the model")

        self.robot_body_ids = [
            bid for name in ROBOT_BODIES
            if (bid := mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name)) >= 0
        ]

        self.initial_xyz = self.data.site_xpos[self.eef_site_id].copy()
        self._initial_mat = \
            self.data.site_xmat[self.eef_site_id].reshape(3, 3).copy()
        self.initial_rpy = mat_to_rpy(self._initial_mat)

        self.get_logger().info(
            f'initial EEF xyz = ({self.initial_xyz[0]:+.3f}, '
            f'{self.initial_xyz[1]:+.3f}, {self.initial_xyz[2]:+.3f})')
        self.get_logger().info(
            f'initial EEF rpy = ({self.initial_rpy[0]:+.3f}, '
            f'{self.initial_rpy[1]:+.3f}, {self.initial_rpy[2]:+.3f})')

        self.ctrl = ControlState(self.initial_xyz, self.initial_rpy, HOME_QPOS)
        self.ik_data = mujoco.MjData(self.model)

    def _init_pubsub(self):
        """Create ROS2 publishers and subscribers (pure pub/sub only)."""
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        # State publisher
        self.state_pub = self.create_publisher(
            JointState, '/ur10e/leader/joint_state', 10)

        # Subscribe to follower's joint state (bilateral coupling)
        self.create_subscription(
            JointState, '/ur10e/follower/joint_state', self._peer_cb, 10)

        # Mode pub/sub (latched)
        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)
        self.create_subscription(
            Float64MultiArray, '/ur10e/mode', self._mode_cb, qos_latched)

        # Reset subscriber (latched)
        self.create_subscription(
            Int32, '/ur10e/reset', self._reset_cb, qos_latched)

        # Pre-allocate reusable message objects (avoid per-iteration allocation)
        self._state_msg = JointState()
        self._state_msg.name = list(JOINT_NAMES)
        self._mode_msg = Float64MultiArray()

        # Publish initial state + mode
        self._publish_state(
            self.data.qpos[self.qpos_ids].copy(),
            np.zeros(N), np.zeros(N))
        self._publish_mode(MODE_ACTIVE)

    def _init_threads(self, robot: str):
        """Create cached subscription data, keyboard handler, exchange buffer,
        control thread."""
        # Publish throttle
        self._pub_tick = 0
        self._PUB_EVERY = 2   # 500 Hz control / 2 = 250 Hz publish

        # Cached peer (follower) joint state for bilateral coupling
        self._peer_q = None
        self._peer_dq = None
        self._peer_lock = threading.Lock()
        self._mode_state = MODE_ACTIVE
        self._mode_t_start = 0.0
        self._mode_duration = 0.0
        self._mode_lock = threading.Lock()
        self._reset_counter = 0

        # Keyboard handler
        self.keyboard = KeyboardHandler(robot)

        # Exchange buffer
        self.buf = ExchangeBuffer(
            self.data.qpos[self.qpos_ids].copy(),
            self.initial_xyz, self._initial_mat)

        mujoco.mj_forward(self.model, self.data)
        with self.buf.lock:
            self.buf.tau_grav[:] = self.data.qfrc_bias[self.qvel_ids]

        self.stop_event = threading.Event()
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name='leader-control',
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

    def _reset_cb(self, msg: Int32):
        self._reset_counter = msg.data

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

    # ---- Key callback (MuJoCo GLFW single-shot keys) ----------------------

    def _key_callback(self, keycode: int):
        cur_state = self._mode_state

        if keycode == ord('P'):
            if cur_state == MODE_ACTIVE:
                self._publish_mode(MODE_PAUSED)
                self.get_logger().info('key P  →  PAUSED')
            else:
                self._publish_mode(MODE_ACTIVE)
                self.get_logger().info('key P  →  ACTIVE')
            return

        if keycode == ord('I'):
            self._publish_mode(MODE_HOMING, time.time(), HOMING_DURATION)
            self.get_logger().info(
                f'key I  →  HOMING to init pose ({HOMING_DURATION:.1f}s)')
            return

        if keycode == ord('0') and cur_state == MODE_ACTIVE:
            with self.ctrl.lock:
                self.ctrl.target_xyz[:] = self.initial_xyz
                self.ctrl.target_rpy[:] = self.initial_rpy
                self.ctrl.q_target[:]   = HOME_QPOS
                self.ctrl.dirty         = True
            self.get_logger().info('key 0  →  reset EEF target to home')

    # ---- Control thread ----------------------------------------------------

    def _control_loop(self):
        prev_state = MODE_ACTIVE
        q_hold = HOME_QPOS.copy()
        q_home_start = HOME_QPOS.copy()
        overforce_cooldown = 0.0
        last_reset_counter = 0

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
                    drag_active = self.buf.drag_active
                    eef_xyz = self.buf.eef_xyz.copy()
                    eef_mat = self.buf.eef_mat.copy()

                # ---- Read mode from ROS2 topic ----
                with self._mode_lock:
                    cur_state = self._mode_state
                    h_t_start = self._mode_t_start
                    h_duration = self._mode_duration

                # ---- Consume external reset requests ----
                cur_reset_counter = self._reset_counter
                if cur_reset_counter != last_reset_counter:
                    last_reset_counter = cur_reset_counter
                    if cur_state == MODE_ACTIVE:
                        with self.ctrl.lock:
                            self.ctrl.target_xyz[:] = self.initial_xyz
                            self.ctrl.target_rpy[:] = self.initial_rpy
                            self.ctrl.q_target[:] = HOME_QPOS
                            self.ctrl.dirty = True
                        self.get_logger().info(
                            'reset request consumed → EEF target reset to home')

                # ---- State transitions ----
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
                    elif cur_state == MODE_ACTIVE:
                        with self.ctrl.lock:
                            self.ctrl.target_xyz[:] = eef_xyz
                            self.ctrl.target_rpy[:] = mat_to_rpy(eef_mat)
                            self.ctrl.q_target[:] = q.copy()
                            self.ctrl.dirty = False

                # ---- Apply held movement keys (20 Hz) ----
                if cur_state == MODE_ACTIVE:
                    self.keyboard.apply_held_keys(self.ctrl, now)

                # ---- Drag: promote current EEF pose to user target ----
                if cur_state == MODE_ACTIVE and drag_active:
                    with self.ctrl.lock:
                        self.ctrl.q_target[:] = q
                        self.ctrl.target_xyz[:] = eef_xyz
                        self.ctrl.target_rpy[:] = mat_to_rpy(eef_mat)
                        self.ctrl.dirty = False

                # ---- Snapshot user target (for IK) ----
                with self.ctrl.lock:
                    dirty = self.ctrl.dirty
                    self.ctrl.dirty = False
                    target_xyz = self.ctrl.target_xyz.copy()
                    target_rpy = self.ctrl.target_rpy.copy()
                    q_seed = self.ctrl.q_target.copy()

                if dirty:
                    target_quat = rpy_to_quat(target_rpy)
                    q_user = solve_ik(self.model, self.ik_data,
                                      self.eef_site_id,
                                      target_xyz, target_quat,
                                      q_seed, self.qpos_ids, self.qvel_ids)
                    with self.ctrl.lock:
                        self.ctrl.q_target = q_user
                else:
                    q_user = q_seed

                # ---- Read peer (follower) state for bilateral coupling ----
                with self._peer_lock:
                    peer_q = self._peer_q
                    peer_dq = self._peer_dq
                bilateral_active = peer_q is not None

                # ---- State-specific control law ----
                if cur_state == MODE_ACTIVE:
                    if drag_active:
                        tau_user = np.zeros(N)
                        tau = tau_grav - 0.3 * self.KD_USER * dq
                    else:
                        tau_user = (self.KP_USER * (q_user - q)
                                    - self.KD_USER * dq)
                        if bilateral_active:
                            tau_bi = (self.KP_BI * (peer_q - q)
                                      + self.KD_BI * (peer_dq - dq))
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
                    ease = 3.0 * alpha ** 2 - 2.0 * alpha ** 3
                    q_des = (1.0 - ease) * q_home_start + ease * HOME_QPOS
                    tau = (self.KP_HOLD * (q_des - q)
                           - self.KD_HOLD * dq + tau_grav)
                    if alpha >= 1.0:
                        self._publish_mode(MODE_PAUSED)
                        self.get_logger().info('HOMING complete → PAUSED')

                else:
                    tau_user = np.zeros(N)
                    tau = tau_grav

                tau = np.clip(tau, self.act_lo, self.act_hi)

                with self.buf.lock:
                    self.buf.tau[:] = tau

                # ---- Over-force detection ----
                if (cur_state == MODE_ACTIVE
                        and not drag_active
                        and now > overforce_cooldown):
                    tau_user_spring = self.KP_USER * (q_user - q)
                    over_user = np.abs(tau_user_spring) > self.OVERFORCE_USER
                    over_const = (np.abs(tau_contact)
                                  > self.OVERFORCE_CONSTRAINT)
                    if np.any(over_user) or np.any(over_const):
                        if np.any(over_user):
                            i = int(np.argmax(np.abs(tau_user_spring)))
                            self.get_logger().warn(
                                f'OVER-FORCE  user overreach on joint[{i}] = '
                                f'{tau_user_spring[i]:+.1f} Nm  →  PAUSED')
                        if np.any(over_const):
                            i = int(np.argmax(np.abs(tau_contact)))
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

        with mujoco.viewer.launch_passive(
                self.model, self.data,
                show_left_ui=False, show_right_ui=False,
                key_callback=self._key_callback) as viewer:
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -20.0
            viewer.cam.distance = 2.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.4]

            self.get_logger().info(
                'Running.  Click this window to focus, then HOLD:')
            self.get_logger().info(
                '  ↑/↓  -X/+X   ←/→ -Y/+Y   [/] -Z/+Z   (0.01 m / tick)')
            self.get_logger().info(
                '  R/F  roll    T/G pitch   Y/H yaw     (0.05 rad / tick)')
            self.get_logger().info(
                '  P = toggle ACTIVE / PAUSED  (tap)')
            self.get_logger().info(
                '  I = home both arms, then PAUSED  (tap)')
            self.get_logger().info(
                '  0 = reset EEF target to home pose  (tap)')
            self.get_logger().info(
                'Movement keys auto-repeat at 20 Hz while held.')

            try:
                while viewer.is_running() and not self.stop_event.is_set():
                    step_start = time.time()

                    with self.buf.lock:
                        self.data.ctrl[:N] = self.buf.tau

                    mujoco.mj_step(self.model, self.data)

                    drag_active = bool(np.any(np.abs(
                        self.data.xfrc_applied[self.robot_body_ids, :]
                    ) > 1e-6))

                    with self.buf.lock:
                        self.buf.q[:] = self.data.qpos[self.qpos_ids]
                        self.buf.dq[:] = self.data.qvel[self.qvel_ids]
                        self.buf.tau_grav[:] = \
                            self.data.qfrc_bias[self.qvel_ids]
                        self.buf.tau_contact[:] = \
                            self.data.qfrc_constraint[self.qvel_ids]
                        self.buf.drag_active = drag_active
                        self.buf.eef_xyz[:] = \
                            self.data.site_xpos[self.eef_site_id]
                        self.buf.eef_mat[:] = \
                            self.data.site_xmat[self.eef_site_id] \
                            .reshape(3, 3)

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

    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['ur10e', 'ur3e'], default='ur10e',
                        help='Leader robot type (default ur10e)')
    parser.add_argument('--model', default=None,
                        help='Override the MJCF path.')
    args = parser.parse_args()

    from ament_index_python.packages import get_package_share_directory
    share_dir = get_package_share_directory('ur10e_teleop_mujoco')
    model_path = args.model or os.path.join(
        share_dir, 'xml',
        ROBOT_CONFIG[args.robot]['model_file'])

    node = LeaderSim(args.robot, model_path)

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
