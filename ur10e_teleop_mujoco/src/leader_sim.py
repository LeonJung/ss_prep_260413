#!/usr/bin/env python3
"""
leader_sim.py — MuJoCo simulation for the UR10E Leader arm.

Control architecture (classical position-position bilateral teleoperation):

  tau_leader = tau_user_PD          ← soft virtual spring from user keyboard
             + tau_bilateral_PD     ← stiff coupling to follower (from shm)
             + gravity_compensation

The user PD is intentionally ~10× softer than the bilateral coupling, so when
the follower is blocked by an obstacle the leader is also pulled back and the
user feels a "wall".

State machine (shared with follower_sim via /dev/shm/ur10e_mode):

  ACTIVE  : normal bilateral teleop, user keyboard moves leader
  PAUSED  : both arms hold their current q, keyboard ignored
  HOMING  : both arms interpolate slowly back to HOME_QPOS, then PAUSED

Keys (focus the leader window — title contains "MuJoCo : ur10e" but NOT
"follower" — hold for continuous motion):
  ↑ / ↓  : -X / +X     (0.01 m per 50 ms)
  ← / →  : -Y / +Y     (0.01 m per 50 ms)
  [ / ]  : -Z / +Z     (0.01 m per 50 ms)
  R / F  : -roll  / +roll     (0.05 rad per 50 ms)
  T / G  : -pitch / +pitch    (0.05 rad per 50 ms)
  Y / H  : -yaw   / +yaw      (0.05 rad per 50 ms)
  0      : reset EEF target to home pose (single-shot, does NOT move robot)
  P      : toggle ACTIVE / PAUSED (single-shot)
  I      : slowly return both arms to init pose, then PAUSED (single-shot)

Movement keys auto-repeat while held at 20 Hz.  Single-shot keys must be
tapped — they are handled by MuJoCo's built-in key callback rather than
the pynput listener to avoid double-triggering on hold.

Over-force: if the user pushes the leader hard enough that |tau_user| exceeds
OVERFORCE_USER, or if the leader's constraint force (hitting something) exceeds
OVERFORCE_CONSTRAINT, the system auto-transitions to PAUSED.
"""

import argparse
import os
import sys
import threading
import time
import traceback

import numpy as np
import mujoco
import mujoco.viewer

from pynput import keyboard as pk
import Xlib.display
import Xlib.X

sys.path.insert(0, os.path.dirname(__file__))
import struct
from shm_manager import (
    StateWriter, ModeShm, ResetShm,
    _open_shm, _STATE_FMT, STATE_SIZE,
    MODE_ACTIVE, MODE_PAUSED, MODE_HOMING,
)

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
#
# Both UR10e and UR3e share:
#   - identical joint topology (6 revolute joints, same names)
#   - identical HOME_QPOS (UR standard "upright" home)
#   - identical keyboard step sizes / repeat interval
#
# They differ in:
#   - model file (mesh/geometry)
#   - user / hold / bilateral PD gains (scaled to peak joint torque)
#   - over-force thresholds (scaled to peak joint torque)
#
# UR3e is a smaller arm (~40% reach) with much lower peak joint torques
# (54/54/28/9/9/9 Nm vs UR10e's 330/330/150/56/56/56 Nm).  Gains are scaled
# so the same control behaviour applies — the "10× bilateral-over-user"
# ratio is preserved across robots.
#
# Note on bilateral coupling:  the shm command written by bilateral_control
# carries kp/kd values tuned for UR10e (SIM_KP=[400,400,300,100,100,50]).
# leader_sim IGNORES those fields and substitutes its robot-local KP_BI/KD_BI
# so that an UR3e leader controlled by bilateral_control written for UR10e
# still uses safe gains.  follower_sim is always UR10e so it keeps using
# the shm kp/kd directly.
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
        # UR3e torques are ≈ 0.17× UR10e on shoulder/wrist, ≈ 0.19× on
        # elbow.  PD gains start from that ratio but user-side gains are
        # then boosted 3× so they overcome the model's joint damping
        # (size2=5, size1=3, size0=1 Nm·s/rad) within the same settling
        # window as UR10e — without boosting, KP_USER*error << damping*dq
        # and the arm barely moves back to home after a reset.  All values
        # are still well below the UR3e peak torque limits (54/28/9 Nm).
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

HOMING_DURATION = 3.0   # seconds

STEP_M = 0.01    # metres per tick  (0.2 m/s at 20 Hz repeat)
STEP_R = 0.05    # radians per tick (1 rad/s at 20 Hz repeat)

REPEAT_INTERVAL = 0.05   # seconds between auto-repeat ticks (20 Hz)

RENDER_EVERY = 16   # render every N physics steps (≈ 31 Hz)

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
# Helper: normalise pynput key to a hashable form
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
    """Return a zero-arg callable that returns True iff the leader MuJoCo
    window currently has X11 input focus."""
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
# Exchange buffer (lock-protected thread communication)
# ---------------------------------------------------------------------------
class ExchangeBuffer:
    """Lock-protected numpy arrays shared between MuJoCo and control threads."""

    def __init__(self, q0: np.ndarray, eef_xyz0: np.ndarray,
                 eef_mat0: np.ndarray):
        self.lock = threading.Lock()
        # physics → control
        self.q = q0.copy()
        self.dq = np.zeros(N)
        self.tau_grav = np.zeros(N)
        self.tau_contact = np.zeros(N)
        self.drag_active = False
        self.eef_xyz = eef_xyz0.copy()
        self.eef_mat = eef_mat0.copy()       # (3, 3)
        # control → physics
        self.tau = np.zeros(N)


# ---------------------------------------------------------------------------
# Control state shared between key callback and control thread
# ---------------------------------------------------------------------------
class ControlState:
    def __init__(self, xyz0, rpy0, q0):
        self.lock       = threading.Lock()
        self.target_xyz = xyz0.copy()
        self.target_rpy = rpy0.copy()
        self.q_target   = q0.copy()
        self.dirty      = True


# ---------------------------------------------------------------------------
# KeyboardHandler — pynput hold-to-repeat + X11 focus
# ---------------------------------------------------------------------------
class KeyboardHandler:
    """Manages pynput hold-to-repeat keys and X11 focus checking."""

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
        """If repeat interval elapsed and leader window focused, apply
        held movement keys to the ControlState target."""
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
# LeaderSim
# ---------------------------------------------------------------------------
class LeaderSim:
    """MuJoCo simulation for the Leader arm (UR10e or UR3e)."""

    def __init__(self, robot: str, model_path: str):
        self._init_params(robot)
        self._init_mujoco(model_path)
        self._init_shm()
        self._init_threads(robot)

    # ---- Sub-init methods --------------------------------------------------

    def _init_params(self, robot: str):
        """Extract per-robot gains from ROBOT_CONFIG."""
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
        """Load model, create data, resolve IDs, compute initial EEF pose."""
        print(f'[leader_sim] model      : {model_path}')

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

        # Initial EEF pose via FK
        self.initial_xyz = self.data.site_xpos[self.eef_site_id].copy()
        self._initial_mat = \
            self.data.site_xmat[self.eef_site_id].reshape(3, 3).copy()
        self.initial_rpy = mat_to_rpy(self._initial_mat)

        print(f'[leader_sim] initial EEF xyz = '
              f'({self.initial_xyz[0]:+.3f}, {self.initial_xyz[1]:+.3f}, '
              f'{self.initial_xyz[2]:+.3f})')
        print(f'[leader_sim] initial EEF rpy = '
              f'({self.initial_rpy[0]:+.3f}, {self.initial_rpy[1]:+.3f}, '
              f'{self.initial_rpy[2]:+.3f})')

        self.ctrl = ControlState(self.initial_xyz, self.initial_rpy, HOME_QPOS)
        self.ik_data = mujoco.MjData(self.model)

    def _init_shm(self):
        """Set up shared-memory readers/writers."""
        self.state_writer = StateWriter('leader')
        self._peer_shm = _open_shm('/dev/shm/ur10e_follower_state', STATE_SIZE)
        self.state_writer.write(
            self.data.qpos[self.qpos_ids].copy(), np.zeros(N),
            np.zeros(N), time.time())

        self.mode_shm = ModeShm()
        self.mode_shm.write(MODE_ACTIVE)

        self.reset_shm = ResetShm()
        self.reset_shm.clear()

    def _init_threads(self, robot: str):
        """Create exchange buffer, keyboard handler, stop event, control thread."""
        self.keyboard = KeyboardHandler(robot)

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

    # ---- Key callback (MuJoCo GLFW single-shot keys) ----------------------

    def _key_callback(self, keycode: int):
        cur_state, _, _ = self.mode_shm.read()

        if keycode == ord('P'):
            if cur_state == MODE_ACTIVE:
                self.mode_shm.write(MODE_PAUSED)
                print('[leader_sim] key P  →  PAUSED')
            else:
                self.mode_shm.write(MODE_ACTIVE)
                print('[leader_sim] key P  →  ACTIVE')
            return

        if keycode == ord('I'):
            self.mode_shm.write(MODE_HOMING, time.time(), HOMING_DURATION)
            print(f'[leader_sim] key I  →  HOMING to init pose '
                  f'({HOMING_DURATION:.1f}s)')
            return

        if keycode == ord('0') and cur_state == MODE_ACTIVE:
            with self.ctrl.lock:
                self.ctrl.target_xyz[:] = self.initial_xyz
                self.ctrl.target_rpy[:] = self.initial_rpy
                self.ctrl.q_target[:]   = HOME_QPOS
                self.ctrl.dirty         = True
            print('[leader_sim] key 0  →  reset EEF target to home')

    # ---- Control thread ----------------------------------------------------

    def _control_loop(self):
        """Control thread: SHM I/O + keyboard + IK + mode + PD + over-force."""
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

                # ---- Read shared mode & detect edges ----
                cur_state, h_t_start, h_duration = self.mode_shm.read()

                # ---- Consume external reset requests ----
                cur_reset_counter = self.reset_shm.read()
                if cur_reset_counter != last_reset_counter:
                    last_reset_counter = cur_reset_counter
                    if cur_state == MODE_ACTIVE:
                        with self.ctrl.lock:
                            self.ctrl.target_xyz[:] = self.initial_xyz
                            self.ctrl.target_rpy[:] = self.initial_rpy
                            self.ctrl.q_target[:] = HOME_QPOS
                            self.ctrl.dirty = True
                        print('[leader_sim] reset request consumed '
                              '→ EEF target reset to home')

                # ---- State transitions ----
                if cur_state != prev_state:
                    print(f'[leader_sim] state  '
                          f'{_state_name(prev_state)} → '
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

                # ---- Read peer (follower) state directly from shm ----
                self._peer_shm.seek(0)
                pv = struct.unpack(_STATE_FMT,
                                   self._peer_shm.read(STATE_SIZE))
                peer_q = np.array(pv[0:6])
                peer_dq = np.array(pv[6:12])
                peer_ts = pv[18]
                bilateral_active = peer_ts > 0

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
                        self.mode_shm.write(MODE_PAUSED)
                        print('[leader_sim] HOMING complete → PAUSED')

                else:
                    tau_user = np.zeros(N)
                    tau = tau_grav

                tau = np.clip(tau, self.act_lo, self.act_hi)

                # ---- Write tau for MuJoCo thread ----
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
                            print(
                                f'[leader_sim] OVER-FORCE  user overreach '
                                f'on joint[{i}] = '
                                f'{tau_user_spring[i]:+.1f} Nm  →  PAUSED')
                        if np.any(over_const):
                            i = int(np.argmax(np.abs(tau_contact)))
                            print(
                                f'[leader_sim] OVER-FORCE  contact '
                                f'on joint[{i}] = '
                                f'{tau_contact[i]:+.1f} Nm  →  PAUSED')
                        self.mode_shm.write(MODE_PAUSED)
                        overforce_cooldown = now + 1.0

                # ---- Publish state to shm ----
                self.state_writer.write(q, dq, tau_contact, time.time())

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
        """Start control thread, open viewer, and run the physics loop.
        Must be called from the main thread (GLFW requirement)."""
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

            print('[leader_sim] Running.  Click this window to focus, '
                  'then HOLD:')
            print('[leader_sim]   ↑/↓  -X/+X   ←/→ -Y/+Y   '
                  '[/] -Z/+Z   (0.01 m / tick)')
            print('[leader_sim]   R/F  roll    T/G pitch   '
                  'Y/H yaw     (0.05 rad / tick)')
            print('[leader_sim]   P = toggle ACTIVE / PAUSED  (tap)')
            print('[leader_sim]   I = home both arms, then PAUSED  (tap)')
            print('[leader_sim]   0 = reset EEF target to home pose  (tap)')
            print('[leader_sim] Movement keys auto-repeat at 20 Hz '
                  'while held.')

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
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['ur10e', 'ur3e'], default='ur10e',
                        help='Leader robot type (default ur10e)')
    parser.add_argument('--model', default=None,
                        help='Override the MJCF path.  Usually unnecessary — '
                             '--robot selects the right file under models/.')
    args = parser.parse_args()

    model_path = args.model or os.path.join(
        os.path.dirname(__file__), '..', 'xml',
        ROBOT_CONFIG[args.robot]['model_file'])

    print(f'[leader_sim] robot      : {args.robot}')

    LeaderSim(args.robot, model_path).run()


if __name__ == '__main__':
    main()
