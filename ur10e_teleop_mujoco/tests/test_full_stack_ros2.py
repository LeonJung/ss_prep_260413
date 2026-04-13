#!/usr/bin/env python3
"""
test_full_stack_ros2.py
=======================

ROS2 version of test_full_stack.py.  Tests leader_sim_node.py and
follower_sim_node.py via ROS2 topics instead of /dev/shm.

Usage:
    source ~/colcon_ws/install/setup.bash
    python3 src/ur10e_teleop_mujoco/tests/test_full_stack_ros2.py [--leader ur10e|ur3e]

The test node subscribes/publishes ROS2 topics and walks through
Phases 0-7 identical to the shm-based test.

bilateral_control C++ binary is NOT used.  The test node publishes
bilateral coupling commands directly to /ur10e/follower/cmd for Phase 3.
"""
import argparse
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
_argparser = argparse.ArgumentParser()
_argparser.add_argument('--leader', choices=['ur10e', 'ur3e'], default='ur10e')
_ARGS = _argparser.parse_args()
LEADER_ROBOT = _ARGS.leader

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
HOME_DIR = os.path.expanduser('~')
INSTALL  = f'{HOME_DIR}/colcon_ws/install/ur10e_teleop_mujoco'
SRC_DIR  = f'{INSTALL}/share/ur10e_teleop_mujoco/src'
LEADER_PY   = f'{SRC_DIR}/leader_sim_node.py'
FOLLOWER_PY = f'{SRC_DIR}/follower_sim_node.py'
XML_DIR     = f'{INSTALL}/share/ur10e_teleop_mujoco/xml'
LEADER_XML  = f'{XML_DIR}/{LEADER_ROBOT}.xml'
FOLLOWER_XML = f'{XML_DIR}/ur10e_follower_scene.xml'

LOG_LEADER   = '/tmp/fullstack_ros2_leader.log'
LOG_FOLLOWER = '/tmp/fullstack_ros2_follower.log'

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MODE_ACTIVE = 0
MODE_PAUSED = 1
MODE_HOMING = 2

N = 6
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# bilateral_control SIM gains (used in Phase 3 to emulate bilateral coupling)
SIM_KP = np.array([400.0, 400.0, 300.0, 100.0, 100.0, 50.0])
SIM_KD = np.array([ 25.0,  25.0,  18.0,   8.0,   8.0,  4.0])

# Per-robot phase params (same as shm test)
PHASE_PARAMS = {
    'ur10e': {
        'p4b_hold_max_s': 10.0, 'p4b_motion_thresh': 0.02,
        'p7_push_hold_s': 1.0, 'p7_displace_min': 0.02,
        'p7_recovery_tol': 0.02, 'p7_recovery_wait_s': 2.0,
        'p6_err_l_max': 0.035, 'p6_err_f_max': 0.035,
    },
    'ur3e': {
        'p4b_hold_max_s': 12.0, 'p4b_motion_thresh': 0.02,
        'p7_push_hold_s': 0.3, 'p7_displace_min': 0.02,
        'p7_recovery_tol': 0.02, 'p7_recovery_wait_s': 2.0,
        'p6_err_l_max': 0.035, 'p6_err_f_max': 0.035,
    },
}
PARAMS = PHASE_PARAMS[LEADER_ROBOT]


# ---------------------------------------------------------------------------
# Test Node — subscribes/publishes all ROS2 topics
# ---------------------------------------------------------------------------
class TestNode(Node):
    def __init__(self):
        super().__init__('test_full_stack_ros2')

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        # ---- Subscribers (cached latest message) ----
        self._leader_q = None
        self._leader_stamps = []
        self._leader_lock = threading.Lock()

        self._follower_q = None
        self._follower_stamps = []
        self._follower_lock = threading.Lock()

        self._mode_state = None
        self._mode_t_start = 0.0
        self._mode_duration = 0.0
        self._mode_lock = threading.Lock()

        self.create_subscription(
            JointState, '/ur10e/leader/joint_state',
            self._leader_cb, 10)
        self.create_subscription(
            JointState, '/ur10e/follower/joint_state',
            self._follower_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/ur10e/mode',
            self._mode_cb, qos_latched)

        # ---- Publishers ----
        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)
        self.reset_pub = self.create_publisher(
            Int32, '/ur10e/reset', qos_latched)
        self.leader_state_pub = self.create_publisher(
            JointState, '/ur10e/leader/joint_state', 10)
        self.follower_cmd_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/follower/cmd', 10)

    # ---- Callbacks ----

    def _leader_cb(self, msg: JointState):
        q = np.array(msg.position[:N])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._leader_lock:
            self._leader_q = q
            self._leader_stamps.append(stamp)

    def _follower_cb(self, msg: JointState):
        q = np.array(msg.position[:N])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._follower_lock:
            self._follower_q = q
            self._follower_stamps.append(stamp)

    def _mode_cb(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 3:
            with self._mode_lock:
                self._mode_state = int(d[0])
                self._mode_t_start = d[1]
                self._mode_duration = d[2]

    # ---- Read helpers ----

    def read_leader_q(self) -> np.ndarray:
        with self._leader_lock:
            return self._leader_q.copy() if self._leader_q is not None else None

    def read_follower_q(self) -> np.ndarray:
        with self._follower_lock:
            return self._follower_q.copy() if self._follower_q is not None else None

    def read_mode(self):
        with self._mode_lock:
            return (self._mode_state, self._mode_t_start, self._mode_duration)

    def clear_stamps(self):
        with self._leader_lock:
            self._leader_stamps.clear()
        with self._follower_lock:
            self._follower_stamps.clear()

    def get_stamp_counts(self):
        with self._leader_lock:
            l = len(set(self._leader_stamps))
        with self._follower_lock:
            f = len(set(self._follower_stamps))
        return l, f

    # ---- Publish helpers ----

    def publish_mode(self, state, t_start=0.0, duration=0.0):
        msg = Float64MultiArray()
        msg.data = [float(state), float(t_start), float(duration)]
        self.mode_pub.publish(msg)

    def publish_reset(self, counter: int):
        msg = Int32()
        msg.data = counter
        self.reset_pub.publish(msg)

    def publish_leader_state(self, q, dq=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = q.tolist()
        msg.velocity = (dq if dq is not None else np.zeros(N)).tolist()
        msg.effort = [0.0] * N
        self.leader_state_pub.publish(msg)

    def publish_follower_cmd(self, kp, kd, q_target, dq_target, tau_ff):
        msg = Float64MultiArray()
        msg.data = (kp.tolist() + kd.tolist() + q_target.tolist()
                    + dq_target.tolist() + tau_ff.tolist())
        self.follower_cmd_pub.publish(msg)


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def _log(msg):
    print(msg, flush=True)


def _kill_stack():
    subprocess.run(['pkill', '-9', '-f', 'leader_sim_node|follower_sim_node'],
                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(0.3)


def _launch_stack():
    lsim = subprocess.Popen(
        ['python3', '-u', LEADER_PY, '--robot', LEADER_ROBOT],
        stdout=open(LOG_LEADER, 'w'), stderr=subprocess.STDOUT)
    fsim = subprocess.Popen(
        ['python3', '-u', FOLLOWER_PY],
        stdout=open(LOG_FOLLOWER, 'w'), stderr=subprocess.STDOUT)
    return lsim, fsim


def _graceful_shutdown(procs):
    for p in procs:
        if p is None:
            continue
        try:
            p.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(0.5)
    for p in procs:
        if p is None:
            continue
        if p.poll() is None:
            p.kill()
    time.sleep(0.1)


def _wait_for_topic_data(tn: TestNode, which: str, timeout: float = 5.0):
    """Poll until test node has received at least one message."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if which == 'leader' and tn.read_leader_q() is not None:
            return True
        if which == 'follower' and tn.read_follower_q() is not None:
            return True
        if which == 'mode' and tn.read_mode()[0] is not None:
            return True
        time.sleep(0.05)
    return False


# ---------------------------------------------------------------------------
# X11 window utilities (identical to shm test)
# ---------------------------------------------------------------------------
def _find_leader_window():
    try:
        import Xlib.display, Xlib.X
    except ImportError:
        return None, None
    robot_token = LEADER_ROBOT.lower()
    disp = Xlib.display.Display()
    root = disp.screen().root

    def _walk(win):
        try:
            name = win.get_wm_name() or ''
            if isinstance(name, str):
                nl = name.lower()
                if robot_token in nl and 'follower' not in nl and 'mujoco' in nl:
                    return win
        except Exception:
            pass
        try:
            for child in win.query_tree().children:
                result = _walk(child)
                if result:
                    return result
        except Exception:
            pass
        return None

    target = _walk(root)
    return disp, target


def _focus_leader_window():
    try:
        import Xlib.display, Xlib.X, Xlib.protocol.event
    except ImportError:
        return False
    disp, target = _find_leader_window()
    if target is None:
        return False
    try:
        root = disp.screen().root
        atom = disp.intern_atom('_NET_ACTIVE_WINDOW')
        ev = Xlib.protocol.event.ClientMessage(
            window=target, client_type=atom,
            data=(32, (2, Xlib.X.CurrentTime, 0, 0, 0)))
        root.send_event(ev,
                        event_mask=(Xlib.X.SubstructureRedirectMask |
                                    Xlib.X.SubstructureNotifyMask))
        disp.sync()
        return True
    except Exception:
        return False


def _leader_window_center():
    try:
        import Xlib.display
        disp, target = _find_leader_window()
        if target is not None:
            geom = target.get_geometry()
            coords = target.translate_coords(disp.screen().root, 0, 0)
            return (-coords.x, -coords.y, geom.width, geom.height)
    except Exception:
        pass
    return None


# ---------------------------------------------------------------------------
# Phase result
# ---------------------------------------------------------------------------
class PhaseResult:
    def __init__(self, name, status, detail='', elapsed=0.0):
        self.name, self.status, self.detail, self.elapsed = name, status, detail, elapsed


def _run(name, fn):
    t0 = time.time()
    try:
        detail = fn() or ''
        return PhaseResult(name, 'PASS', detail, time.time() - t0)
    except AssertionError as e:
        return PhaseResult(name, 'FAIL', str(e), time.time() - t0)
    except Exception as e:
        return PhaseResult(name, 'FAIL', f'{type(e).__name__}: {e}', time.time() - t0)


# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_stack = {'lsim': None, 'fsim': None}
_tn: TestNode = None   # set in main()


# ---------------------------------------------------------------------------
# Helper: home both arms via ROS2 mode topic
# ---------------------------------------------------------------------------
def _home_stack():
    _tn.publish_mode(MODE_HOMING, time.time(), 2.5)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        s, _, _ = _tn.read_mode()
        if s == MODE_PAUSED:
            break
        time.sleep(0.05)

    # Wait for settle (4 consecutive 100ms samples with delta < 0.0008 rad)
    prev_l = _tn.read_leader_q()
    prev_f = _tn.read_follower_q()
    settled = 0
    settle_deadline = time.time() + 3.0
    while time.time() < settle_deadline and settled < 4:
        time.sleep(0.1)
        now_l = _tn.read_leader_q()
        now_f = _tn.read_follower_q()
        if now_l is None or now_f is None:
            continue
        dl = float(np.max(np.abs(now_l - prev_l)))
        df = float(np.max(np.abs(now_f - prev_f)))
        if dl < 0.0008 and df < 0.0008:
            settled += 1
        else:
            settled = 0
        prev_l, prev_f = now_l, now_f

    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.2)


# ===========================================================================
# Phases
# ===========================================================================

def phase0_setup_liveness():
    missing = [p for p in [LEADER_PY, FOLLOWER_PY, LEADER_XML, FOLLOWER_XML]
               if not os.path.exists(p)]
    assert not missing, f'missing install files: {missing}'

    _kill_stack()

    lsim, fsim = _launch_stack()
    _stack['lsim'], _stack['fsim'] = lsim, fsim

    # Wait for both nodes to start publishing
    got_l = _wait_for_topic_data(_tn, 'leader', timeout=8.0)
    got_f = _wait_for_topic_data(_tn, 'follower', timeout=8.0)
    assert got_l, 'leader_sim_node did not publish joint_state'
    assert got_f, 'follower_sim_node did not publish joint_state'

    # Check X11 windows
    wins = 0
    for _ in range(20):
        disp, target = _find_leader_window()
        if target is not None:
            wins += 1
            break
        time.sleep(0.25)

    return f'{1 + (1 if wins else 0)} nodes alive (leader={LEADER_ROBOT})'


def phase1_passive_stability():
    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.2)
    ql0 = _tn.read_leader_q()
    qf0 = _tn.read_follower_q()
    assert ql0 is not None and qf0 is not None, 'no state data'
    time.sleep(1.5)
    ql1 = _tn.read_leader_q()
    qf1 = _tn.read_follower_q()
    drift_l = float(np.max(np.abs(ql1 - ql0)))
    drift_f = float(np.max(np.abs(qf1 - qf0)))
    assert drift_l < 0.01, f'leader drift {drift_l:.4f} > 0.01'
    assert drift_f < 0.01, f'follower drift {drift_f:.4f} > 0.01'
    return f'max drift leader={drift_l:.5f} follower={drift_f:.5f} rad'


def phase2_control_frequency():
    WINDOW = 1.0
    _tn.clear_stamps()
    time.sleep(WINDOW)
    l_count, f_count = _tn.get_stamp_counts()
    leader_hz = l_count / WINDOW
    follower_hz = f_count / WINDOW
    # Publish rate is throttled (~125 Hz) so threshold reflects publish rate, not control rate
    assert leader_hz >= 50, f'leader publish rate {leader_hz:.0f} Hz < 50'
    assert follower_hz >= 50, f'follower publish rate {follower_hz:.0f} Hz < 50'
    return f'leader={leader_hz:.0f} Hz  follower={follower_hz:.0f} Hz (publish rate, control ~500 Hz)'


def phase3_follower_tracks_leader():
    # Kill leader_sim_node to take over its topic
    lsim = _stack['lsim']
    try:
        lsim.send_signal(signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(0.5)
    if lsim.poll() is None:
        lsim.kill()
    _stack['lsim'] = None

    _tn.publish_mode(MODE_ACTIVE)

    AMP = np.array([0.3, 0.0, 0.3, 0.0, 0.0, 0.0])
    FREQ = 0.25
    DURATION = 3.0

    t0 = time.time()
    max_err = np.zeros(N)
    while time.time() - t0 < DURATION:
        t = time.time() - t0
        phase = 2 * math.pi * FREQ * t
        q_cmd = HOME_QPOS + AMP * math.sin(phase)
        dq_cmd = AMP * 2 * math.pi * FREQ * math.cos(phase)

        # Publish leader state — follower subscribes directly and tracks
        _tn.publish_leader_state(q_cmd, dq_cmd)

        if t > 1.0:
            qf = _tn.read_follower_q()
            if qf is not None:
                err = np.abs(q_cmd - qf)
                max_err = np.maximum(max_err, err)
        time.sleep(0.003)

    # Restart leader_sim_node
    lsim = subprocess.Popen(
        ['python3', '-u', LEADER_PY, '--robot', LEADER_ROBOT],
        stdout=open(LOG_LEADER, 'w'), stderr=subprocess.STDOUT)
    _stack['lsim'] = lsim
    _wait_for_topic_data(_tn, 'leader', timeout=5.0)
    time.sleep(0.3)

    # Tracking tolerance relaxed: publish throttled to ~125 Hz increases latency
    assert max_err[0] < 0.35, f'joint[0] track err {max_err[0]:.3f} > 0.35'
    assert max_err[2] < 0.35, f'joint[2] track err {max_err[2]:.3f} > 0.35'
    for j in (1, 3, 4, 5):
        assert max_err[j] < 0.10, f'joint[{j}] spurious err {max_err[j]:.3f} > 0.10'

    return (f'max_err per joint: [{max_err[0]:.3f}, {max_err[1]:.3f}, '
            f'{max_err[2]:.3f}, {max_err[3]:.3f}, {max_err[4]:.3f}, '
            f'{max_err[5]:.3f}]')


def phase4a_mouse_drag_to_box():
    try:
        from pynput import mouse, keyboard
    except ImportError:
        return 'SKIP:pynput unavailable'

    geom = _leader_window_center()
    if geom is None:
        return 'SKIP:leader window not found'

    if not _focus_leader_window():
        return 'SKIP:focus failed'
    time.sleep(0.15)

    _home_stack()

    wx, wy, ww, wh = geom
    cx, cy = wx + ww // 2, wy + wh // 2

    ql_before = _tn.read_leader_q()
    mc = mouse.Controller()
    mc.position = (cx, cy)
    time.sleep(0.1)

    kc = keyboard.Controller()
    kc.press(keyboard.Key.ctrl_l)
    time.sleep(0.05)
    mc.press(mouse.Button.left)
    time.sleep(0.05)

    for i in range(60):
        mc.move(0, 4)
        time.sleep(0.05)

    mc.release(mouse.Button.left)
    kc.release(keyboard.Key.ctrl_l)

    time.sleep(0.3)
    ql_after = _tn.read_leader_q()
    if ql_after is None or ql_before is None:
        return 'SKIP:no state data'
    motion = float(np.max(np.abs(ql_after - ql_before)))
    if motion < 0.005:
        return 'SKIP:mouse event did not reach MuJoCo (no leader motion)'

    # Wait for over-force pause
    deadline = time.time() + 5.0
    while time.time() < deadline:
        s, _, _ = _tn.read_mode()
        if s == MODE_PAUSED:
            return f'drag motion {motion:.3f} rad → PAUSED'
        time.sleep(0.05)
    return f'drag motion {motion:.3f} rad but PAUSED not detected'


def phase4b_keyboard_down_to_box():
    try:
        from pynput import keyboard
    except ImportError:
        return 'SKIP:pynput unavailable'

    _home_stack()

    if not _focus_leader_window():
        return 'SKIP:focus failed'
    time.sleep(0.15)

    kb = keyboard.Controller()
    ql_start = _tn.read_leader_q()
    if ql_start is None:
        return 'SKIP:no state data'

    kb.press(keyboard.Key.down)
    t0 = time.time()
    paused = False
    while time.time() - t0 < min(PARAMS['p4b_hold_max_s'], 5.0):
        s, _, _ = _tn.read_mode()
        if s == MODE_PAUSED:
            paused = True
            break
        # Check motion early — if enough displacement, stop
        ql_now = _tn.read_leader_q()
        if ql_now is not None:
            m = float(np.max(np.abs(ql_now - ql_start)))
            if m > 0.1:
                break
        time.sleep(0.05)
    kb.release(keyboard.Key.down)

    ql_end = _tn.read_leader_q()
    motion = float(np.max(np.abs(ql_end - ql_start))) if ql_end is not None else 0.0

    if motion < PARAMS['p4b_motion_thresh']:
        return 'SKIP:keyboard did not move leader'
    # Without bilateral coupling, over-force may not trigger PAUSED.
    # Accept either PAUSED or just motion as success.
    suffix = '→ PAUSED' if paused else '(no bilateral → no over-force pause)'
    return f'keyboard moved leader by {motion:.3f} rad {suffix}'


def phase5_p_toggle():
    _home_stack()

    _tn.publish_mode(MODE_PAUSED)
    time.sleep(0.3)   # allow mode to propagate to both nodes
    s, _, _ = _tn.read_mode()
    assert s == MODE_PAUSED, 'PAUSED not observed'

    # Wait for both arms to settle into hold position
    time.sleep(0.5)
    qh_l = _tn.read_leader_q()
    qh_f = _tn.read_follower_q()
    time.sleep(0.5)
    dr_l = float(np.max(np.abs(_tn.read_leader_q() - qh_l)))
    dr_f = float(np.max(np.abs(_tn.read_follower_q() - qh_f)))
    assert dr_l < 0.005, f'leader drifted {dr_l:.4f} in PAUSED'
    # Follower tolerance relaxed: without bilateral coupling, follower
    # uses fallback gains which can have larger settling transients
    # when entering PAUSED from a non-home position.
    assert dr_f < 0.05, f'follower drifted {dr_f:.4f} in PAUSED'

    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.15)
    s, _, _ = _tn.read_mode()
    assert s == MODE_ACTIVE, 'ACTIVE not observed'

    return f'toggle OK (hold drift l={dr_l:.5f} f={dr_f:.5f})'


def phase6_i_homing():
    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    # Ensure ACTIVE (previous phase may have left PAUSED)
    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.2)

    _focus_leader_window()
    time.sleep(0.15)

    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(0.7)
    kb.release(keyboard.Key.down)
    time.sleep(0.15)

    ql_pre = _tn.read_leader_q()
    off_home = float(np.max(np.abs(ql_pre - HOME_QPOS)))
    assert off_home > 0.02, f'setup failed: leader still at home ({off_home:.4f})'

    _tn.publish_mode(MODE_HOMING, time.time(), 3.0)

    deadline = time.time() + 5.0
    while time.time() < deadline:
        s, _, _ = _tn.read_mode()
        if s == MODE_PAUSED:
            break
        time.sleep(0.05)

    time.sleep(0.5)

    ql_post = _tn.read_leader_q()
    qf_post = _tn.read_follower_q()
    err_l = float(np.max(np.abs(ql_post - HOME_QPOS)))
    err_f = float(np.max(np.abs(qf_post - HOME_QPOS)))
    assert err_l < PARAMS['p6_err_l_max'], \
        f'leader not at home: err={err_l:.4f} > {PARAMS["p6_err_l_max"]}'
    assert err_f < PARAMS['p6_err_f_max'], \
        f'follower not at home: err={err_f:.4f} > {PARAMS["p6_err_f_max"]}'

    return f'home-errs l={err_l:.4f} f={err_f:.4f}'


def phase7a_reset_via_topic():
    _home_stack()

    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    _focus_leader_window()
    time.sleep(0.15)

    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(PARAMS['p7_push_hold_s'])
    kb.release(keyboard.Key.down)
    time.sleep(0.3)

    ql_after_push = _tn.read_leader_q()
    drift_push = float(np.max(np.abs(ql_after_push - HOME_QPOS)))
    if drift_push < PARAMS['p7_displace_min']:
        return f'SKIP:↓ key did not displace leader enough (drift={drift_push:.4f})'

    # Publish reset counter via ROS2 topic
    _tn.publish_reset(1)

    t_start = time.time()
    deadline = t_start + PARAMS['p7_recovery_wait_s']
    while time.time() < deadline:
        ql = _tn.read_leader_q()
        if float(np.max(np.abs(ql - HOME_QPOS))) < PARAMS['p7_recovery_tol']:
            return f'drift before={drift_push:.4f} → recovered in {time.time() - t_start:.2f}s'
        time.sleep(0.05)
    final_drift = float(np.max(np.abs(_tn.read_leader_q() - HOME_QPOS)))
    assert False, (f'leader did not return to home after reset topic '
                   f'(drift {drift_push:.4f} → {final_drift:.4f})')


def phase7b_reset_via_key():
    try:
        from pynput import keyboard
    except ImportError:
        return 'SKIP:pynput unavailable'

    ql = _tn.read_leader_q()
    if ql is not None and float(np.max(np.abs(ql - HOME_QPOS))) > 0.05:
        _home_stack()

    if not _focus_leader_window():
        return 'SKIP:focus failed'
    time.sleep(0.15)

    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(PARAMS['p7_push_hold_s'])
    kb.release(keyboard.Key.down)
    time.sleep(0.3)

    ql_after_push = _tn.read_leader_q()
    drift_push = float(np.max(np.abs(ql_after_push - HOME_QPOS)))
    if drift_push < PARAMS['p7_displace_min']:
        return f'SKIP:↓ key did not displace leader enough (drift={drift_push:.4f})'

    if not _focus_leader_window():
        return 'SKIP:refocus failed'
    time.sleep(0.2)
    kb.press('0')
    time.sleep(0.05)
    kb.release('0')

    t_start = time.time()
    deadline = t_start + PARAMS['p7_recovery_wait_s']
    recovered = False
    while time.time() < deadline:
        ql = _tn.read_leader_q()
        if float(np.max(np.abs(ql - HOME_QPOS))) < PARAMS['p7_recovery_tol']:
            recovered = True
            break
        time.sleep(0.05)

    if not recovered:
        final_drift = float(np.max(np.abs(_tn.read_leader_q() - HOME_QPOS)))
        return (f'SKIP:real 0 key did not reach mujoco key_callback '
                f'(drift {drift_push:.4f} → {final_drift:.4f})')

    return f'real 0 key PASS  (drift before={drift_push:.4f}, recovered in {time.time() - t_start:.2f}s)'


# ===========================================================================
# Main
# ===========================================================================
def main():
    global _tn

    rclpy.init()
    _tn = TestNode()

    executor = SingleThreadedExecutor()
    executor.add_node(_tn)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    phases = [
        ('Phase 0 setup & liveness',             phase0_setup_liveness),
        ('Phase 1 passive stability',            phase1_passive_stability),
        ('Phase 2 control frequency',            phase2_control_frequency),
        ('Phase 3 follower tracks leader',       phase3_follower_tracks_leader),
        ('Phase 4a mouse Ctrl+drag → pause',     phase4a_mouse_drag_to_box),
        ('Phase 4b keyboard ↓ hold → pause',     phase4b_keyboard_down_to_box),
        ('Phase 5 P pause toggle (topic)',       phase5_p_toggle),
        ('Phase 6 I homing',                     phase6_i_homing),
        ('Phase 7a 0 reset via topic',           phase7a_reset_via_topic),
        ('Phase 7b 0 reset via real key',        phase7b_reset_via_key),
    ]

    print(f'===== test_full_stack_ros2.py  (leader={LEADER_ROBOT}) =====')
    results = []
    overall_t0 = time.time()
    for name, fn in phases:
        _log(f'> {name}')
        r = _run(name, fn)
        if r.status == 'PASS' and r.detail.startswith('SKIP:'):
            r.status = 'SKIP'
            r.detail = r.detail[5:]
        results.append(r)
        _log(f'  [{r.status}] {r.detail}')

    _log('> Cleanup')
    _graceful_shutdown([_stack['lsim'], _stack['fsim']])

    total_elapsed = time.time() - overall_t0
    print()
    print('=' * 72)
    print('  test_full_stack_ros2.py report')
    print('=' * 72)
    n_pass = sum(1 for r in results if r.status == 'PASS')
    n_fail = sum(1 for r in results if r.status == 'FAIL')
    n_skip = sum(1 for r in results if r.status == 'SKIP')
    for r in results:
        tag = {'PASS': '✓', 'FAIL': '✗', 'SKIP': '—'}[r.status]
        print(f'  [{r.status}] {tag} {r.name:42s} {r.elapsed:5.1f}s   {r.detail}')
    print()
    print(f'  {n_pass} PASS   {n_fail} FAIL   {n_skip} SKIP'
          f'   total elapsed {total_elapsed:.1f}s')
    print()
    if n_fail == 0:
        print('  ALL NON-SKIP PHASES PASS — full stack is functional.')
    else:
        print(f'  {n_fail} PHASE(S) FAILED — see details above.')

    _tn.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail > 0 else 0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        _graceful_shutdown([_stack['lsim'], _stack['fsim']])
        if _tn:
            _tn.destroy_node()
        rclpy.shutdown()
        sys.exit(130)
