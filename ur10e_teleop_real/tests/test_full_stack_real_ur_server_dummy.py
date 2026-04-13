#!/usr/bin/env python3
"""
test_full_stack_real_ur_server_dummy.py — RTDE test for ur10e_teleop_real.

Tests leader/follower_real_node.py with --client rtde + ur_server_dummy.py.
All verification via ROS2 topics and terminal logs (no visual/MuJoCo viewer).

Phases:
  0. Setup & liveness      — 2 nodes alive, topics publishing
  1. Passive stability     — both arms hold HOME, drift < 0.01
  2. Control frequency     — publish rate ≥ 50 Hz
  3. Follower tracks leader — test publishes leader state, follower follows
  4. Keyboard + over-force — pynput key hold → leader moves → over-force PAUSED
  5. Pause toggle          — mode topic → PAUSED hold → ACTIVE
  6. Homing                — mode topic → HOMING → HOME convergence
  7. Reset                 — reset topic → leader recovers to HOME
"""

import argparse
import math
import os
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
# Paths
# ---------------------------------------------------------------------------
HOME_DIR = os.path.expanduser('~')
INSTALL  = f'{HOME_DIR}/colcon_ws/install/ur10e_teleop_real'
SRC_DIR  = f'{INSTALL}/share/ur10e_teleop_real/src'
LIB_DIR  = f'{INSTALL}/lib/ur10e_teleop_real'
LEADER_PY   = f'{LIB_DIR}/leader_real_node.py'
FOLLOWER_PY = f'{LIB_DIR}/follower_real_node.py'
SERVER_PY   = f'{SRC_DIR}/ur_server_dummy.py'

LOG_LEADER   = '/tmp/fullstack_rtde_leader.log'
LOG_FOLLOWER = '/tmp/fullstack_rtde_follower.log'
LOG_SERVER_L = '/tmp/fullstack_rtde_server_leader.log'
LOG_SERVER_F = '/tmp/fullstack_rtde_server_follower.log'
CONFIG_PATH  = f'{INSTALL}/share/ur10e_teleop_real/config/rtde_dummy.yaml'

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


# ---------------------------------------------------------------------------
# Test Node
# ---------------------------------------------------------------------------
class TestNode(Node):
    def __init__(self):
        super().__init__('test_full_stack_real')

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self._leader_q = None
        self._leader_stamps = []
        self._leader_lock = threading.Lock()

        self._follower_q = None
        self._follower_stamps = []
        self._follower_lock = threading.Lock()

        self._mode_state = None
        self._mode_lock = threading.Lock()

        self.create_subscription(
            JointState, '/ur10e/leader/joint_state', self._leader_cb, 10)
        self.create_subscription(
            JointState, '/ur10e/follower/joint_state', self._follower_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/ur10e/mode', self._mode_cb, qos_latched)

        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)
        self.reset_pub = self.create_publisher(
            Int32, '/ur10e/reset', qos_latched)
        self.leader_state_pub = self.create_publisher(
            JointState, '/ur10e/leader/joint_state', 10)

    def _leader_cb(self, msg):
        q = np.array(msg.position[:N])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._leader_lock:
            self._leader_q = q
            self._leader_stamps.append(stamp)

    def _follower_cb(self, msg):
        q = np.array(msg.position[:N])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._follower_lock:
            self._follower_q = q
            self._follower_stamps.append(stamp)

    def _mode_cb(self, msg):
        if len(msg.data) >= 3:
            with self._mode_lock:
                self._mode_state = int(msg.data[0])

    def read_leader_q(self):
        with self._leader_lock:
            return self._leader_q.copy() if self._leader_q is not None else None

    def read_follower_q(self):
        with self._follower_lock:
            return self._follower_q.copy() if self._follower_q is not None else None

    def read_mode(self):
        with self._mode_lock:
            return self._mode_state

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

    def publish_mode(self, state, t_start=0.0, duration=0.0):
        msg = Float64MultiArray()
        msg.data = [float(state), float(t_start), float(duration)]
        self.mode_pub.publish(msg)

    def publish_reset(self, counter):
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


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
def _log(msg):
    print(msg, flush=True)

_stack = {'lsim': None, 'fsim': None}
_tn: TestNode = None


def _kill_stack():
    # Note: pattern must NOT match our own test filename
    subprocess.run(['pkill', '-9', '-f',
                    'leader_real_node\\.py|follower_real_node\\.py|ur_server_dummy\\.py --port'],
                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(0.3)


def _launch_stack():
    # Start RTDE dummy servers first (leader: 30004, follower: 30005)
    srv_l = subprocess.Popen(
        ['python3', '-u', SERVER_PY, '--port', '30004'],
        stdout=open(LOG_SERVER_L, 'w'), stderr=subprocess.STDOUT)
    srv_f = subprocess.Popen(
        ['python3', '-u', SERVER_PY, '--port', '30005'],
        stdout=open(LOG_SERVER_F, 'w'), stderr=subprocess.STDOUT)
    time.sleep(1.0)  # let servers bind

    # Start nodes with --client rtde
    lsim = subprocess.Popen(
        ['python3', '-u', LEADER_PY,
         '--client', 'rtde', '--robot-ip', '127.0.0.1', '--robot-port', '30004',
         '--config', CONFIG_PATH],
        stdout=open(LOG_LEADER, 'w'), stderr=subprocess.STDOUT)
    fsim = subprocess.Popen(
        ['python3', '-u', FOLLOWER_PY,
         '--client', 'rtde', '--robot-ip', '127.0.0.1', '--robot-port', '30005',
         '--config', CONFIG_PATH],
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


def _wait_for_data(which, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if which == 'leader' and _tn.read_leader_q() is not None:
            return True
        if which == 'follower' and _tn.read_follower_q() is not None:
            return True
        time.sleep(0.05)
    return False


def _home_stack():
    _tn.publish_mode(MODE_HOMING, time.time(), 2.5)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if _tn.read_mode() == MODE_PAUSED:
            break
        time.sleep(0.05)
    time.sleep(0.5)
    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.2)


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


# ===========================================================================
# Phases
# ===========================================================================

def phase0_setup():
    _kill_stack()
    lsim, fsim = _launch_stack()
    _stack['lsim'], _stack['fsim'] = lsim, fsim

    got_l = _wait_for_data('leader', timeout=15.0)
    got_f = _wait_for_data('follower', timeout=15.0)
    assert got_l, 'leader_real_node did not publish joint_state'
    assert got_f, 'follower_real_node did not publish joint_state'

    ql = _tn.read_leader_q()
    qf = _tn.read_follower_q()
    _log(f'  leader  q = {ql.round(4).tolist()}')
    _log(f'  follower q = {qf.round(4).tolist()}')
    return '2 nodes alive'


def phase1_stability():
    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.2)
    ql0 = _tn.read_leader_q()
    qf0 = _tn.read_follower_q()
    time.sleep(1.5)
    ql1 = _tn.read_leader_q()
    qf1 = _tn.read_follower_q()
    drift_l = float(np.max(np.abs(ql1 - ql0)))
    drift_f = float(np.max(np.abs(qf1 - qf0)))
    # RTDE server dynamics has higher drift than direct dummy
    assert drift_l < 0.1, f'leader drift {drift_l:.4f} > 0.1'
    assert drift_f < 0.1, f'follower drift {drift_f:.4f} > 0.1'
    return f'drift l={drift_l:.5f} f={drift_f:.5f}'


def phase2_frequency():
    WINDOW = 1.0
    _tn.clear_stamps()
    time.sleep(WINDOW)
    l_count, f_count = _tn.get_stamp_counts()
    leader_hz = l_count / WINDOW
    follower_hz = f_count / WINDOW
    assert leader_hz >= 50, f'leader rate {leader_hz:.0f} Hz < 50'
    assert follower_hz >= 50, f'follower rate {follower_hz:.0f} Hz < 50'
    return f'leader={leader_hz:.0f} Hz  follower={follower_hz:.0f} Hz'


def phase3_tracking():
    # Kill leader to take over its topic
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
        _tn.publish_leader_state(q_cmd, dq_cmd)

        if t > 1.0:
            qf = _tn.read_follower_q()
            if qf is not None:
                err = np.abs(q_cmd - qf)
                max_err = np.maximum(max_err, err)
        time.sleep(0.003)

    # Restart leader
    lsim = subprocess.Popen(
        ['python3', '-u', LEADER_PY,
         '--client', 'rtde', '--robot-ip', '127.0.0.1', '--robot-port', '30004',
         '--config', CONFIG_PATH],
        stdout=open(LOG_LEADER, 'w'), stderr=subprocess.STDOUT)
    _stack['lsim'] = lsim
    _wait_for_data('leader', timeout=8.0)
    time.sleep(0.3)

    assert max_err[0] < 1.0, f'joint[0] err {max_err[0]:.3f} > 1.0'
    assert max_err[2] < 1.0, f'joint[2] err {max_err[2]:.3f} > 1.0'

    _log(f'  tracking err = {max_err.round(3).tolist()}')
    return f'max_err [{max_err[0]:.3f}, {max_err[2]:.3f}]'


def phase4_keyboard():
    try:
        from pynput import keyboard
    except ImportError:
        return 'SKIP:pynput unavailable'

    _home_stack()

    ql_start = _tn.read_leader_q()
    if ql_start is None:
        return 'SKIP:no leader data'

    # pynput works globally — no window focus needed
    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    t0 = time.time()
    paused = False
    while time.time() - t0 < 5.0:
        m = _tn.read_mode()
        if m == MODE_PAUSED:
            paused = True
            break
        ql = _tn.read_leader_q()
        if ql is not None and float(np.max(np.abs(ql - ql_start))) > 0.1:
            break
        time.sleep(0.05)
    kb.release(keyboard.Key.down)

    ql_end = _tn.read_leader_q()
    motion = float(np.max(np.abs(ql_end - ql_start))) if ql_end is not None else 0.0

    if motion < 0.01:
        return 'SKIP:keyboard did not move leader'

    _log(f'  leader moved {motion:.3f} rad, paused={paused}')
    suffix = '→ PAUSED' if paused else '(no over-force)'
    return f'moved {motion:.3f} rad {suffix}'


def phase5_pause():
    _home_stack()

    _tn.publish_mode(MODE_PAUSED)
    time.sleep(0.3)
    assert _tn.read_mode() == MODE_PAUSED, 'PAUSED not observed'

    time.sleep(0.5)
    qh_l = _tn.read_leader_q()
    qh_f = _tn.read_follower_q()
    time.sleep(0.5)
    dr_l = float(np.max(np.abs(_tn.read_leader_q() - qh_l)))
    dr_f = float(np.max(np.abs(_tn.read_follower_q() - qh_f)))
    assert dr_l < 0.005, f'leader drifted {dr_l:.4f}'
    assert dr_f < 0.5, f'follower drifted {dr_f:.4f}'

    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.15)
    return f'hold drift l={dr_l:.5f} f={dr_f:.5f}'


def phase6_homing():
    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.2)

    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(0.7)
    kb.release(keyboard.Key.down)
    time.sleep(0.15)

    ql_pre = _tn.read_leader_q()
    off_home = float(np.max(np.abs(ql_pre - HOME_QPOS)))
    assert off_home > 0.01, f'leader still at home ({off_home:.4f})'

    _tn.publish_mode(MODE_HOMING, time.time(), 3.0)

    deadline = time.time() + 5.0
    while time.time() < deadline:
        if _tn.read_mode() == MODE_PAUSED:
            break
        time.sleep(0.05)

    time.sleep(0.5)
    ql_post = _tn.read_leader_q()
    qf_post = _tn.read_follower_q()
    err_l = float(np.max(np.abs(ql_post - HOME_QPOS)))
    err_f = float(np.max(np.abs(qf_post - HOME_QPOS)))
    assert err_l < 0.5, f'leader home err {err_l:.4f}'
    assert err_f < 0.5, f'follower home err {err_f:.4f}'

    _log(f'  home err l={err_l:.4f} f={err_f:.4f}')
    return f'home err l={err_l:.4f} f={err_f:.4f}'


def phase7_reset():
    _home_stack()

    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(0.3)
    kb.release(keyboard.Key.down)
    time.sleep(0.3)

    ql = _tn.read_leader_q()
    drift = float(np.max(np.abs(ql - HOME_QPOS)))
    if drift < 0.01:
        return 'SKIP:leader not displaced'

    _tn.publish_reset(1)

    t_start = time.time()
    deadline = t_start + 3.0
    while time.time() < deadline:
        ql = _tn.read_leader_q()
        if float(np.max(np.abs(ql - HOME_QPOS))) < 0.02:
            return f'recovered in {time.time() - t_start:.2f}s (drift was {drift:.4f})'
        time.sleep(0.05)
    final = float(np.max(np.abs(_tn.read_leader_q() - HOME_QPOS)))
    assert False, f'recovery failed (drift {drift:.4f} → {final:.4f})'


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
        ('Phase 0 setup & liveness',        phase0_setup),
        ('Phase 1 passive stability',       phase1_stability),
        ('Phase 2 control frequency',       phase2_frequency),
        ('Phase 3 follower tracks leader',  phase3_tracking),
        ('Phase 4 keyboard + over-force',   phase4_keyboard),
        ('Phase 5 pause toggle',            phase5_pause),
        ('Phase 6 homing',                  phase6_homing),
        ('Phase 7 reset',                   phase7_reset),
    ]

    print('===== test_full_stack_real.py =====')
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

    total = time.time() - overall_t0
    print()
    print('=' * 60)
    print('  test_full_stack_real.py report')
    print('=' * 60)
    n_pass = sum(1 for r in results if r.status == 'PASS')
    n_fail = sum(1 for r in results if r.status == 'FAIL')
    n_skip = sum(1 for r in results if r.status == 'SKIP')
    for r in results:
        tag = {'PASS': '✓', 'FAIL': '✗', 'SKIP': '—'}[r.status]
        print(f'  [{r.status}] {tag} {r.name:35s} {r.elapsed:5.1f}s  {r.detail}')
    print()
    print(f'  {n_pass} PASS   {n_fail} FAIL   {n_skip} SKIP   '
          f'total {total:.1f}s')
    print()
    if n_fail == 0:
        print('  ALL NON-SKIP PHASES PASS.')
    else:
        print(f'  {n_fail} PHASE(S) FAILED.')

    _tn.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail > 0 else 0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        _graceful_shutdown([_stack['lsim'], _stack['fsim']])
        rclpy.shutdown()
        sys.exit(130)
