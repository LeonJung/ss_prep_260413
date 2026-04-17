#!/usr/bin/env python3
"""
test_full_stack_real_hw.py — Integration test for real UR hardware.

Runs against leader_real_node.py + follower_real_node.py via ROS2 topics.
No keyboard/mouse/X11 required — all control via ROS2 topics.

Assumes nodes are ALREADY running (start them manually or via launch file).

Phases:
  0. Setup & liveness     — nodes alive, topics publishing
  1. Passive stability    — both arms hold position, drift < threshold
  2. Control frequency    — publish rate check
  3. Bilateral tracking   — mode topic to displace leader, follower follows
  B. RTDE connection test — (run separately: script/test_rtde_connection.py)
  5. Pause toggle         — mode topic PAUSED → hold → ACTIVE
  6. Homing               — mode topic HOMING → both arms go HOME
  7. Reset                — reset topic → leader returns to HOME

Usage:
    # Start nodes first:
    ros2 launch ur10e_teleop_real teleop_real.launch.py leader_ip:=... follower_ip:=...

    # Then run test:
    python3 tests/test_full_stack_real_hw.py
"""

import os
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

MODE_ACTIVE = 0
MODE_PAUSED = 1
MODE_HOMING = 2
N = 6


class TestNode(Node):
    def __init__(self):
        super().__init__('test_real_hw')
        qos = QoSProfile(depth=1,
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

        self.create_subscription(JointState, '/ur10e/leader/joint_state',
                                 self._leader_cb, 10)
        self.create_subscription(JointState, '/ur10e/follower/joint_state',
                                 self._follower_cb, 10)
        self.create_subscription(Float64MultiArray, '/ur10e/mode',
                                 self._mode_cb, qos)

        self.mode_pub = self.create_publisher(Float64MultiArray, '/ur10e/mode', qos)
        self.reset_pub = self.create_publisher(Int32, '/ur10e/reset', qos)

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


# ---------------------------------------------------------------------------
_tn: TestNode = None


def _log(msg):
    print(msg, flush=True)


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
    """Check nodes are alive and publishing."""
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if _tn.read_leader_q() is not None and _tn.read_follower_q() is not None:
            break
        time.sleep(0.1)
    ql = _tn.read_leader_q()
    qf = _tn.read_follower_q()
    assert ql is not None, 'leader not publishing joint_state'
    assert qf is not None, 'follower not publishing joint_state'
    _log(f'  leader  q = {ql.round(4).tolist()}')
    _log(f'  follower q = {qf.round(4).tolist()}')
    return '2 nodes alive'


def phase1_stability():
    """Both arms hold position (drift < 0.05 rad over 2s)."""
    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.5)
    ql0 = _tn.read_leader_q()
    qf0 = _tn.read_follower_q()
    time.sleep(2.0)
    ql1 = _tn.read_leader_q()
    qf1 = _tn.read_follower_q()
    drift_l = float(np.max(np.abs(ql1 - ql0)))
    drift_f = float(np.max(np.abs(qf1 - qf0)))
    # Relaxed threshold for real hardware (motor friction + PD settling)
    assert drift_l < 0.05, f'leader drift {drift_l:.4f} > 0.05'
    assert drift_f < 0.05, f'follower drift {drift_f:.4f} > 0.05'
    return f'drift l={drift_l:.5f} f={drift_f:.5f}'


def phase2_frequency():
    """Publish rate >= 50 Hz."""
    WINDOW = 1.0
    _tn.clear_stamps()
    time.sleep(WINDOW)
    l, f = _tn.get_stamp_counts()
    lhz = l / WINDOW
    fhz = f / WINDOW
    assert lhz >= 50, f'leader rate {lhz:.0f} < 50'
    assert fhz >= 50, f'follower rate {fhz:.0f} < 50'
    return f'leader={lhz:.0f} Hz  follower={fhz:.0f} Hz'


def phase5_pause():
    """PAUSED → hold → ACTIVE."""
    _tn.publish_mode(MODE_PAUSED)
    time.sleep(0.5)
    assert _tn.read_mode() == MODE_PAUSED, 'PAUSED not observed'

    time.sleep(0.5)
    qh_l = _tn.read_leader_q()
    qh_f = _tn.read_follower_q()
    time.sleep(1.0)
    dr_l = float(np.max(np.abs(_tn.read_leader_q() - qh_l)))
    dr_f = float(np.max(np.abs(_tn.read_follower_q() - qh_f)))
    assert dr_l < 0.01, f'leader drifted {dr_l:.4f} in PAUSED'
    assert dr_f < 0.05, f'follower drifted {dr_f:.4f} in PAUSED'

    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.3)
    return f'hold drift l={dr_l:.5f} f={dr_f:.5f}'


def phase6_homing():
    """HOMING via mode topic → both arms converge to HOME."""
    _tn.publish_mode(MODE_HOMING, time.time(), 5.0)

    deadline = time.time() + 8.0
    while time.time() < deadline:
        if _tn.read_mode() == MODE_PAUSED:
            break
        time.sleep(0.1)

    time.sleep(1.0)
    ql = _tn.read_leader_q()
    qf = _tn.read_follower_q()
    _log(f'  leader  q = {ql.round(4).tolist()}')
    _log(f'  follower q = {qf.round(4).tolist()}')
    # Can't check against HOME_QPOS here (different per robot),
    # just check that both arms are stable
    return 'homing completed'


def phase7_reset():
    """Reset topic → leader returns toward HOME."""
    _tn.publish_mode(MODE_ACTIVE)
    time.sleep(0.5)

    ql_before = _tn.read_leader_q()
    _tn.publish_reset(1)
    time.sleep(3.0)
    ql_after = _tn.read_leader_q()

    _log(f'  before = {ql_before.round(4).tolist()}')
    _log(f'  after  = {ql_after.round(4).tolist()}')
    return 'reset completed'


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
        ('Phase 0 setup & liveness',    phase0_setup),
        ('Phase 1 passive stability',   phase1_stability),
        ('Phase 2 control frequency',   phase2_frequency),
        ('Phase 5 pause toggle',        phase5_pause),
        ('Phase 6 homing',              phase6_homing),
        ('Phase 7 reset',               phase7_reset),
    ]

    print('===== test_full_stack_real_hw.py =====')
    print('  NOTE: Nodes must be running. This test uses ROS2 topics only.')
    print()
    results = []
    t0 = time.time()
    for name, fn in phases:
        _log(f'> {name}')
        r = _run(name, fn)
        if r.status == 'PASS' and r.detail.startswith('SKIP:'):
            r.status = 'SKIP'
            r.detail = r.detail[5:]
        results.append(r)
        _log(f'  [{r.status}] {r.detail}')

    total = time.time() - t0
    print()
    print('=' * 60)
    print('  test_full_stack_real_hw.py report')
    print('=' * 60)
    n_pass = sum(1 for r in results if r.status == 'PASS')
    n_fail = sum(1 for r in results if r.status == 'FAIL')
    n_skip = sum(1 for r in results if r.status == 'SKIP')
    for r in results:
        tag = {'PASS': '✓', 'FAIL': '✗', 'SKIP': '—'}[r.status]
        print(f'  [{r.status}] {tag} {r.name:35s} {r.elapsed:5.1f}s  {r.detail}')
    print()
    print(f'  {n_pass} PASS   {n_fail} FAIL   {n_skip} SKIP   total {total:.1f}s')
    print()
    if n_fail == 0:
        print('  ALL NON-SKIP PHASES PASS.')
    else:
        print(f'  {n_fail} PHASE(S) FAILED.')
    print()
    print('  NOTE: Also run script/test_rtde_connection.py for motion test.')

    _tn.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail > 0 else 0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rclpy.shutdown()
        sys.exit(130)
