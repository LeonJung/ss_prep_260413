#!/usr/bin/env python3
"""
calibrate_bilateral.py — interactive bilateral home calibration via ROS2.

Runs ALONGSIDE the teleop nodes (leader_real_node, follower_real_node).
Does NOT open RTDE itself — subscribes to /ur10e/leader/joint_state and
/ur10e/follower/joint_state, and publishes to /ur10e/mode to put both
arms into FREEDRIVE so the user can position them by hand.

Flow:
  1. Pub MODE_FREEDRIVE
  2. N times: user moves both arms to a sync pose, presses Enter, script
     averages ~0.1 s of recent samples and records (leader_q, follower_q).
  3. Pub MODE_PAUSED so arms re-hold.
  4. Print per-joint offset = mean(follower_q - leader_q), residual std,
     and a suggested leader_home / follower_home pair.

Usage:
    # Start teleop first:
    ros2 launch ur10e_teleop_real teleop_real.launch.py
    # In another terminal, after homing completes:
    python3 script/calibrate_bilateral.py
    python3 script/calibrate_bilateral.py --n 5
"""

import argparse
import sys
import threading
import time
from collections import deque

import numpy as np

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

MODE_ACTIVE = 0
MODE_PAUSED = 1
MODE_HOMING = 2
MODE_FREEDRIVE = 3

N = 6
AVG_WINDOW_SEC = 0.1


class Calibrator(Node):
    def __init__(self):
        super().__init__('bilateral_calibrator')
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self._leader_buf = deque(maxlen=500)
        self._follower_buf = deque(maxlen=500)
        self._lock = threading.Lock()

        self.create_subscription(JointState, '/ur10e/leader/joint_state',
                                 self._leader_cb, 10)
        self.create_subscription(JointState, '/ur10e/follower/joint_state',
                                 self._follower_cb, 10)
        self.mode_pub = self.create_publisher(
            Float64MultiArray, '/ur10e/mode', qos_latched)

    def _leader_cb(self, msg):
        q = np.array(msg.position[:N])
        t = time.time()
        with self._lock:
            self._leader_buf.append((t, q))

    def _follower_cb(self, msg):
        q = np.array(msg.position[:N])
        t = time.time()
        with self._lock:
            self._follower_buf.append((t, q))

    def publish_mode(self, mode_int):
        msg = Float64MultiArray()
        msg.data = [float(mode_int), 0.0, 0.0]
        self.mode_pub.publish(msg)

    def wait_for_both_streams(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                l_ok = len(self._leader_buf) > 0
                f_ok = len(self._follower_buf) > 0
            if l_ok and f_ok:
                return True
            time.sleep(0.05)
        return False

    def sample_pair_avg(self, window=AVG_WINDOW_SEC):
        """Average samples received in the last `window` seconds."""
        t_now = time.time()
        with self._lock:
            l_recent = [q for (t, q) in self._leader_buf   if t_now - t <= window]
            f_recent = [q for (t, q) in self._follower_buf if t_now - t <= window]
        if not l_recent or not f_recent:
            return None, None, 0, 0
        ql = np.mean(np.stack(l_recent),   axis=0)
        qf = np.mean(np.stack(f_recent), axis=0)
        return ql, qf, len(l_recent), len(f_recent)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=int, default=5,
                        help='Number of pose pairs to capture (default 5)')
    args = parser.parse_args()

    rclpy.init()
    node = Calibrator()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('=' * 68)
    print('  Bilateral Home Calibration via ROS2')
    print('=' * 68)
    print('  Requires teleop nodes already running.')
    print('  Script will put both arms in FREEDRIVE (tau = gravity-comp only)')
    print('  so you can position them by hand for each pose pair.')
    print()

    print('Waiting for joint_state streams...')
    if not node.wait_for_both_streams(timeout=10.0):
        print('FAIL: never received from both /ur10e/leader/joint_state AND'
              ' /ur10e/follower/joint_state. Is teleop running?')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print('Both streams alive.')
    print()
    print('Publishing MODE_FREEDRIVE...')
    node.publish_mode(MODE_FREEDRIVE)
    time.sleep(0.5)
    print('Both arms should now be in FREEDRIVE (zero-torque, gravity-comp).')

    pairs = []
    try:
        for i in range(args.n):
            print()
            print('-' * 68)
            print(f'  Pose {i+1}/{args.n}: move BOTH arms to a visually-synced'
                  ' pose by hand, then press Enter.')
            print('  Type "q" + Enter to stop early.')
            ans = input('  > ').strip()
            if ans.lower() == 'q':
                print('  stopping early.')
                break

            ql, qf, nl, nf = node.sample_pair_avg()
            if ql is None:
                print('  WARN: no recent samples; retry.')
                continue
            pairs.append((ql, qf))
            diff = qf - ql
            print(f'  leader_q   = [{", ".join(f"{v:+.4f}" for v in ql)}]'
                  f'  ({nl} samples)')
            print(f'  follower_q = [{", ".join(f"{v:+.4f}" for v in qf)}]'
                  f'  ({nf} samples)')
            print(f'  diff       = [{", ".join(f"{v:+.4f}" for v in diff)}]')
    except KeyboardInterrupt:
        print()
        print('  interrupted.')
    finally:
        print()
        print('Publishing MODE_PAUSED...')
        node.publish_mode(MODE_PAUSED)
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()

    if not pairs:
        print('No pairs captured.')
        return 1

    # --- Analysis ---
    print()
    print('=' * 68)
    print(f'  ANALYSIS ({len(pairs)} pairs)')
    print('=' * 68)
    for i, (ql, qf) in enumerate(pairs):
        print(f'  Pose {i+1}:')
        print(f'    leader_q   = [{", ".join(f"{v:+.4f}" for v in ql)}]')
        print(f'    follower_q = [{", ".join(f"{v:+.4f}" for v in qf)}]')

    diffs = np.stack([qf - ql for ql, qf in pairs])
    mean_offset = diffs.mean(axis=0)
    std_offset = diffs.std(axis=0)
    print()
    print('  per-joint (follower_q - leader_q) across all samples:')
    print(f'    mean OFFSET = [{", ".join(f"{v:+.4f}" for v in mean_offset)}]')
    print(f'    std         = [{", ".join(f"{v:.4f}" for v in std_offset)}]')
    print()

    max_std = float(std_offset.max())
    if max_std < 0.01:
        verdict = 'GOOD — offset model consistent across poses'
    elif max_std < 0.05:
        verdict = ('OK — small variation; might need minor MIRROR_SIGN tweaks '
                   'or more samples')
    else:
        verdict = ('POOR — offset NOT constant; some joints may need sign flip '
                   'or scale. Inspect the diffs above.')
    print(f'  fit verdict : {verdict}')

    # Suggest new home values: take pose 0's leader_q as leader_home,
    # follower_home = leader_home + mean_offset
    ql0, qf0 = pairs[0]
    print()
    print('  Suggested config (based on pose 1 as canonical reference):')
    print(f'    leader_home:   [{", ".join(f"{v:+.4f}" for v in ql0)}]')
    print(f'    follower_home: [{", ".join(f"{v:+.4f}" for v in (ql0 + mean_offset))}]')
    print()
    print('  Alternative: use pose 1\'s measured values directly:')
    print(f'    leader_home:   [{", ".join(f"{v:+.4f}" for v in ql0)}]')
    print(f'    follower_home: [{", ".join(f"{v:+.4f}" for v in qf0)}]')
    print()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(130)
