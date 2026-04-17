#!/usr/bin/env python3
"""
test_rtde_connection.py — Quick RTDE connection + motion test for real UR.

Tests:
  1. RTDE connect/disconnect
  2. Read joint state (actual_q, actual_qd)
  3. Wait for homing to complete (assumes auto_home_on_start=true)
  4. Small sinusoid motion on joint 0 (±0.05 rad, 3 seconds)
  5. Return to home position

Usage:
    python3 test_rtde_connection.py --robot-ip 192.168.1.100 --config real_ur.yaml
    python3 test_rtde_connection.py --robot-ip 127.0.0.1    # with ur_server_dummy
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
try:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(
        get_package_share_directory('ur10e_teleop_real_py'), 'src'))
except Exception:
    pass

from control import URControl

N = 6


def _load_config(path):
    if path and os.path.exists(path):
        with open(path) as f:
            return yaml.safe_load(f)
    return {}


def main():
    parser = argparse.ArgumentParser(description='RTDE connection + motion test')
    parser.add_argument('--robot-ip', default='127.0.0.1')
    parser.add_argument('--robot-port', type=int, default=30004)
    parser.add_argument('--config', default=None)
    args = parser.parse_args()

    cfg = _load_config(args.config)
    home = np.array(cfg.get('leader_home',
                             [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]))
    torque_limit = np.array(cfg.get('torque_limit', [10]*6))

    print('=' * 60)
    print('  RTDE Connection + Motion Test')
    print('=' * 60)
    print(f'  Robot IP    : {args.robot_ip}:{args.robot_port}')
    print(f'  HOME        : {home.round(4).tolist()}')
    print(f'  Torque limit: {torque_limit.tolist()}')
    print()

    # ---- Step 1: Connect ----
    print('[1/6] Connecting...')
    robot = URControl(robot_ip=args.robot_ip, port=args.robot_port)
    ok = robot.connect()
    if not ok:
        print('  FAIL: Could not connect.')
        return 1
    print('  OK: Connected.')

    # ---- Step 2: Read joint state ----
    print('[2/6] Reading joint state...')
    time.sleep(0.5)  # wait for first state packet
    q, dq = robot.read_joint_state()
    print(f'  actual_q  = {q.round(4).tolist()}')
    print(f'  actual_qd = {dq.round(4).tolist()}')
    dist_from_home = np.max(np.abs(q - home))
    print(f'  dist from HOME = {dist_from_home:.4f} rad')
    status = robot.read_status()
    print(f'  robot_mode  = {status["robot_mode"]}  '
          f'safety_mode = {status["safety_mode"]}')
    print(f'  recv_count  = {status["recv_count"]}  '
          f'write_count = {status["write_count"]}')
    if status['robot_mode'] != 7:
        print('  *** WARNING: robot_mode != RUNNING(7) — '
              'torque commands will not move the robot! ***')
        print('  Check: UR pendant → Remote Control enabled? Brakes released?')

    # ---- Step 3: Homing (if far from home) ----
    if dist_from_home > 0.05:
        print('[3/6] Homing (quintic spline, 5s)...')
        q_start = q.copy()
        KP = np.array([100.0, 100.0, 50.0, 25.0, 25.0, 25.0])
        KD = np.array([ 20.0,  20.0, 12.0, 10.0, 10.0, 10.0])
        dur = 5.0
        t0 = time.time()
        while time.time() - t0 < dur:
            t = time.time() - t0
            alpha = min(t / dur, 1.0)
            ease = 10*alpha**3 - 15*alpha**4 + 6*alpha**5
            q_des = (1.0 - ease) * q_start + ease * home
            q_now, dq_now = robot.read_joint_state()
            tau = KP * (q_des - q_now) - KD * dq_now
            tau = np.clip(tau, -torque_limit, torque_limit)
            robot.write_torque(tau)
            time.sleep(0.002)
        # Hold at home for 1s
        for _ in range(500):
            q_now, dq_now = robot.read_joint_state()
            tau = KP * (home - q_now) - KD * dq_now
            tau = np.clip(tau, -torque_limit, torque_limit)
            robot.write_torque(tau)
            time.sleep(0.002)
        q, _ = robot.read_joint_state()
        err = np.max(np.abs(q - home))
        print(f'  Homing error = {err:.4f} rad')
        if err > 0.1:
            print('  WARN: Homing may not have completed. Continuing anyway.')
    else:
        print('[3/6] Already at HOME. Skipping homing.')

    # ---- Step 4: Read state at home ----
    print('[4/6] State at HOME:')
    q, dq = robot.read_joint_state()
    print(f'  q  = {q.round(4).tolist()}')
    print(f'  dq = {dq.round(4).tolist()}')

    # ---- Step 5: Small sinusoid motion (joint 0, ±0.05 rad, 3s) ----
    print('[5/6] Sinusoid motion test (joint 0, ±0.05 rad, 3s)...')
    KP = np.array([100.0, 100.0, 50.0, 25.0, 25.0, 25.0])
    KD = np.array([ 20.0,  20.0, 12.0, 10.0, 10.0, 10.0])
    AMP = 0.05
    FREQ = 0.5
    DUR = 3.0
    max_err = 0.0
    max_tau = 0.0
    q_start = q.copy()
    q_home = q.copy()
    t0 = time.time()
    while time.time() - t0 < DUR:
        t = time.time() - t0
        q_des = q_home.copy()
        q_des[0] += AMP * math.sin(2 * math.pi * FREQ * t)
        q_now, dq_now = robot.read_joint_state()
        tau = KP * (q_des - q_now) - KD * dq_now
        tau = np.clip(tau, -torque_limit, torque_limit)
        robot.write_torque(tau)
        err = abs(q_now[0] - q_des[0])
        max_err = max(max_err, err)
        max_tau = max(max_tau, float(np.abs(tau).max()))
        time.sleep(0.002)
    q_end, _ = robot.read_joint_state()
    joint0_travel = float(abs(q_end[0] - q_start[0]))
    joint_max_delta = float(np.max(np.abs(q_end - q_start)))
    print(f'  max tracking error (joint 0) = {max_err:.4f} rad')
    print(f'  max |tau| sent               = {max_tau:.2f} Nm')
    print(f'  joint 0 delta                = {joint0_travel:.4f} rad')
    print(f'  max joint delta (any)        = {joint_max_delta:.4f} rad')
    if joint_max_delta < 0.005:
        print('  *** WARNING: NO joint moved despite torques being sent! ***')
        print('  Likely causes:')
        print('    1. URScript not running on UR (check pendant log)')
        print('    2. UR not in Remote Control mode')
        print('    3. Robot brakes not released / not RUNNING mode')

    # ---- Step 6: Return to home ----
    print('[6/6] Returning to HOME...')
    for _ in range(1000):
        q_now, dq_now = robot.read_joint_state()
        tau = KP * (q_home - q_now) - KD * dq_now
        tau = np.clip(tau, -torque_limit, torque_limit)
        robot.write_torque(tau)
        time.sleep(0.002)
    q_final, _ = robot.read_joint_state()
    final_err = np.max(np.abs(q_final - q_home))
    print(f'  final error = {final_err:.4f} rad')

    robot.disconnect()

    print()
    print('=' * 60)
    print('  RESULT: ALL STEPS COMPLETED')
    print(f'  Sinusoid tracking error: {max_err:.4f} rad')
    print(f'  Final homing error:      {final_err:.4f} rad')
    print('=' * 60)
    return 0


if __name__ == '__main__':
    sys.exit(main())
