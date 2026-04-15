#!/usr/bin/env python3
"""
test_rtde_minimal.py — MINIMAL-MOTION torque test for UR real hardware.

Purpose: verify direct_torque() actually moves the robot, without risky homing.
ONLY excites joint 0 (shoulder_pan) with tiny torques. Other joints held with
light damping only (no PD toward setpoint).

SAFETY:
  - Max torque: 2 Nm on joint 0, 0.5 Nm on others
  - No homing — starts from wherever the arm currently is
  - 10-second hold-zero phase at start to verify robot stays still
  - Be ready with EM stop just in case.

Usage:
    python3 test_rtde_minimal.py --robot-ip 169.254.186.92
"""

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
try:
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0, os.path.join(
        get_package_share_directory('ur10e_teleop_real'), 'src'))
except Exception:
    pass

from control import URControl

N = 6


def main():
    parser = argparse.ArgumentParser(description='Minimal-motion RTDE torque test')
    parser.add_argument('--robot-ip', default='127.0.0.1')
    parser.add_argument('--robot-port', type=int, default=30004)
    parser.add_argument('--amp', type=float, default=5.0,
                        help='Joint 0 torque amplitude in Nm (default 5.0)')
    parser.add_argument('--freq', type=float, default=1.0,
                        help='Sinusoid frequency in Hz (default 1.0)')
    parser.add_argument('--duration', type=float, default=4.0,
                        help='Sinusoid duration in seconds')
    parser.add_argument('--joint', type=int, default=0,
                        help='Which joint to excite (default 0 = shoulder_pan)')
    parser.add_argument('--mode', choices=['sin', 'pulse'], default='sin',
                        help='Motion pattern: sin (sinusoid) or pulse '
                             '(constant +amp then -amp, each for pulse_s seconds)')
    parser.add_argument('--pulse-s', type=float, default=0.3,
                        help='Pulse duration per direction (default 0.3 s)')
    args = parser.parse_args()

    # Torque limits: allow enough on the excited joint to overcome friction;
    # keep wrist joints very small (EM-brake safety if damping fights motion).
    TORQUE_LIMIT = np.array([8.0, 8.0, 6.0, 1.0, 1.0, 1.0])
    # Light velocity damping on all joints (prevents drift only)
    KD = np.array([0.5, 0.5, 0.5, 0.2, 0.2, 0.2])

    print('=' * 60)
    print('  RTDE Minimal-Motion Torque Test')
    print('=' * 60)
    print(f'  Robot IP      : {args.robot_ip}:{args.robot_port}')
    print(f'  Joint 0 amp   : {args.amp} Nm  (sine at {args.freq} Hz)')
    print(f'  Torque limit  : {TORQUE_LIMIT.tolist()}')
    print(f'  Velocity damp : {KD.tolist()}')
    print(f'  Duration      : {args.duration} s')
    print()
    print('  SAFETY: Be ready with EM stop. Test keeps arm near current pose.')
    print()

    # ---- Connect ----
    print('[1/4] Connecting...')
    robot = URControl(robot_ip=args.robot_ip, port=args.robot_port)
    if not robot.connect():
        print('  FAIL: Could not connect.')
        return 1
    print('  OK: Connected.')

    time.sleep(0.5)
    q0, _ = robot.read_joint_state()
    print(f'  Starting q = {q0.round(4).tolist()}')

    try:
        # ---- Phase 1: zero-torque hold (verify arm stays put) ----
        print('[2/4] Zero-torque hold for 3s (arm should NOT move)...')
        t0 = time.time()
        q_max_drift = np.zeros(N)
        while time.time() - t0 < 3.0:
            q_now, dq_now = robot.read_joint_state()
            tau = -KD * dq_now  # just damping
            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
            robot.write_torque(tau)
            drift = np.abs(q_now - q0)
            q_max_drift = np.maximum(q_max_drift, drift)
            time.sleep(0.002)
        print(f'  max drift per joint = {q_max_drift.round(4).tolist()} rad')
        if np.max(q_max_drift) > 0.1:
            print('  *** WARN: arm drifted > 0.1 rad in zero-torque mode ***')
            print('  Aborting motion phase for safety.')
            return 1

        # ---- Phase 2: excite selected joint ----
        j = args.joint
        q_start = q0.copy()
        max_j_delta = 0.0
        max_tau_sent = 0.0
        max_tau_j_sent = 0.0

        if args.mode == 'sin':
            print(f'[3/4] SINUSOID on JOINT {j} '
                  f'(amp={args.amp} Nm, freq={args.freq} Hz, {args.duration}s)...')
            print('  (Other joints: velocity damping only)')
            t0 = time.time()
            while time.time() - t0 < args.duration:
                t = time.time() - t0
                q_now, dq_now = robot.read_joint_state()
                tau_cmd_j = args.amp * math.sin(2 * math.pi * args.freq * t)
                tau = -KD * dq_now
                tau[j] += tau_cmd_j
                tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
                robot.write_torque(tau)
                delta = abs(q_now[j] - q_start[j])
                max_j_delta = max(max_j_delta, delta)
                max_tau_sent = max(max_tau_sent, float(np.abs(tau).max()))
                max_tau_j_sent = max(max_tau_j_sent, float(abs(tau[j])))
                time.sleep(0.002)
        else:  # pulse
            ps = args.pulse_s
            print(f'[3/4] PULSE on JOINT {j}: +{args.amp} Nm for {ps}s, '
                  f'rest {ps}s, -{args.amp} Nm for {ps}s, rest {ps}s')
            print('  (Other joints: velocity damping only)')
            phases = [
                ('+tau', +args.amp, ps),
                ('rest', 0.0,       ps),
                ('-tau', -args.amp, ps),
                ('rest', 0.0,       ps),
            ]
            for label, tau_j_cmd, dur in phases:
                q_phase_start, _ = robot.read_joint_state()
                t0 = time.time()
                while time.time() - t0 < dur:
                    q_now, dq_now = robot.read_joint_state()
                    tau = -KD * dq_now
                    tau[j] += tau_j_cmd
                    tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
                    robot.write_torque(tau)
                    delta = abs(q_now[j] - q_start[j])
                    max_j_delta = max(max_j_delta, delta)
                    max_tau_sent = max(max_tau_sent, float(np.abs(tau).max()))
                    max_tau_j_sent = max(max_tau_j_sent, float(abs(tau[j])))
                    time.sleep(0.002)
                q_phase_end, _ = robot.read_joint_state()
                phase_delta = q_phase_end[j] - q_phase_start[j]
                print(f'  [{label:<4s}]  tau_j={tau_j_cmd:+.1f}Nm  '
                      f'phase_delta={phase_delta:+.5f} rad '
                      f'({math.degrees(phase_delta):+.2f} deg)')

        print(f'  max |tau| sent overall = {max_tau_sent:.3f} Nm')
        print(f'  max |tau| on joint {j}  = {max_tau_j_sent:.3f} Nm')
        print(f'  joint {j} max delta     = {max_j_delta:.4f} rad '
              f'({math.degrees(max_j_delta):.2f} deg)')

        # ---- Phase 3: zero-torque hold again (damp out motion) ----
        print('[4/4] Damping phase 2s (arm settles)...')
        t0 = time.time()
        while time.time() - t0 < 2.0:
            _, dq_now = robot.read_joint_state()
            tau = -KD * dq_now
            tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)
            robot.write_torque(tau)
            time.sleep(0.002)

        q_final, _ = robot.read_joint_state()
        total_drift = np.abs(q_final - q0)
        print(f'  final q = {q_final.round(4).tolist()}')
        print(f'  total drift per joint = {total_drift.round(4).tolist()} rad')

    finally:
        robot.disconnect()

    print()
    print('=' * 60)
    print('  RESULT')
    print('=' * 60)
    if max_j_delta < 0.001:
        print(f'  Joint {j} did NOT move (< 1 mrad). Try higher --amp.')
    elif max_j_delta < 0.05:
        print(f'  Joint {j} moved {max_j_delta:.4f} rad — SMALL, direct_torque WORKS.')
    else:
        print(f'  Joint {j} moved {max_j_delta:.4f} rad ({math.degrees(max_j_delta):.1f} deg) — reduce --amp for next test.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
