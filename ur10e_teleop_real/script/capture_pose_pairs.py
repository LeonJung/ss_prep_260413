#!/usr/bin/env python3
"""
capture_pose_pairs.py — interactive capture of leader/follower joint pairs.

Connects to both robots via RTDE (read-only, no URScript upload).
User physically positions both arms to a sync pose, presses Enter; script
reads current joint_state from both and prints the pair. Repeat N times.

After all captures, prints a summary suitable for pasting back.

Usage:
    python3 capture_pose_pairs.py
    python3 capture_pose_pairs.py --n 5
    python3 capture_pose_pairs.py --leader-ip 169.254.186.94 --follower-ip 169.254.186.92
"""

import argparse
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


def main():
    parser = argparse.ArgumentParser(description='Capture paired poses for bilateral calibration')
    parser.add_argument('--leader-ip',   default='169.254.186.94', help='Leader UR3e IP')
    parser.add_argument('--follower-ip', default='169.254.186.92', help='Follower UR10e IP')
    parser.add_argument('--n', type=int, default=5, help='Number of pose pairs to capture')
    args = parser.parse_args()

    print('=' * 68)
    print('  Bilateral Pose-Pair Capture')
    print('=' * 68)
    print(f'  Leader   (UR3e) : {args.leader_ip}')
    print(f'  Follower (UR10e): {args.follower_ip}')
    print(f'  Target pairs    : {args.n}')
    print()
    print('  1. Move BOTH arms by hand to a visually-synced pose.')
    print('  2. Press Enter to capture.  (Type "q" + Enter to abort.)')
    print('  No URScript is uploaded — robots stay in whatever mode they are.')
    print()

    # Connect (read-only: upload_urscript=False)
    leader = URControl(robot_ip=args.leader_ip, robot_name='ur3e', upload_urscript=False)
    follower = URControl(robot_ip=args.follower_ip, robot_name='ur10e', upload_urscript=False)

    print('[1/2] Connecting leader...')
    if not leader.connect():
        print('  FAIL')
        return 1
    print('[2/2] Connecting follower...')
    if not follower.connect():
        print('  FAIL')
        leader.disconnect()
        return 1

    time.sleep(0.5)

    pairs = []
    try:
        for i in range(args.n):
            print()
            print('-' * 68)
            print(f'  Pose {i+1}/{args.n}: move both arms, then press Enter...')
            ans = input('  > ').strip()
            if ans.lower() == 'q':
                print('  aborted.')
                break

            # Take several samples and average (reduce noise)
            ql_list = []
            qf_list = []
            for _ in range(20):
                ql, _ = leader.read_joint_state()
                qf, _ = follower.read_joint_state()
                ql_list.append(ql)
                qf_list.append(qf)
                time.sleep(0.005)
            ql = np.mean(np.stack(ql_list), axis=0)
            qf = np.mean(np.stack(qf_list), axis=0)

            pairs.append((ql, qf))
            print(f'  leader_q   = [{", ".join(f"{v:+.4f}" for v in ql)}]')
            print(f'  follower_q = [{", ".join(f"{v:+.4f}" for v in qf)}]')
            diff = qf - ql
            print(f'  diff       = [{", ".join(f"{v:+.4f}" for v in diff)}]')
    finally:
        leader.disconnect()
        follower.disconnect()

    if not pairs:
        print('No pairs captured.')
        return 1

    # Summary
    print()
    print('=' * 68)
    print(f'  SUMMARY ({len(pairs)} pairs)')
    print('=' * 68)
    for i, (ql, qf) in enumerate(pairs):
        print()
        print(f'Pose {i+1}:')
        print(f'  leader_q   = [{", ".join(f"{v:+.4f}" for v in ql)}]')
        print(f'  follower_q = [{", ".join(f"{v:+.4f}" for v in qf)}]')

    print()
    print('--- per-joint follower_q - leader_q across all samples ---')
    diffs = np.stack([qf - ql for ql, qf in pairs])
    mean_offset = diffs.mean(axis=0)
    std_offset = diffs.std(axis=0)
    print(f'  mean  OFFSET       = [{", ".join(f"{v:+.4f}" for v in mean_offset)}]')
    print(f'  std   (consistency) = [{", ".join(f"{v:.4f}" for v in std_offset)}]')
    print()
    if std_offset.max() < 0.01:
        print('  -> GOOD fit: offset model consistent across poses.')
    elif std_offset.max() < 0.05:
        print('  -> OK fit: small variation, may need minor MIRROR_SIGN tweaks.')
    else:
        print('  -> POOR fit: some joints may need sign flip or scale; share data for analysis.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
