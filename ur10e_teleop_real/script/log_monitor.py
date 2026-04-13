#!/usr/bin/env python3
"""
log_monitor.py — Real-time status monitor during teleop logging.

Prints periodic summary showing ALL 6 joints, tracking error, mode, warnings.
Output is also saved to monitor_output.log by logging.sh.

Usage (standalone):
    python3 log_monitor.py /tmp/teleop_logs_XXXXXX
"""

import os
import sys
import time

LOG_DIR = sys.argv[1] if len(sys.argv) > 1 else '/tmp'
INTERVAL = 3.0  # seconds between status prints
N = 6


def _tail_last_line(filepath):
    try:
        with open(filepath, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            if size == 0:
                return None
            pos = max(0, size - 2048)
            f.seek(pos)
            lines = f.read().decode('utf-8', errors='replace').strip().split('\n')
            return lines[-1] if lines else None
    except Exception:
        return None


def _parse_joint_csv(line):
    """Extract 6 joint positions from JointState CSV line."""
    try:
        parts = line.split(',')
        floats = []
        for p in parts:
            try:
                v = float(p.strip())
                if -10.0 < v < 10.0:
                    floats.append(v)
            except ValueError:
                continue
        if len(floats) >= N:
            return floats[:N]
    except Exception:
        pass
    return None


def _parse_mode_csv(line):
    try:
        parts = line.split(',')
        for p in parts:
            try:
                v = float(p.strip())
                if v in (0.0, 1.0, 2.0):
                    return {0: 'ACTIVE', 1: 'PAUSED', 2: 'HOMING'}[int(v)]
            except ValueError:
                continue
    except Exception:
        pass
    return '??'


def _count_warnings(filepath):
    try:
        with open(filepath) as f:
            content = f.read()
        warns = content.count('WARN') + content.count('ERROR') + content.count('OVER-FORCE')
        total = content.count('\n')
        return warns, total
    except Exception:
        return 0, 0


def _fmt_q(q_list):
    """Format 6 joint values compactly."""
    if q_list is None:
        return '--- no data ---'
    return '[' + ', '.join(f'{v:+.3f}' for v in q_list) + ']'


def _tracking_error(lq, fq):
    """Compute max absolute tracking error."""
    if lq is None or fq is None:
        return None
    import numpy as np
    return float(np.max(np.abs(np.array(lq) - np.array(fq))))


def main():
    print(f'[monitor] Watching {LOG_DIR} (every {INTERVAL}s)')
    print(f'[monitor] All 6 joints shown. Tracking error = max|leader - follower|.')
    print()

    hdr = (f'{"Time":>7s}  {"Mode":>7s}  {"Warn":>5s}  {"TrkErr":>7s}  '
           f'{"Leader q[0:5]":>46s}  {"Follower q[0:5]":>46s}')
    print(hdr)
    print('-' * len(hdr))

    t0 = time.time()
    last_warn_count = 0

    while True:
        try:
            elapsed = time.time() - t0

            lq = _parse_joint_csv(
                _tail_last_line(os.path.join(LOG_DIR, 'leader_state.csv')) or '')
            fq = _parse_joint_csv(
                _tail_last_line(os.path.join(LOG_DIR, 'follower_state.csv')) or '')
            mode_line = _tail_last_line(os.path.join(LOG_DIR, 'mode.csv'))
            mode = _parse_mode_csv(mode_line) if mode_line else '??'

            warns, _ = _count_warnings(os.path.join(LOG_DIR, 'rosout.csv'))
            trk_err = _tracking_error(lq, fq)
            trk_str = f'{trk_err:.4f}' if trk_err is not None else '---'

            # Alert on new warnings
            alert = ''
            if warns > last_warn_count:
                new = warns - last_warn_count
                alert = f'  *** {new} NEW WARNING(S) ***'
                last_warn_count = warns

            print(f'{elapsed:6.0f}s  {mode:>7s}  {warns:>5d}  {trk_str:>7s}  '
                  f'{_fmt_q(lq):>46s}  {_fmt_q(fq):>46s}{alert}',
                  flush=True)

            time.sleep(INTERVAL)
        except KeyboardInterrupt:
            break

    print()
    print(f'[monitor] Stopped. Total warnings: {last_warn_count}')


if __name__ == '__main__':
    main()
