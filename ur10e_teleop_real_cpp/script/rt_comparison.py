#!/usr/bin/env python3
"""
rt_comparison.py — run jitter_benchmark twice (non-RT vs RT) and report.

Sources:
  jitter_benchmark writes per-cycle `cycle,now_ns,dt_us` CSV to stdout,
  summary lines to stderr.

Typical usage:
    # after colcon build + source setup.bash
    python3 script/rt_comparison.py --duration 30

Flags:
    --duration SEC       per-run length (default 30 s)
    --period-us US       target period  (default 2000 → 500 Hz)
    --binary PATH        explicit path to jitter_benchmark
    --save-csv DIR       write nonrt.csv and rt.csv under DIR
    --plot               show jitter histogram (needs matplotlib)
"""

import argparse
import os
import shutil
import subprocess
import sys


# ----------------------------------------------------------------------------
def find_benchmark():
    """Locate jitter_benchmark via ros2 pkg prefix, then colcon install path."""
    candidates = []
    try:
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'ur10e_teleop_real_cpp'],
            stderr=subprocess.DEVNULL, text=True).strip()
        candidates.append(os.path.join(prefix, 'lib', 'ur10e_teleop_real_cpp',
                                        'jitter_benchmark'))
    except Exception:
        pass
    candidates += [
        os.path.expanduser('~/colcon_ws/install/ur10e_teleop_real_cpp/'
                            'lib/ur10e_teleop_real_cpp/jitter_benchmark'),
        shutil.which('jitter_benchmark') or '',
    ]
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return None


# ----------------------------------------------------------------------------
def run_bench(binary, rt_mode, duration, period_us, rt_cpu=None, rt_priority=None):
    cmd = [binary,
           '--rt-mode', 'true' if rt_mode else 'false',
           '--duration', str(duration),
           '--period-us', str(period_us)]
    if rt_cpu is not None:
        cmd += ['--rt-cpu', str(rt_cpu)]
    if rt_priority is not None:
        cmd += ['--rt-priority', str(rt_priority)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.stdout, proc.stderr, proc.returncode


def parse_dts(stdout):
    dts = []
    for line in stdout.splitlines():
        if not line or line.startswith('#') or line.startswith('cycle,'):
            continue
        parts = line.split(',')
        if len(parts) == 3:
            try:
                dts.append(int(parts[2]))
            except ValueError:
                pass
    return dts


def percentile(sorted_list, p):
    if not sorted_list:
        return None
    idx = int(round(p * (len(sorted_list) - 1)))
    return sorted_list[max(0, min(idx, len(sorted_list) - 1))]


def stats(dts, target_us):
    if not dts:
        return None
    s = sorted(dts)
    n = len(s)
    return {
        'n':      n,
        'target': target_us,
        'mean':   sum(s) / n,
        'min':    s[0],
        'p50':    percentile(s, 0.50),
        'p90':    percentile(s, 0.90),
        'p99':    percentile(s, 0.99),
        'p999':   percentile(s, 0.999),
        'max':    s[-1],
    }


def print_stats(label, st):
    if st is None:
        print(f"  {label}: no data")
        return
    print(f"  {label}: n={st['n']}  target={st['target']}us")
    print(f"    mean {st['mean']:7.1f}  min {st['min']:6d}  p50 {st['p50']:6d}"
          f"  p90 {st['p90']:6d}  p99 {st['p99']:6d}  p99.9 {st['p999']:6d}"
          f"  max {st['max']:6d}   [µs]")


# ----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--binary')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--period-us', type=int, default=2000)
    parser.add_argument('--rt-cpu', type=int, default=None,
        help='Pin RT thread to this CPU core (reduces migration jitter). '
             'Applied to both runs for fair comparison.')
    parser.add_argument('--rt-priority', type=int, default=None,
        help='SCHED_FIFO priority for the RT run (default 80)')
    parser.add_argument('--load', action='store_true',
        help='Generate background CPU load during each run (stress-ng or yes) '
             'so the RT benefit is visible.')
    parser.add_argument('--save-csv')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()

    binary = args.binary or find_benchmark()
    if not binary or not os.path.isfile(binary):
        print(f"jitter_benchmark not found. Build first: "
              f"colcon build --packages-select ur10e_teleop_real_cpp")
        return 1

    print('=' * 68)
    print('  RT vs non-RT cyclic-timer comparison')
    print('=' * 68)
    print(f"  binary   : {binary}")
    print(f"  duration : {args.duration:.1f} s per run")
    print(f"  period   : {args.period_us} us  (target {1e6/args.period_us:.0f} Hz)")
    print()

    # Optional background CPU load — surfaces the RT advantage.
    load_proc = None
    def start_load():
        nonlocal load_proc
        if args.load:
            # Spawn N-1 'yes' processes to saturate CPU without stealing the
            # RT-pinned core (if --rt-cpu is used). Simple and always available.
            import multiprocessing
            n = max(1, multiprocessing.cpu_count() - 1)
            print(f"  [load] spawning {n} busy processes")
            load_proc = [subprocess.Popen(['yes'], stdout=subprocess.DEVNULL)
                         for _ in range(n)]
    def stop_load():
        nonlocal load_proc
        if load_proc:
            for p in load_proc:
                try: p.terminate()
                except Exception: pass
            for p in load_proc:
                try: p.wait(timeout=1.0)
                except Exception: p.kill()
            load_proc = None

    print("[1/2] running non-RT ...")
    start_load()
    try:
        out_nr, err_nr, rc_nr = run_bench(binary, False, args.duration,
                                            args.period_us,
                                            rt_cpu=args.rt_cpu,
                                            rt_priority=args.rt_priority)
    finally:
        stop_load()
    dts_nr = parse_dts(out_nr)
    print(f"  captured {len(dts_nr)} cycles")
    for line in err_nr.strip().splitlines():
        print(f"  stderr: {line}")

    print("[2/2] running RT ...")
    start_load()
    try:
        out_rt, err_rt, rc_rt = run_bench(binary, True, args.duration,
                                            args.period_us,
                                            rt_cpu=args.rt_cpu,
                                            rt_priority=args.rt_priority)
    finally:
        stop_load()
    dts_rt = parse_dts(out_rt)
    print(f"  captured {len(dts_rt)} cycles")
    for line in err_rt.strip().splitlines():
        print(f"  stderr: {line}")

    if args.save_csv:
        os.makedirs(args.save_csv, exist_ok=True)
        with open(os.path.join(args.save_csv, 'nonrt.csv'), 'w') as f: f.write(out_nr)
        with open(os.path.join(args.save_csv, 'rt.csv'),   'w') as f: f.write(out_rt)
        print(f"  csv saved → {args.save_csv}/")

    s_nr = stats(dts_nr, args.period_us)
    s_rt = stats(dts_rt, args.period_us)

    print()
    print('=' * 68)
    print('  SUMMARY')
    print('=' * 68)
    print_stats('non-RT', s_nr)
    print()
    print_stats('RT    ', s_rt)
    print()

    if s_nr and s_rt:
        print('  Jitter vs target period (max - target):')
        print(f'    non-RT  : {s_nr["max"] - s_nr["target"]:+d} µs worst-case')
        print(f'    RT      : {s_rt["max"] - s_rt["target"]:+d} µs worst-case')
        if s_rt['max'] > 0:
            factor = (s_nr['max'] - s_nr['target']) / max(1, s_rt['max'] - s_rt['target'])
            print(f'    → RT is {factor:.1f}x tighter worst-case')

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
            for ax, dts, label in [(axes[0], dts_nr, 'non-RT'),
                                    (axes[1], dts_rt, 'RT')]:
                ax.hist(dts, bins=200, color='steelblue', edgecolor='none')
                ax.axvline(args.period_us, color='red', linestyle='--',
                           label=f'target {args.period_us}us')
                ax.set_title(label)
                ax.set_xlabel('cycle dt (µs)')
                ax.set_yscale('log')
                ax.legend()
            axes[0].set_ylabel('count (log)')
            fig.suptitle(
                f'rt_comparison  duration={args.duration}s  period={args.period_us}us')
            plt.tight_layout()
            plt.show()
        except ImportError:
            print('  (matplotlib not installed; skipping --plot)')

    return 0 if rc_nr == 0 and rc_rt == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
