#!/usr/bin/env python3
"""
Post-process bench CSVs and emit jitter / loss / RTT tables.

Usage:
    analyze.py FILE.csv [FILE2.csv ...]

Each CSV is expected to be in the format produced by
StatsRecorder (one row per receive event):
    recv_ts_ns,peer_seq,peer_send_ts_ns,
    peer_acked_own_seq,computed_rtt_ns,inferred_loss
"""
import sys
import csv
import statistics
from collections import defaultdict


def load(path):
    rtts_us = []
    loss_total = 0
    rows = 0
    with open(path) as f:
        for row in csv.DictReader(f):
            rows += 1
            try:
                rtt = int(row["computed_rtt_ns"])
                if rtt > 0:
                    rtts_us.append(rtt / 1000.0)
            except (KeyError, ValueError):
                pass
            try:
                loss_total += int(row["inferred_loss"])
            except (KeyError, ValueError):
                pass
    return rtts_us, loss_total, rows


def percentile(xs, p):
    if not xs:
        return float("nan")
    xs_sorted = sorted(xs)
    k = int(p * (len(xs_sorted) - 1))
    return xs_sorted[k]


def report(path):
    rtts, loss, rows = load(path)
    print(f"\n=== {path} ===")
    print(f"rows         : {rows}")
    print(f"rtt samples  : {len(rtts)}")
    print(f"inferred loss: {loss}")
    if rtts:
        print(f"rtt µs p50   : {percentile(rtts, 0.50):8.1f}")
        print(f"rtt µs p90   : {percentile(rtts, 0.90):8.1f}")
        print(f"rtt µs p99   : {percentile(rtts, 0.99):8.1f}")
        print(f"rtt µs p99.9 : {percentile(rtts, 0.999):8.1f}")
        print(f"rtt µs max   : {max(rtts):8.1f}")
        print(f"rtt µs mean  : {statistics.mean(rtts):8.1f}")
        print(f"rtt µs stdev : {statistics.pstdev(rtts):8.1f}")


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    for p in sys.argv[1:]:
        report(p)


if __name__ == "__main__":
    main()
