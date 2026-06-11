#!/usr/bin/env python3
"""fit_friction.py — fit per-joint tanh friction model from oa_friction_cali CSV.

friction sample:  f = tau_meas - tau_gravity_model   at (near-)constant velocity
model:            f(v) = Fc*tanh(k*v) + Fv*v + Fo
For each joint: 1-D search over k; (Fc, Fv, Fo) by linear least squares.
Outputs a ready-to-paste oa_fd.yaml `friction:` block with a SAFETY factor
applied (friction FF overestimation causes chatter around v=0).

Usage: fit_friction.py --csv /tmp/friction_cali.csv [--safety 0.75]
"""
import argparse
import csv
from collections import defaultdict

import numpy as np

KS = [4, 8, 15, 25, 40, 60, 90]


def _nnls2_plus_offset(X, f):
    """LSQ with Fc>=0, Fv>=0 (Fo free): tiny active-set over the 2 bounds."""
    best = None
    for fix_fc in (False, True):
        for fix_fv in (False, True):
            cols = [2]  # Fo always free
            if not fix_fc:
                cols.insert(0, 0)
            if not fix_fv:
                cols.insert(-1, 1)
            coef = np.zeros(3)
            sol, *_ = np.linalg.lstsq(X[:, cols], f, rcond=None)
            coef[cols] = sol
            if coef[0] < 0 or coef[1] < 0:
                continue
            r = np.sqrt(np.mean((X @ coef - f) ** 2))
            if best is None or r < best[0]:
                best = (r, coef)
    return best


def fit_joint(v, f):
    # Physical constraints: Fc >= 0, Fv >= 0 (a negative viscous term would
    # INJECT energy -> instability; negatives in an unconstrained fit are
    # Stribeck / gravity-model-error leakage, not real friction).
    # k weakly identifiable at sweep speeds -> smallest k within 2% of best.
    results = []
    for k in KS:
        X = np.column_stack([np.tanh(k * v), v, np.ones_like(v)])
        out = _nnls2_plus_offset(X, f)
        if out is not None:
            results.append((out[0], k, out[1]))
    rbest = min(r for r, _, _ in results)
    r, k, (Fc, Fv, Fo) = next(t for t in results if t[0] <= rbest * 1.02)
    return dict(Fc=Fc, k=k, Fv=Fv, Fo=Fo, rms=r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--safety', type=float, default=0.75)
    ap.add_argument('--vmin', type=float, default=0.05,
                    help='discard |dq| below this (stiction band, not modelable)')
    args = ap.parse_args()

    data = defaultdict(lambda: ([], []))
    for r in csv.DictReader(open(args.csv)):
        j = int(r['joint'])
        v = float(r['dq'])
        if abs(v) < args.vmin:
            continue
        f = float(r['tau_meas']) - float(r['tau_gravity_model'])
        data[j][0].append(v)
        data[j][1].append(f)

    fits = {}
    print(f'{"joint":>5} {"n":>6} {"Fc":>7} {"k":>4} {"Fv":>7} {"Fo":>7} {"rms":>6}')
    for j in range(1, 8):
        v, f = np.array(data[j][0]), np.array(data[j][1])
        if len(v) < 50:
            print(f'{j:>5} {len(v):>6}  -- not enough samples, leaving zeros')
            fits[j] = dict(Fc=0, k=30, Fv=0, Fo=0, rms=0)
            continue
        fit = fit_joint(v, f)
        fits[j] = fit
        print(f'{j:>5} {len(v):>6} {fit["Fc"]:7.3f} {fit["k"]:>4} '
              f'{fit["Fv"]:7.3f} {fit["Fo"]:7.3f} {fit["rms"]:6.3f}')

    # Fo soaks up gravity-model bias, not just friction asymmetry — cap it
    # so it cannot fight the gravity calibration. Anything bigger is reported.
    FO_CAP = 0.25
    for j in range(1, 8):
        if abs(fits[j]['Fo']) > FO_CAP:
            print(f'  ! j{j}: Fo={fits[j]["Fo"]:.3f} capped to ±{FO_CAP} '
                  f'(likely gravity-model bias, not friction)')
            fits[j]['Fo'] = float(np.clip(fits[j]['Fo'], -FO_CAP, FO_CAP))

    s = args.safety
    print(f'\n# paste into oa_fd.yaml (safety x{s} applied to Fc/Fv/Fo):')
    print('friction:')
    for key, fmt in [('Fc', '{:.3f}'), ('k', '{:.0f}'), ('Fv', '{:.3f}'),
                     ('Fo', '{:.3f}')]:
        vals = []
        for j in range(1, 8):
            val = fits[j][key]
            if key != 'k':
                val *= s
            vals.append(fmt.format(val))
        print(f'  {key}: [{", ".join(vals)}]')


if __name__ == '__main__':
    main()
