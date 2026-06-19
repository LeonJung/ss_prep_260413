#!/usr/bin/env python3
# TEMP throwaway tool: estimate follower-vs-leader per-joint output ratio.
#
# Feed two single-joint constant-velocity sweeps (oa_friction_cali --joint N)
# of the SAME joint: one leader, one follower (run SEQUENTIALLY, never both at
# once -> CAN bandwidth). For each sweep we bin by q and average the +v and -v
# passes (cancels symmetric friction) -> pure gravity torque g(q).
#
# Reports:
#   (1) direct ratio  k = follower_g(q) / leader_g(q)   [through-origin LS]
#       q7  -> this IS the link7 (end/"link8") mass multiple (same COM).
#   (2) per-arm slope meas/model (tau_gravity_model column) -> tells if each
#       arm matches its own URDF; ratio of these = motor-output scale once the
#       distal mass is matched (use for q6/q4/q2/q1 with link7 already fixed).
# R^2 reported so we know if the gravity signal is big enough to trust.
#
#   python3 tmp_lf_motor_ratio.py --leader L.csv --follower F.csv --joint 7
import argparse, csv, numpy as np

def load(path, joint):
    q, dq, meas, model = [], [], [], []
    for r in csv.DictReader(open(path)):
        if int(float(r['joint'])) != joint:
            continue
        q.append(float(r['q'])); dq.append(float(r['dq']))
        meas.append(float(r['tau_meas']))
        model.append(float(r.get('tau_gravity_model', 'nan') or 'nan'))
    return (np.array(q), np.array(dq), np.array(meas), np.array(model))

def grav_by_bin(q, dq, tau, model, bw=0.04, vmin=0.02, min_each=3):
    """+v/-v average per q-bin -> friction-cancelled gravity & model."""
    keep = np.abs(dq) > vmin
    q, dq, tau, model = q[keep], dq[keep], tau[keep], model[keep]
    out_q, out_g, out_m = [], [], []
    if len(q) == 0:
        return map(np.array, (out_q, out_g, out_m))
    bins = np.round(q / bw).astype(int)
    for b in np.unique(bins):
        m = bins == b
        pos = m & (dq > 0); neg = m & (dq < 0)
        if pos.sum() < min_each or neg.sum() < min_each:
            continue
        g = 0.5 * (tau[pos].mean() + tau[neg].mean())      # friction cancels
        mg = 0.5 * (model[pos].mean() + model[neg].mean())
        out_q.append(q[m].mean()); out_g.append(g); out_m.append(mg)
    return map(np.array, (out_q, out_g, out_m))

def ls_origin(x, y):
    """slope of y = k x through origin + R^2."""
    if len(x) < 2 or np.allclose(x, 0):
        return float('nan'), float('nan')
    k = float(x @ y / (x @ x))
    ss_res = float(((y - k * x) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else float('nan')
    return k, r2

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--leader', required=True)
    ap.add_argument('--follower', required=True)
    ap.add_argument('--joint', type=int, required=True)
    ap.add_argument('--bin', type=float, default=0.04)
    a = ap.parse_args()

    lq, ldq, lm, lmod = load(a.leader, a.joint)
    fq, fdq, fm, fmod = load(a.follower, a.joint)
    LqB, LgB, LmB = list(grav_by_bin(lq, ldq, lm, lmod, a.bin))
    FqB, FgB, FmB = list(grav_by_bin(fq, fdq, fm, fmod, a.bin))
    print(f"joint {a.joint}: leader {len(lq)} samp -> {len(LqB)} bins | "
          f"follower {len(fq)} samp -> {len(FqB)} bins")

    # per-arm meas/model slope (does each arm match its own model column?)
    kL, rL = ls_origin(LmB, LgB)
    kF, rF = ls_origin(FmB, FgB)
    print(f"  meas/model slope  leader k={kL:+.3f} (R2={rL:.3f})   "
          f"follower k={kF:+.3f} (R2={rF:.3f})")
    if np.isfinite(kL) and abs(kL) > 1e-6:
        print(f"    -> follower/leader (motor-output scale) = {kF/kL:+.3f}")

    # direct follower/leader gravity ratio at matched q (interp follower onto leader q)
    if len(LqB) >= 2 and len(FqB) >= 2:
        order = np.argsort(FqB)
        Fi = np.interp(LqB, FqB[order], FgB[order],
                       left=np.nan, right=np.nan)
        ok = np.isfinite(Fi)
        kd, rd = ls_origin(LgB[ok], Fi[ok])
        print(f"  direct gravity ratio follower/leader k={kd:+.3f} (R2={rd:.3f}, "
              f"n={ok.sum()})")
        if a.joint == 7:
            print(f"    -> link7 (end) mass multiple ~= {kd:+.3f}  "
                  f"(x leader link7 0.624972 = {kd*0.624972:.4f})")
    # raw amplitude sanity
    if len(LgB): print(f"  leader   g(q) range [{LgB.min():+.3f},{LgB.max():+.3f}] Nm")
    if len(FgB): print(f"  follower g(q) range [{FgB.min():+.3f},{FgB.max():+.3f}] Nm")

if __name__ == '__main__':
    main()
