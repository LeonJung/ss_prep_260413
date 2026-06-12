#!/usr/bin/env python3
"""fit_gravity.py — solve per-link mass & COM corrections from oa_gravity_cali CSV.

Static gravity torque is LINEAR in the link first-moment parameters:
    tau_k(q) = sum_i  a_k(q) . [ (p_i(q) - p_k(q)) x g ] * m_i
             + sum_i  a_k(q) . [ (R_i(q) (m_i c_i)) x g_x ]      (per axis)
so with measurements {q, tau_meas} we solve  A theta = tau  for
theta = [m_1, m_1*c_1(3), ..., m_7, m_7*c_7(3)]  (28 params), ridge-regularized
toward the current URDF values (prior) so a modest pose set stays well-posed.

Usage:
  fit_gravity.py --csv /tmp/gravity_cali.csv --urdf openarmx_arm_v2mass.urdf \
                 --out openarmx_arm_cali.urdf [--gz 9.81] [--lam 0.05]

Optionally ingest a friction-sweep CSV (oa_friction_cali output) as EXTRA
gravity measurements via --friction-csv: averaging the +v and -v passes at the
same angle cancels the symmetric friction, leaving gravity (plus the small Fo
asymmetry, <~0.2 Nm, ignored). Sweeps provide hundreds of points along whole
joint trajectories — far richer excitation than the static pose grid
(notably for the q1 curve's amplitude/phase).
Assumption: during a joint's sweep the OTHER joints sit at the cali base pose
(left: [0,0,0,0.3,0,0,0]; right mirrors j1/j2) held by stiff impedance.
"""
import argparse
import csv
import xml.etree.ElementTree as ET

import numpy as np

CHAIN = [f'openarmx_joint{k}' for k in range(1, 8)]
LINKS = [f'openarmx_link{k}' for k in range(1, 8)]   # links moved by the chain
DOF = 7


def rpy_R(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_R(a, th):
    a = a / np.linalg.norm(a)
    x, y, z = a
    c, s = np.cos(th), np.sin(th)
    C = 1 - c
    return np.array([[c + x*x*C, x*y*C - z*s, x*z*C + y*s],
                     [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
                     [z*x*C - y*s, z*y*C + x*s, c + z*z*C]])


def load_urdf(path):
    t = ET.parse(path)
    root = t.getroot()
    joints, links = {}, {}
    for j in root.findall('joint'):
        o = j.find('origin')
        ax = j.find('axis')
        v3 = lambda s: np.array([float(x) for x in s.split()])
        joints[j.get('name')] = dict(
            parent=j.find('parent').get('link'), child=j.find('child').get('link'),
            xyz=v3(o.get('xyz', '0 0 0')) if o is not None else np.zeros(3),
            rpy=v3(o.get('rpy', '0 0 0')) if o is not None else np.zeros(3),
            axis=v3(ax.get('xyz')) if ax is not None else np.array([0, 0, 1.0]))
    for L in root.findall('link'):
        I = L.find('inertial')
        if I is None:
            continue
        o = I.find('origin')
        links[L.get('name')] = dict(
            m=float(I.find('mass').get('value')),
            com=np.array([float(x) for x in
                          (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]))
    return t, joints, links


def friction_csv_to_static(path, side='left', bin_width=0.04, min_each=3):
    """Convert friction-sweep samples into static-equivalent measurements.

    Returns list of (q7, k, tau) — full joint vector, joint index (0-based),
    gravity torque — one per q-bin that has BOTH +v and -v samples.
    """
    base = np.array([0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0])
    if side == 'right':
        base[0], base[1] = -base[0], -base[1]
    # per joint: bin -> ([tau+...], [tau-...], [q...])
    bins = {}
    for r in csv.DictReader(open(path)):
        j = int(r['joint']) - 1
        q = float(r['q'])
        v = float(r['dq'])
        t = float(r['tau_meas'])
        if abs(v) < 0.03:
            continue                      # ~stationary: direction ambiguous
        key = (j, int(np.floor(q / bin_width)))
        e = bins.setdefault(key, ([], [], []))
        e[0 if v > 0 else 1].append(t)
        e[2].append(q)
    out = []
    for (j, _b), (tp, tm, qq) in bins.items():
        if len(tp) < min_each or len(tm) < min_each:
            continue                      # need both directions to cancel friction
        tau = 0.5 * (np.mean(tp) + np.mean(tm))
        q7 = base.copy()
        q7[j] = np.mean(qq)
        out.append((q7, j, tau))
    return out


def fk(joints, q):
    """Return per-chain-joint (world pos, world axis) and per-link (R, p)."""
    linkT = {'openarmx_link0': np.eye(4)}
    jpos, jax = [], []
    for i, jn in enumerate(CHAIN):
        J = joints[jn]
        To = np.eye(4)
        To[:3, :3] = rpy_R(J['rpy'])
        To[:3, 3] = J['xyz']
        Tj = linkT[J['parent']] @ To
        jpos.append(Tj[:3, 3].copy())
        jax.append(Tj[:3, :3] @ J['axis'])
        Tr = np.eye(4)
        Tr[:3, :3] = axis_R(J['axis'], q[i])
        linkT[J['child']] = Tj @ Tr
    return jpos, jax, linkT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--gz', type=float, default=9.81)
    ap.add_argument('--lam', type=float, default=0.05,
                    help='ridge weight toward URDF prior')
    ap.add_argument('--fit-links', default='1,2,3,4',
                    help='comma list of link indices to fit; others stay at the '
                         'URDF prior. Only fit links the pose set actually '
                         'excites! (default poses never move j5-j7 -> fitting '
                         'links 5-7 lets them drift to garbage = wrist runaway)')
    ap.add_argument('--side', default='left', choices=['left', 'right'])
    ap.add_argument('--fit-mass', action='store_true',
                    help='also fit link masses. Default OFF: mass and COM are '
                         'poorly separable from static torques (the fit slides '
                         'to small-mass/far-COM and hits the clamps); masses '
                         'are pinned to the URDF prior (enactic values, '
                         'independently corroborated) and only COMs are fit.')
    ap.add_argument('--friction-csv', default='',
                    help='oa_friction_cali CSV; +v/-v passes averaged per '
                         'q-bin into extra gravity measurements (see header)')
    ap.add_argument('--friction-weight', type=float, default=0.3,
                    help='weight of each sweep-derived equation relative to a '
                         'static one (sweeps contribute hundreds of points; '
                         'keep them from drowning out the multi-joint poses)')
    ap.add_argument('--fit-mass-links', default='',
                    help='comma list of links whose MASS is also fit even with '
                         'masses otherwise pinned (e.g. 7 — the hand/gripper '
                         'mass has no reliable prior and its error shows up as '
                         'q1<->q4 cross-coupling).')
    ap.add_argument('--drop-joints', default='',
                    help='comma list of joint indices whose MEASUREMENT '
                         'equations are excluded (e.g. 6,7 — wrist torque '
                         'telemetry reads ~8Nm static, physically impossible '
                         'for <2Nm wrist gravity: telemetry/differential '
                         'artifact, poisons the fit)')
    args = ap.parse_args()
    fit_idx = {int(x) for x in args.fit_links.split(',')}
    drop_j = ({int(x) - 1 for x in args.drop_joints.split(',')}
              if args.drop_joints else set())
    fit_mass_idx = ({int(x) for x in args.fit_mass_links.split(',')}
                    if args.fit_mass_links else set())

    # joint limits (left arm; right mirrors j1/j2). Equations for a joint
    # measured NEAR ITS LIMIT are dropped: at a hard stop the motor torque
    # contains the contact force, not gravity (e.g. j4=0 is the elbow stop —
    # half the default poses sit on it and poisoned the j4 column).
    LIM_L = [(-3.34, 0.91), (-3.27, 0.13), (-1.57, 1.57), (0.0, 1.8),
             (-1.5, 1.5), (-0.75, 0.75), (-1.4, 1.4)]
    if args.side == 'right':
        LIM = [(-hi, -lo) for lo, hi in LIM_L[:2]] + LIM_L[2:]
    else:
        LIM = LIM_L
    MARGIN = 0.08

    tree, joints, links = load_urdf(args.urdf)
    gvec = np.array([0.0, 0.0, args.gz])
    # KDL applied-FF convention measured on HW = NEGATIVE of raw cross-product sum
    SIGN = -1.0

    rows = list(csv.DictReader(open(args.csv)))
    qs = [np.array([float(r[f'q{k}']) for k in range(1, 8)]) for r in rows]
    taus = [np.array([float(r[f'tau_meas{k}']) for k in range(1, 8)]) for r in rows]
    print(f'{len(rows)} measurements')

    # ---- build linear system: tau_k = SIGN * sum_i a_k.[(p_i + R_i c_i - p_k) x m_i g]
    #      params per link i: m_i (1) + first moment h_i = m_i c_i (3)
    nP = 4 * len(LINKS)
    A, b, meta = [], [], []
    dropped = 0
    for q, tau in zip(qs, taus):
        jpos, jax, linkT = fk(joints, q)
        for k in range(DOF):
            if k in drop_j:
                continue          # excluded joint telemetry (see --drop-joints)
            lo, hi = LIM[k]
            if q[k] < lo + MARGIN or q[k] > hi - MARGIN:
                dropped += 1
                continue          # joint resting on its hard stop -> torque ≠ gravity
            meta.append((q, k))
            row = np.zeros(nP)
            for li, ln in enumerate(LINKS):
                T = linkT[ln]
                Ri, pi = T[:3, :3], T[:3, 3]
                # d tau_k / d m_i  (with c at link origin)
                row[4 * li] = SIGN * jax[k].dot(np.cross(pi - jpos[k], gvec))
                # d tau_k / d h_i (h in link frame -> world: Ri h)
                for ax3 in range(3):
                    row[4 * li + 1 + ax3] = SIGN * jax[k].dot(
                        np.cross(Ri[:, ax3], gvec))
            A.append(row)
            b.append(tau[k])
    n_static = len(b)

    # ---- extra equations from friction sweeps (friction cancelled) ----
    # The +v/-v average cancels SYMMETRIC friction but not the Fo asymmetry,
    # which would otherwise bias the COMs. Add one bias unknown per swept
    # joint (columns nP..nP+6) that only sweep equations see; it absorbs
    # Fo + any constant torque-sensor offset.
    nB = DOF if args.friction_csv else 0
    A = [np.concatenate([row, np.zeros(nB)]) for row in A]
    if args.friction_csv:
        pts = friction_csv_to_static(args.friction_csv, side=args.side)
        sw = np.sqrt(args.friction_weight)
        added = 0
        for q, k, tau in pts:
            if k in drop_j:
                continue
            lo, hi = LIM[k]
            if q[k] < lo + MARGIN or q[k] > hi - MARGIN:
                continue
            jpos, jax, linkT = fk(joints, q)
            row = np.zeros(nP + nB)
            for li, ln in enumerate(LINKS):
                T = linkT[ln]
                Ri, pi = T[:3, :3], T[:3, 3]
                row[4 * li] = SIGN * jax[k].dot(np.cross(pi - jpos[k], gvec))
                for ax3 in range(3):
                    row[4 * li + 1 + ax3] = SIGN * jax[k].dot(
                        np.cross(Ri[:, ax3], gvec))
            row[nP + k] = 1.0             # per-joint sweep bias (Fo etc.)
            A.append(row * sw)
            b.append(tau * sw)
            meta.append((q, k))
            added += 1
        print(f'friction sweeps: {added} bin-averaged equations added '
              f'(weight {args.friction_weight}, +{nB} bias unknowns)')

    A = np.array(A)
    b = np.array(b)
    print(f'{dropped} near-limit equations dropped, '
          f'{n_static} static + {len(b) - n_static} sweep kept')

    # prior = current URDF params (+ zero prior for sweep bias unknowns)
    theta0 = np.zeros(nP + nB)
    for li, ln in enumerate(LINKS):
        m = links[ln]['m']
        theta0[4 * li] = m
        theta0[4 * li + 1:4 * li + 4] = m * links[ln]['com']

    # ridge toward prior (scale-aware: mass entries get larger weight so the
    # fit prefers adjusting COMs unless data demands mass changes).
    # Links NOT in --fit-links are pinned hard to the prior.
    w = np.tile([1.0, 0.3, 0.3, 0.3], len(LINKS)) * args.lam * len(rows)
    for li in range(len(LINKS)):
        if (li + 1) not in fit_idx and (li + 1) not in fit_mass_idx:
            w[4 * li:4 * li + 4] = 1e6          # pin to prior
        elif not args.fit_mass and (li + 1) not in fit_mass_idx:
            w[4 * li] = 1e6                     # pin mass; fit COM only
    if nB:
        w = np.concatenate([w, np.full(nB, 0.1 * args.lam * len(rows))])
    W = np.diag(w)
    lhs = A.T @ A + W
    rhs = A.T @ b + W @ theta0
    theta = np.linalg.solve(lhs, rhs)

    # physical sanity clamps (protect against unobserved directions drifting).
    # Links explicitly opened with --fit-mass-links get a wide range — their
    # prior is known-wrong (e.g. leader handle vs V10 hand: much lighter).
    for li, ln in enumerate(LINKS):
        m0 = theta0[4 * li]
        if (li + 1) in fit_mass_idx:
            theta[4 * li] = np.clip(theta[4 * li], 0.05, 5.0 * m0)
        else:
            theta[4 * li] = np.clip(theta[4 * li], 0.3 * m0, 3.0 * m0)
        m = theta[4 * li]
        com = theta[4 * li + 1:4 * li + 4] / m
        com = np.clip(com, -0.20, 0.20)          # |COM| <= 20 cm from link origin
        theta[4 * li + 1:4 * li + 4] = m * com

    if nB:
        bias = theta[nP:]
        print('sweep bias per joint (should ~match friction Fo): ['
              + ' '.join(f'{x:6.3f}' for x in bias) + ']')

    res0 = np.abs(A @ theta0 - b)
    res1 = np.abs(A @ theta - b)
    print(f'residual |tau| Nm:  before mean={res0.mean():.3f} max={res0.max():.3f}'
          f'  ->  after mean={res1.mean():.3f} max={res1.max():.3f}')
    # worst residuals: which pose/joint the model cannot explain
    order = np.argsort(-res1)[:8]
    print('worst residuals (after):')
    for o in order:
        q, jk = meta[o]
        print(f'  j{jk+1} {res1[o]:5.2f} Nm  at q=[' +
              ' '.join(f'{x:5.2f}' for x in q) + ']')

    # ---- write corrected URDF
    for li, ln in enumerate(LINKS):
        m = max(theta[4 * li], 0.05)
        com = theta[4 * li + 1:4 * li + 4] / m
        print(f'  {ln}: m {links[ln]["m"]:.3f} -> {m:.3f},  '
              f'com {np.round(links[ln]["com"], 4)} -> {np.round(com, 4)}')
        for L in tree.getroot().findall('link'):
            if L.get('name') == ln:
                L.find('inertial/mass').set('value', f'{m:.6f}')
                L.find('inertial/origin').set('xyz',
                    f'{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}')
    tree.write(args.out)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
