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
    args = ap.parse_args()
    fit_idx = {int(x) for x in args.fit_links.split(',')}

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
    A = np.array(A)
    b = np.array(b)
    print(f'{dropped} near-limit equations dropped, {len(b)} kept')

    # prior = current URDF params
    theta0 = np.zeros(nP)
    for li, ln in enumerate(LINKS):
        m = links[ln]['m']
        theta0[4 * li] = m
        theta0[4 * li + 1:4 * li + 4] = m * links[ln]['com']

    # ridge toward prior (scale-aware: mass entries get larger weight so the
    # fit prefers adjusting COMs unless data demands mass changes).
    # Links NOT in --fit-links are pinned hard to the prior.
    w = np.tile([1.0, 0.3, 0.3, 0.3], len(LINKS)) * args.lam * len(rows)
    for li in range(len(LINKS)):
        if (li + 1) not in fit_idx:
            w[4 * li:4 * li + 4] = 1e6          # pin to prior
        elif not args.fit_mass:
            w[4 * li] = 1e6                     # pin mass; fit COM only
    W = np.diag(w)
    lhs = A.T @ A + W
    rhs = A.T @ b + W @ theta0
    theta = np.linalg.solve(lhs, rhs)

    # physical sanity clamps (protect against unobserved directions drifting)
    for li, ln in enumerate(LINKS):
        m0 = theta0[4 * li]
        theta[4 * li] = np.clip(theta[4 * li], 0.3 * m0, 3.0 * m0)
        m = theta[4 * li]
        com = theta[4 * li + 1:4 * li + 4] / m
        com = np.clip(com, -0.20, 0.20)          # |COM| <= 20 cm from link origin
        theta[4 * li + 1:4 * li + 4] = m * com

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
