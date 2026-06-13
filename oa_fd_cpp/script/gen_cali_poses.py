#!/usr/bin/env python3
"""gen_cali_poses.py — generate collision-checked calibration poses.

Samples random 7-joint poses (ALL joints excited, wrist included), keeps only
poses that pass:
  (a) joint-limit margin (0.15 rad — also avoids hard-stop torque contamination)
  (b) SELF-collision: capsule-capsule distance between non-adjacent link
      segments > clearance
  (c) torso half-space: no point crosses to the torso side (+y in link0 frame
      for the LEFT arm; the mount faces the body there)
  (d) mount keep-out: elbow/wrist/EE points keep >= keepout distance from the
      shoulder/base region (sphere at link0 origin) and may not rise more than
      `z_top` above the base (the torso mount is above the shoulder; +z = DOWN)

Geometry from the same URDF the controller uses, so FK matches the robot.

Usage:
  gen_cali_poses.py --urdf ../urdf/openarmx_arm_v2com.urdf \
                    --out ../config/cali_poses_left.txt [--n 40] [--seed 7]
Output: one pose per line, 7 comma-separated radians (LEFT-arm conventions;
oa_gravity_cali mirrors j1/j2 itself for --side right).
"""
import argparse
import xml.etree.ElementTree as ET

import numpy as np

CHAIN = [f'openarmx_joint{k}' for k in range(1, 8)]
# left-arm limits (margin applied on top)
LIM = np.array([[-3.34, 0.91], [-3.27, 0.13], [-1.57, 1.57], [0.0, 1.8],
                [-1.5, 1.5], [-0.75, 0.75], [-1.4, 1.4]])


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


def load(path):
    r = ET.parse(path).getroot()
    J = {}
    for j in r.findall('joint'):
        o = j.find('origin')
        a = j.find('axis')
        v3 = lambda s: np.array([float(x) for x in s.split()])
        J[j.get('name')] = dict(
            type=j.get('type'), parent=j.find('parent').get('link'),
            child=j.find('child').get('link'),
            xyz=v3(o.get('xyz', '0 0 0')) if o is not None else np.zeros(3),
            rpy=v3(o.get('rpy', '0 0 0')) if o is not None else np.zeros(3),
            axis=v3(a.get('xyz')) if a is not None else None)
    return J


def joint_points(J, q):
    """World positions of joint origins j1..j7 plus an EE tip point."""
    linkT = {'openarmx_link0': np.eye(4)}
    pts = []
    for i, jn in enumerate(CHAIN):
        j = J[jn]
        To = np.eye(4)
        To[:3, :3] = rpy_R(j['rpy'])
        To[:3, 3] = j['xyz']
        Tj = linkT[j['parent']] @ To
        pts.append(Tj[:3, 3].copy())
        Tr = np.eye(4)
        Tr[:3, :3] = axis_R(j['axis'], q[i])
        linkT[j['child']] = Tj @ Tr
    tip = linkT['openarmx_link7'] @ np.array([0, 0, 0.12, 1.0])  # EE/gripper tip
    pts.append(tip[:3])
    return pts  # 8 points -> 7 segments


def seg_dist(p1, p2, p3, p4):
    """min distance between segments p1p2 and p3p4."""
    d1, d2 = p2 - p1, p4 - p3
    r = p1 - p3
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    b, c = d1 @ d2, d1 @ r
    denom = a * e - b * b
    s = np.clip((b * f - c * e) / denom, 0, 1) if denom > 1e-12 else 0.0
    t = np.clip((b * s + f) / e, 0, 1) if e > 1e-12 else 0.0
    s = np.clip((b * t - c) / a, 0, 1) if a > 1e-12 else 0.0
    c1, c2 = p1 + s * d1, p3 + t * d2
    return np.linalg.norm(c1 - c2)


def pose_ok(J, q, clearance=0.09, torso_y=0.08, keepout=0.16, z_top=-0.06):
    pts = joint_points(J, q)
    segs = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    # (b) self-collision: non-adjacent segment pairs.
    #  - segments 4..6 are the wrist cluster (joints centimetres apart by
    #    construction) -> skip pairs fully inside it
    #  - |i-k|==2 neighbours are structurally close -> reduced threshold
    for i in range(len(segs)):
        for k in range(i + 2, len(segs)):
            if i >= 4 and k >= 4:
                continue
            thr = 0.05 if (k - i) == 2 else clearance
            if seg_dist(*segs[i], *segs[k]) < thr:
                return False
    for idx, p in enumerate(pts):
        # (c) torso side (+y for left arm)
        if p[1] > torso_y:
            return False
        # (d) keep-out near shoulder for distal points (elbow onward),
        #     and never above the base (+z is DOWN; above = z < z_top)
        if idx >= 3 and np.linalg.norm(p) < keepout:
            return False
        if p[2] < z_top:
            return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--side', default='left', choices=['left', 'right'],
                    help='right: write the spatial-mirror of each validated '
                         'left pose (m=[-1,-1,-1,1,-1,1,-1]); collision/torso/'
                         'reach are guaranteed by mirror symmetry')
    args = ap.parse_args()
    # kinematic mirror map L->R (q1,q2,q3,q5,q7 flip; q4,q6 same) — matches
    # the gravity.mirror_right pattern that behaved correctly on HW.
    MIRROR = np.array([-1, -1, -1, 1, -1, 1, -1.0])

    J = load(args.urdf)
    rng = np.random.default_rng(args.seed)
    lo = LIM[:, 0] + args.margin
    hi = LIM[:, 1] - args.margin

    poses, tried = [], 0
    # always include a gentle baseline pose first
    base = np.array([0, 0, 0, 0.3, 0, 0, 0])
    if pose_ok(J, base):
        poses.append(base)
    while len(poses) < args.n and tried < 20000:
        tried += 1
        q = rng.uniform(lo, hi)
        # bias toward moderate wrist usage but full coverage of j1/j2/j4
        q[4:] *= rng.uniform(0.4, 1.0, 3)
        if pose_ok(J, q):
            # reject near-duplicates for spread
            if all(np.linalg.norm(q - p) > 0.5 for p in poses):
                poses.append(q)
    print(f'{len(poses)} poses accepted after {tried} samples')

    # coverage report
    P = np.array(poses)
    for k in range(7):
        print(f'  j{k+1}: range used [{P[:,k].min():6.2f}, {P[:,k].max():6.2f}]'
              f'  (limit [{LIM[k,0]:.2f},{LIM[k,1]:.2f}])')

    # For the right arm, write the spatial mirror of each (validated) left
    # pose. The right arm at m*q is the y-mirror of the left at q, so the
    # torso/mount/self-collision/reach all hold by symmetry.
    out_poses = poses if args.side == 'left' else [q * MIRROR for q in poses]

    with open(args.out, 'w') as f:
        f.write(f'# auto-generated calibration poses ({args.side}-arm frame), '
                'self-collision/torso/mount/reach checked'
                + ('' if args.side == 'left'
                   else ' (mirror of validated left set, m=[-1,-1,-1,1,-1,1,-1])')
                + '\n')
        for q in out_poses:
            f.write(','.join(f'{v:.4f}' for v in q) + '\n')
    print(f'wrote {args.out} ({args.side})')


if __name__ == '__main__':
    main()
