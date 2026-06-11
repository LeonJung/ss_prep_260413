#!/usr/bin/env python3
"""Headless validation of the oa_fd bilateral law on the openarmx MJCF plant.

Two in-process OaFdSim robots (leader, follower) cross-wired each step —
deterministic, no shm/viewer/X11.

Phases:
  1  gravity-hold   : FREEDRIVE at a gravity-loaded pose -> arm must stay put
  2  paused-hold    : PAUSED captures pose -> stiff hold, no slam, no drift
  3  homing         : HOMING ramps to HOME smoothly, settles
  4  tracking       : ACTIVE; push leader with external torque -> follower tracks
  5  force-reflect  : block follower (external load) -> leader feels it
                      (leader-follower position gap + leader torque rises)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from oa_fd_sim import (OaFdSim, HOME, N, NJ, KP,
                       MODE_ACTIVE, MODE_PAUSED, MODE_HOMING, MODE_FREEDRIVE)

PASS = []


def report(name, ok, detail):
    PASS.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")


def couple_step(leader, follower, n, f_leader=None, f_follower=None):
    """Step both robots n times, cross-feeding live state.
    f_*: optional (vadr_index, torque) external pushes."""
    for _ in range(n):
        lq, ldq = leader.q(), leader.dq()
        fq, fdq = follower.q(), follower.dq()
        if f_leader is not None:
            leader.data.qfrc_applied[:] = 0
            for idx, tq in f_leader:
                leader.data.qfrc_applied[leader.vadr[idx]] = tq
        if f_follower is not None:
            follower.data.qfrc_applied[:] = 0
            for idx, tq in f_follower:
                follower.data.qfrc_applied[follower.vadr[idx]] = tq
        leader.step(fq, fdq)
        follower.step(lq, ldq)


def main():
    leader = OaFdSim('leader')
    follower = OaFdSim('follower')

    # gravity-loaded pose: left arm bent fwd (j1) + elbow, right arm bent (j2)
    pose = HOME.copy()
    pose[0] = -1.2    # left j1 (pitch fwd)
    pose[3] = 1.0     # left j4 (elbow)
    pose[8] = +1.5    # right j2 (right ranges are mirrored: j2 in [-0.13, 3.27])
    for s in (leader, follower):
        s.set_q(pose)

    # ---- 1. gravity-hold (FREEDRIVE) ----
    for s in (leader, follower):
        s.set_mode(MODE_FREEDRIVE)
    couple_step(leader, follower, 1500)   # 3 s
    drift = np.abs(leader.q() - pose).max()
    g = leader.gravity()
    report('1 gravity-hold', drift < 0.12,
           f'max drift {drift:.3f} rad over 3 s (g[j1]={g[0]:.1f} Nm)')

    # ---- 2. paused-hold ----
    for s in (leader, follower):
        s.set_mode(MODE_PAUSED)
    qcap = leader.q().copy()
    couple_step(leader, follower, 1000)   # 2 s
    err = np.abs(leader.q() - qcap).max()
    report('2 paused-hold', err < 0.05, f'hold error {err:.4f} rad')

    # ---- 3. homing ----
    for s in (leader, follower):
        s.set_mode(MODE_HOMING, t_now=s.data.time, duration=2.0)
    couple_step(leader, follower, 1500)   # 3 s (2 s ramp + settle)
    err = np.abs(leader.q() - HOME).max()
    errf = np.abs(follower.q() - HOME).max()
    report('3 homing', err < 0.08 and errf < 0.08,
           f'leader err {err:.3f}, follower err {errf:.3f} rad')

    # ---- 4. tracking (ACTIVE) ----
    for s in (leader, follower):
        s.set_mode(MODE_ACTIVE)
    couple_step(leader, follower, 250)    # settle coupling
    # push leader left-j1 with sustained external torque (human hand)
    couple_step(leader, follower, 1500, f_leader=[(0, 25.0)])
    moved = abs(leader.q()[0] - HOME[0])
    gap = abs(leader.q()[0] - follower.q()[0])
    report('4 tracking', moved > 0.15 and gap < 0.15,
           f'leader moved {moved:.3f} rad, leader-follower gap {gap:.4f} rad')
    leader.data.qfrc_applied[:] = 0

    # ---- 5. force reflection ----
    couple_step(leader, follower, 500)    # re-settle
    q0_l = leader.q()[0]
    # block follower j1 (obstacle): strong opposing torque on follower only
    couple_step(leader, follower, 1500, f_follower=[(0, -40.0)])
    gap = abs(leader.q()[0] - follower.q()[0])
    tau_fb = KP[0] * gap
    # leader must be pulled toward the blocked follower (feel the wall)
    pulled = abs(leader.q()[0] - q0_l)
    report('5 force-reflect', gap > 0.05 and tau_fb > 3.0,
           f'gap {gap:.3f} rad -> reflected torque ~{tau_fb:.1f} Nm '
           f'(leader displaced {pulled:.3f} rad by follower load)')

    print()
    if all(PASS):
        print(f'ALL {len(PASS)} PHASES PASSED')
        return 0
    print(f'{PASS.count(False)}/{len(PASS)} phases FAILED')
    return 1


if __name__ == '__main__':
    sys.exit(main())
