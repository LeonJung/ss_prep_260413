#!/usr/bin/env python3
"""oa_fd_sim — MuJoCo validation of the oa_fd bilateral law (OpenArm A2 bimanual).

Two robots (leader, follower), each the openarmx bimanual model (2 arms x 7
joints). Control law per joint — identical in shape to oa_fd_cpp:

    tau = Kp*(q_ref - q) + Kd*(dq_ref - dq) + g(q)            [+ friction FF]

    ACTIVE   : q_ref/dq_ref = peer's live state (cross-coupled)
    PAUSED   : q_ref = pose captured at PAUSED entry, dq_ref = 0
    HOMING   : q_ref = quintic ramp captured->home, dq_ref = 0
    FREEDRIVE: Kp=Kd=0 -> tau = g(q) only

g(q) here = MuJoCo's own bias force (qfrc_bias at qvel≈0 includes exact
gravity), i.e. a PERFECT model — this isolates the control-law question from
model error. Plant friction (joint damping/frictionloss from the openarmx
MJCF) is simulated by MuJoCo; friction FF is left at 0 in v1.

Run modes:
  - class OaFdSim: in-process core (used by tests; two instances cross-wired)
  - __main__ --role leader|follower: viewer process, state exchanged via /dev/shm
"""
import argparse
import mmap
import os
import struct
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_XML = os.path.join(HERE, '..', 'xml', 'oa_bimanual.xml')

ARMS = ('left', 'right')
NJ = 7                       # joints per arm
N = NJ * len(ARMS)           # 14 controlled joints per robot

MODE_ACTIVE, MODE_PAUSED, MODE_HOMING, MODE_FREEDRIVE = 0, 1, 2, 3

# Gains: same shape as oa_fd_cpp defaults (placeholders there; here they are
# validated against the MJCF plant). Per-arm 7 gains, both arms identical.
KP = np.array([120.0, 120.0, 120.0, 120.0, 18.0, 20.0, 16.0] * 2)
KD = np.array([2.0, 2.0, 2.0, 2.0, 0.2, 0.2, 0.2] * 2)
TAU_MAX = np.array([120.0, 120.0, 60.0, 60.0, 14.0, 14.0, 14.0] * 2)
# mid-range home. NOTE the right arm's j1/j2 ranges are MIRRORED
# (left j2 [-3.27,0.13] vs right j2 [-0.13,3.27]) -> per-arm signs.
HOME = np.array([-0.6, -0.8, 0.0, 0.6, 0.0, 0.0, 0.0,     # left
                 +0.6, +0.8, 0.0, 0.6, 0.0, 0.0, 0.0])    # right (mirrored)

JOINT_NAMES = [f'openarmx_{arm}_joint{k}' for arm in ARMS for k in range(1, NJ + 1)]
ACT_NAMES = [f'm_{n}' for n in JOINT_NAMES]


def quintic(a):
    a = min(max(a, 0.0), 1.0)
    return 10 * a**3 - 15 * a**4 + 6 * a**5


class OaFdSim:
    """One robot (leader or follower): model + per-step oa_fd torque law."""

    def __init__(self, role, model_xml=MODEL_XML):
        self.role = role
        self.model = mujoco.MjModel.from_xml_path(model_xml)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep

        self.qadr = np.array([self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINT_NAMES])
        self.vadr = np.array([self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINT_NAMES])
        self.aadr = np.array([mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACT_NAMES])
        assert (self.qadr >= 0).all() and (self.aadr >= 0).all()

        self.mode = MODE_FREEDRIVE
        self._mode_t0 = 0.0
        self._mode_dur = 0.0
        self.q_hold = self.q().copy()
        self.q_home_start = self.q().copy()
        mujoco.mj_forward(self.model, self.data)

    # ---- state ----
    def q(self):
        return self.data.qpos[self.qadr].copy()

    def dq(self):
        return self.data.qvel[self.vadr].copy()

    def set_q(self, q):
        self.data.qpos[self.qadr] = q
        self.data.qvel[self.vadr] = 0
        mujoco.mj_forward(self.model, self.data)

    def gravity(self):
        """Exact bias torque on our joints (gravity when qvel≈0)."""
        return self.data.qfrc_bias[self.vadr].copy()

    # ---- mode ----
    def set_mode(self, mode, t_now=None, duration=3.0):
        if mode == self.mode:
            return
        if mode == MODE_PAUSED:
            self.q_hold = self.q().copy()
        elif mode == MODE_HOMING:
            self.q_home_start = self.q().copy()
            self._mode_t0 = t_now if t_now is not None else self.data.time
            self._mode_dur = duration
        self.mode = mode

    # ---- the law under test ----
    def compute_tau(self, peer_q, peer_dq, coupled):
        q, dq = self.q(), self.dq()
        g = self.gravity()
        if self.mode == MODE_ACTIVE and coupled:
            qr, dqr = peer_q, peer_dq
            kp, kd = KP, KD
        elif self.mode == MODE_PAUSED:
            qr, dqr = self.q_hold, np.zeros(N)
            kp, kd = KP, KD
        elif self.mode == MODE_HOMING:
            a = quintic((self.data.time - self._mode_t0) / max(self._mode_dur, 1e-6))
            qr = self.q_home_start + a * (HOME - self.q_home_start)
            dqr = np.zeros(N)
            kp, kd = KP, KD
        else:  # FREEDRIVE (or ACTIVE without a live peer)
            qr, dqr = q, dq
            kp = kd = np.zeros(N)
        tau = kp * (qr - q) + kd * (dqr - dq) + g
        return np.clip(tau, -TAU_MAX, TAU_MAX)

    def step(self, peer_q=None, peer_dq=None):
        coupled = peer_q is not None
        if not coupled:
            peer_q, peer_dq = self.q(), self.dq()
        tau = self.compute_tau(peer_q, peer_dq, coupled)
        self.data.ctrl[self.aadr] = tau
        mujoco.mj_step(self.model, self.data)
        return tau


# --------------- shm exchange (viewer processes) ---------------

STATE_FMT = f'<{N}d{N}d d'          # q[14], dq[14], stamp
STATE_SZ = struct.calcsize(STATE_FMT)
MODE_FMT = '<ddd'                   # mode, t_start, duration
MODE_SZ = struct.calcsize(MODE_FMT)


def _shm(path, size):
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, size)
    buf = mmap.mmap(fd, size)
    os.close(fd)
    return buf


class Exchange:
    def __init__(self, role):
        self.me = _shm(f'/dev/shm/oa_{role}_state', STATE_SZ)
        other = 'follower' if role == 'leader' else 'leader'
        self.peer = _shm(f'/dev/shm/oa_{other}_state', STATE_SZ)
        self.mode = _shm('/dev/shm/oa_mode', MODE_SZ)

    def write_state(self, q, dq):
        self.me[:STATE_SZ] = struct.pack(STATE_FMT, *q, *dq, time.time())

    def read_peer(self, max_age=0.2):
        vals = struct.unpack(STATE_FMT, self.peer[:STATE_SZ])
        q = np.array(vals[:N])
        dq = np.array(vals[N:2 * N])
        fresh = (time.time() - vals[-1]) < max_age
        return (q, dq) if fresh else (None, None)

    def read_mode(self):
        m, t0, dur = struct.unpack(MODE_FMT, self.mode[:MODE_SZ])
        return int(m), t0, dur

    def write_mode(self, mode, t0=0.0, dur=3.0):
        self.mode[:MODE_SZ] = struct.pack(MODE_FMT, float(mode), t0, dur)


def run_viewer(role):
    import mujoco.viewer
    sim = OaFdSim(role)
    sim.set_q(HOME)
    ex = Exchange(role)
    if role == 'leader':
        ex.write_mode(MODE_FREEDRIVE)

    print(f'[{role}] viewer up.  keys: SPACE=ACTIVE  P=PAUSED  H=HOMING  F=FREEDRIVE')
    print(f'[{role}] Ctrl+drag an arm to act as the human operator.')

    def key_cb(key):
        kmap = {ord(' '): MODE_ACTIVE, ord('P'): MODE_PAUSED,
                ord('H'): MODE_HOMING, ord('F'): MODE_FREEDRIVE}
        if key in kmap:
            ex.write_mode(kmap[key], time.time(), 3.0)
            print(f'[{role}] mode -> {kmap[key]}')

    with mujoco.viewer.launch_passive(sim.model, sim.data,
                                      key_callback=key_cb) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 2.6, -15, 90 if role == 'leader' else -90
        v.cam.lookat[:] = [0, 0, 0.8]
        nsteps = 0
        last_diag = time.time()
        t_next = time.time()
        while v.is_running():
            mode, t0, dur = ex.read_mode()
            # map wall-clock homing start to sim time on transition
            if mode != sim.mode:
                sim.set_mode(mode, t_now=sim.data.time, duration=dur)
            pq, pdq = ex.read_peer()
            sim.step(pq, pdq)
            ex.write_state(sim.q(), sim.dq())
            nsteps += 1
            if nsteps % 16 == 0:
                v.sync()
            if time.time() - last_diag > 1.0:
                last_diag = time.time()
                g = sim.gravity()
                print(f'[{role}] mode={sim.mode} qL1..4=' +
                      ' '.join(f'{x:5.2f}' for x in sim.q()[:4]) +
                      ' gL1..4=' + ' '.join(f'{x:6.2f}' for x in g[:4]))
            t_next += sim.dt
            sl = t_next - time.time()
            if sl > 0:
                time.sleep(sl)
            else:
                t_next = time.time()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--role', choices=['leader', 'follower'], required=True)
    args = ap.parse_args()
    run_viewer(args.role)
