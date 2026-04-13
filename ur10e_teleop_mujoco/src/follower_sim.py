#!/usr/bin/env python3
"""
follower_sim.py — MuJoCo simulation for the UR10E Follower arm.

Reads the bilateral coupling command from /dev/shm/ur10e_follower_cmd and
applies a gravity-compensated PD law to track the leader.  Cooperates with
leader_sim.py via the shared /dev/shm/ur10e_mode shm:

  ACTIVE  : normal bilateral tracking
  PAUSED  : hold the q captured at entry (keyboard + bilateral cmd ignored)
  HOMING  : interpolate slowly to HOME_QPOS, then the leader transitions PAUSED

The follower auto-triggers PAUSED if its constraint force on any joint
(qfrc_constraint — e.g. pressing into the box) exceeds OVERFORCE_CONSTRAINT.
"""

import argparse
import os
import sys
import threading
import time
import traceback

import numpy as np
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(__file__))
import struct
from shm_manager import (
    StateWriter, ModeShm, _open_shm, _STATE_FMT, STATE_SIZE,
    MODE_ACTIVE, MODE_PAUSED, MODE_HOMING,
)

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
N = 6

# Home pose — chosen to match the menagerie ur10e.xml keyframe with the
# base body's 180° Z quat.  shoulder_pan=-π/2 points the arm along +Y.
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# Fallback gains used only before bilateral_control starts writing commands.
KP_FALLBACK = np.array([100.0, 100.0, 80.0, 40.0, 40.0, 20.0])
KD_FALLBACK = np.array([ 10.0,  10.0,  8.0,  4.0,  4.0,  2.0])

# Hold / homing PD.
KP_HOLD = np.array([200.0, 200.0, 150.0, 60.0, 60.0, 30.0])
KD_HOLD = np.array([ 20.0,  20.0,  12.0,  6.0,  6.0,  3.0])

# Follower-side over-force threshold (contact / constraint reaction torque).
OVERFORCE_CONSTRAINT = np.array([80.0, 80.0, 50.0, 25.0, 25.0, 15.0])   # Nm

HOMING_DURATION_FALLBACK = 3.0

RENDER_EVERY = 16   # render every N physics steps (≈ 31 Hz)


def _state_name(s: int) -> str:
    return {MODE_ACTIVE: 'ACTIVE', MODE_PAUSED: 'PAUSED',
            MODE_HOMING: 'HOMING'}.get(s, '??')


# ---------------------------------------------------------------------------
# Exchange buffer (lock-protected thread communication)
# ---------------------------------------------------------------------------
class ExchangeBuffer:
    """Lock-protected numpy arrays shared between MuJoCo and control threads."""

    def __init__(self, q0: np.ndarray):
        self.lock = threading.Lock()
        # physics → control
        self.q = q0.copy()
        self.dq = np.zeros(N)
        self.tau_grav = np.zeros(N)
        self.tau_contact = np.zeros(N)
        # control → physics
        self.tau = np.zeros(N)


# ---------------------------------------------------------------------------
# FollowerSim
# ---------------------------------------------------------------------------
class FollowerSim:
    """MuJoCo simulation for the UR10E Follower arm."""

    def __init__(self, model_path: str):
        self._init_mujoco(model_path)
        self._init_shm()
        self._init_threads()

    # ---- Sub-init methods --------------------------------------------------

    def _init_mujoco(self, model_path: str):
        """Load model, create data, resolve joint/actuator IDs."""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:N] = HOME_QPOS
        mujoco.mj_forward(self.model, self.data)

        joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                     for n in JOINT_NAMES]
        self.qpos_ids = [int(self.model.jnt_qposadr[j]) for j in joint_ids]
        self.qvel_ids = [int(self.model.jnt_dofadr[j])  for j in joint_ids]

        act_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                     f'a_{n.replace("_joint","")}')
                   for n in JOINT_NAMES]
        self.act_lo = np.array([self.model.actuator_ctrlrange[a, 0]
                                for a in act_ids])
        self.act_hi = np.array([self.model.actuator_ctrlrange[a, 1]
                                for a in act_ids])

    def _init_shm(self):
        """Set up shared-memory readers/writers."""
        self.state_writer = StateWriter('follower')
        self._peer_shm = _open_shm('/dev/shm/ur10e_leader_state', STATE_SIZE)

        q0 = self.data.qpos[self.qpos_ids].copy()
        self.state_writer.write(q0, np.zeros(N), np.zeros(N), time.time())

        self.mode_shm = ModeShm()

    def _init_threads(self):
        """Create exchange buffer, stop event, and control thread."""
        q0 = self.data.qpos[self.qpos_ids].copy()
        self.buf = ExchangeBuffer(q0)

        # Seed tau_grav so the first physics step has gravity comp
        mujoco.mj_forward(self.model, self.data)
        with self.buf.lock:
            self.buf.tau_grav[:] = self.data.qfrc_bias[self.qvel_ids]

        self.stop_event = threading.Event()
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name='follower-control',
        )

    # ---- Control thread ----------------------------------------------------

    def _control_loop(self):
        """Control thread: SHM I/O + mode state machine + PD control + over-force."""
        prev_state = MODE_ACTIVE
        q_hold = HOME_QPOS.copy()
        q_home_start = HOME_QPOS.copy()
        q_target_init = HOME_QPOS.copy()
        overforce_cooldown = 0.0

        try:
            while not self.stop_event.is_set():
                step_start = time.time()
                now = step_start

                # ---- Read state from MuJoCo thread ----
                with self.buf.lock:
                    q = self.buf.q.copy()
                    dq = self.buf.dq.copy()
                    tau_grav = self.buf.tau_grav.copy()
                    tau_contact = self.buf.tau_contact.copy()

                # ---- Shared control mode ----
                cur_state, h_t_start, h_duration = self.mode_shm.read()

                if cur_state != prev_state:
                    print(f'[follower_sim] state  '
                          f'{_state_name(prev_state)} → {_state_name(cur_state)}')
                    if cur_state == MODE_PAUSED:
                        if prev_state == MODE_HOMING:
                            q_hold = HOME_QPOS.copy()
                        else:
                            q_hold = q.copy()
                    elif cur_state == MODE_HOMING:
                        q_home_start = q.copy()

                # ---- State-specific control law ----
                if cur_state == MODE_ACTIVE:
                    # Read peer (leader) state directly from shm
                    self._peer_shm.seek(0)
                    pv = struct.unpack(_STATE_FMT,
                                       self._peer_shm.read(STATE_SIZE))
                    peer_q = np.array(pv[0:6])
                    peer_dq = np.array(pv[6:12])
                    peer_ts = pv[18]

                    if peer_ts > 0:
                        tau = (KP_FALLBACK * (peer_q - q)
                               + KD_FALLBACK * (peer_dq - dq)
                               + tau_grav)
                    else:
                        # Leader hasn't written yet — hold home
                        tau = (KP_FALLBACK * (q_target_init - q)
                               - KD_FALLBACK * dq + tau_grav)

                elif cur_state == MODE_PAUSED:
                    tau = KP_HOLD * (q_hold - q) - KD_HOLD * dq + tau_grav

                elif cur_state == MODE_HOMING:
                    dur = h_duration if h_duration > 0 else HOMING_DURATION_FALLBACK
                    t = max(0.0, now - h_t_start)
                    alpha = min(t / dur, 1.0)
                    ease = 3.0 * alpha ** 2 - 2.0 * alpha ** 3
                    q_des = (1.0 - ease) * q_home_start + ease * HOME_QPOS
                    tau = KP_HOLD * (q_des - q) - KD_HOLD * dq + tau_grav

                else:
                    tau = tau_grav

                tau = np.clip(tau, self.act_lo, self.act_hi)

                # ---- Write tau for MuJoCo thread ----
                with self.buf.lock:
                    self.buf.tau[:] = tau

                # ---- Over-force detection (ACTIVE only) ----
                if cur_state == MODE_ACTIVE and now > overforce_cooldown:
                    abs_c = np.abs(tau_contact)
                    if np.any(abs_c > OVERFORCE_CONSTRAINT):
                        i = int(np.argmax(abs_c))
                        print(f'[follower_sim] OVER-FORCE  contact on joint[{i}] = '
                              f'{tau_contact[i]:+.1f} Nm  →  PAUSED')
                        self.mode_shm.write(MODE_PAUSED)
                        overforce_cooldown = now + 1.0

                # ---- Publish state to shm ----
                self.state_writer.write(q, dq, tau_contact, time.time())

                prev_state = cur_state

                elapsed = time.time() - step_start
                sleep_t = self.model.opt.timestep - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except Exception:
            traceback.print_exc()
            self.stop_event.set()

    # ---- MuJoCo thread (physics + render) ----------------------------------

    def run(self):
        """Start control thread, open viewer, and run the physics loop.
        Must be called from the main thread (GLFW requirement)."""
        self._control_thread.start()

        render_tick = 0

        with mujoco.viewer.launch_passive(self.model, self.data,
                                           show_left_ui=False,
                                           show_right_ui=False) as viewer:
            viewer.cam.azimuth = 225.0
            viewer.cam.elevation = -20.0
            viewer.cam.distance = 2.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.4]

            print('[follower_sim] Running.  '
                  'Waiting for bilateral_control commands...')

            try:
                while viewer.is_running() and not self.stop_event.is_set():
                    step_start = time.time()

                    with self.buf.lock:
                        self.data.ctrl[:N] = self.buf.tau

                    mujoco.mj_step(self.model, self.data)

                    with self.buf.lock:
                        self.buf.q[:] = self.data.qpos[self.qpos_ids]
                        self.buf.dq[:] = self.data.qvel[self.qvel_ids]
                        self.buf.tau_grav[:] = self.data.qfrc_bias[self.qvel_ids]
                        self.buf.tau_contact[:] = \
                            self.data.qfrc_constraint[self.qvel_ids]

                    render_tick += 1
                    if render_tick >= RENDER_EVERY:
                        render_tick = 0
                        viewer.sync()

                    elapsed = time.time() - step_start
                    sleep_t = self.model.opt.timestep - elapsed
                    if sleep_t > 0:
                        time.sleep(sleep_t)
            finally:
                self.stop_event.set()
                self._control_thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=os.path.join(
        os.path.dirname(__file__), '..', 'xml', 'ur10e_follower_scene.xml'))
    args = parser.parse_args()

    FollowerSim(args.model).run()


if __name__ == '__main__':
    main()
