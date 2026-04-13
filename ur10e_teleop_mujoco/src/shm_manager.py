"""
shm_manager.py — Shared-memory manager for MuJoCo bilateral teleop simulations.

Layout (must mirror sim_ur10e.hpp):
  State  (152 B = 19 doubles): q[6], dq[6], tau[6], timestamp
  Cmd    (240 B = 30 doubles): kp[6], kd[6], q_target[6], dq_target[6], tau_ff[6]
  Feedback(48 B =  6 doubles): tau[6]  (leader force feedback from C++)

Files:
  /dev/shm/ur10e_leader_state    — leader  Python writes
  /dev/shm/ur10e_leader_cmd      — leader  Python reads (force feedback cmd)
  /dev/shm/ur10e_follower_state  — follower Python writes
  /dev/shm/ur10e_follower_cmd    — follower Python reads (bilateral control cmd)
  /dev/shm/ur10e_leader_feedback — C++ writes force feedback, leader Python reads
"""

import mmap
import os
import struct
import numpy as np

N = 6  # NUM_JOINTS

# Struct format strings (little-endian doubles)
_STATE_FMT    = '<19d'  # q[6], dq[6], tau[6], timestamp
_CMD_FMT      = '<30d'  # kp[6], kd[6], q[6], dq[6], tau_ff[6]
_FEEDBACK_FMT = '<6d'   # tau[6]

STATE_SIZE    = struct.calcsize(_STATE_FMT)     # 152
CMD_SIZE      = struct.calcsize(_CMD_FMT)       # 240
FEEDBACK_SIZE = struct.calcsize(_FEEDBACK_FMT)  # 48


def _open_shm(path: str, size: int) -> mmap.mmap:
    """Open (creating if needed) a /dev/shm file and return an mmap."""
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    # Extend to required size
    current = os.fstat(fd).st_size
    if current < size:
        os.write(fd, b'\x00' * (size - current))
    mm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(fd)
    return mm


class StateWriter:
    """Python side that writes joint state (q, dq, contact_tau, timestamp)."""

    def __init__(self, role: str):
        path = f'/dev/shm/ur10e_{role}_state'
        self._mm = _open_shm(path, STATE_SIZE)

    def write(self, q: np.ndarray, dq: np.ndarray,
              tau: np.ndarray, timestamp: float) -> None:
        data = struct.pack(_STATE_FMT,
                           *q.tolist(), *dq.tolist(), *tau.tolist(), timestamp)
        self._mm.seek(0)
        self._mm.write(data)


class CmdReader:
    """Python side that reads joint commands written by C++ bilateral control."""

    def __init__(self, role: str):
        path = f'/dev/shm/ur10e_{role}_cmd'
        self._mm = _open_shm(path, CMD_SIZE)

    def clear(self) -> None:
        """Zero the command shm (clears stale data from a previous run)."""
        self._mm.seek(0)
        self._mm.write(b'\x00' * CMD_SIZE)

    def read(self):
        """Returns (kp, kd, q_target, dq_target, tau_ff) as numpy arrays."""
        self._mm.seek(0)
        vals = struct.unpack(_CMD_FMT, self._mm.read(CMD_SIZE))
        kp        = np.array(vals[0:6])
        kd        = np.array(vals[6:12])
        q_target  = np.array(vals[12:18])
        dq_target = np.array(vals[18:24])
        tau_ff    = np.array(vals[24:30])
        return kp, kd, q_target, dq_target, tau_ff


class FeedbackReader:
    """Leader Python side that reads force-feedback torques written by C++."""

    def __init__(self):
        self._mm = _open_shm('/dev/shm/ur10e_leader_feedback', FEEDBACK_SIZE)

    def read(self) -> np.ndarray:
        self._mm.seek(0)
        vals = struct.unpack(_FEEDBACK_FMT, self._mm.read(FEEDBACK_SIZE))
        return np.array(vals)


# ---------------------------------------------------------------------------
# Mode (control state) shm  —  shared between leader_sim.py / follower_sim.py
#
# Layout (24 B):
#   state            : int32   — 0=ACTIVE, 1=PAUSED, 2=HOMING
#   _pad             : int32
#   homing_t_start   : float64 — wall clock at which HOMING began
#   homing_duration  : float64 — interpolation duration (seconds)
# ---------------------------------------------------------------------------
_MODE_FMT = '<iidd'
MODE_SIZE = struct.calcsize(_MODE_FMT)     # 24 bytes

MODE_ACTIVE = 0
MODE_PAUSED = 1
MODE_HOMING = 2


class ModeShm:
    """Shared control state between leader_sim and follower_sim."""

    def __init__(self, path: str = '/dev/shm/ur10e_mode'):
        self._mm = _open_shm(path, MODE_SIZE)

    def read(self):
        """Return (state, homing_t_start, homing_duration)."""
        self._mm.seek(0)
        state, _pad, t_start, duration = struct.unpack(
            _MODE_FMT, self._mm.read(MODE_SIZE))
        return int(state), float(t_start), float(duration)

    def write(self, state: int,
              t_start: float = 0.0,
              duration: float = 0.0) -> None:
        self._mm.seek(0)
        self._mm.write(struct.pack(
            _MODE_FMT, int(state), 0, float(t_start), float(duration)))


# ---------------------------------------------------------------------------
# Reset request shm  —  single-shot counter used to trigger the "0" key
# behaviour (reset EEF target to home pose) from an external tester.
#
# Layout (4 B):
#   counter : int32   — incremented by the tester to request a reset
#
# Semantics:  the tester calls `ResetShm.request()` to bump the counter.
# leader_sim reads the counter each iteration; whenever it sees a value
# different from its locally cached `last_reset_counter` it performs the
# same reset action as pressing "0" on the keyboard, and updates its
# cache.  This is race-free by construction: concurrent bumps may merge
# into one reset, but that is the same behaviour as rapidly tapping "0".
# ---------------------------------------------------------------------------
_RESET_FMT = '<i'
RESET_SIZE = struct.calcsize(_RESET_FMT)   # 4 bytes


class ResetShm:
    """Single-shot reset trigger shared between tests and leader_sim."""

    def __init__(self, path: str = '/dev/shm/ur10e_reset'):
        self._mm = _open_shm(path, RESET_SIZE)

    def read(self) -> int:
        self._mm.seek(0)
        return int(struct.unpack(_RESET_FMT, self._mm.read(RESET_SIZE))[0])

    def request(self) -> None:
        """Increment the request counter by 1 (bump it)."""
        cur = self.read()
        self._mm.seek(0)
        self._mm.write(struct.pack(_RESET_FMT, cur + 1))

    def clear(self) -> None:
        """Zero the counter (only used by tests to set up a clean baseline)."""
        self._mm.seek(0)
        self._mm.write(struct.pack(_RESET_FMT, 0))
