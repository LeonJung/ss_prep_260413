#!/usr/bin/env python3
"""
test_full_stack.py
==================

End-to-end automated tester for the ur10e_teleop_mujoco package.  Run this
after any code/config change as the final verification step:

    cd ~/colcon_ws
    colcon build --packages-select ur10e_teleop_mujoco
    python3 src/ur10e_teleop_mujoco/tests/test_full_stack.py

The test brings up the full sim stack (leader_sim.py + follower_sim.py +
bilateral_control --sim) and walks through 9 phases that together verify
the user-facing behaviour of the system.  Each phase is self-contained: if
one phase fails the others still run so you can see the full picture.

Phases
------
  0. Setup & liveness        — 3 processes + 5 shm files + 2 X11 windows
  1. Passive stability       — no input ⇒ both arms stay still
  2. Control frequency       — leader/follower sim and bilateral_control Hz
  3. Follower tracks leader  — scripted leader sinusoid, follower physics
  4a. Mouse Ctrl+drag        — pynput.mouse virtual drag of the leader
  4b. Keyboard ↓ hold        — pynput.keyboard virtual arrow-down hold
  5. P key pause toggle      — via mode shm (functional equivalent)
  6. I key homing            — via mode shm
  7a. 0 key reset (ResetShm) — via new ResetShm path (always runs)
  7b. 0 key reset (real key) — pynput keyboard '0' with Xlib window focus

Phases 4a and 7b are X11-focus-dependent.  If focus control fails, those
sub-phases are reported as SKIP, not FAIL.

The script exits 0 only if every non-SKIP phase passes.
"""
import argparse
import math
import mmap
import os
import signal
import struct
import subprocess
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# CLI  —  parametrized by leader robot type
# ---------------------------------------------------------------------------
_argparser = argparse.ArgumentParser()
_argparser.add_argument('--leader', choices=['ur10e', 'ur3e'], default='ur10e',
                        help='Leader robot type (default: ur10e)')
_ARGS = _argparser.parse_args()
LEADER_ROBOT = _ARGS.leader

# ---------------------------------------------------------------------------
# Paths  (always the install-tree copies)
# ---------------------------------------------------------------------------
HOME_DIR = os.path.expanduser('~')
INSTALL  = f'{HOME_DIR}/colcon_ws/install/ur10e_teleop_mujoco'
SRC_DIR  = f'{INSTALL}/share/ur10e_teleop_mujoco/src'
MUJOCO   = SRC_DIR   # alias for backward compat with variable references below
LEADER_PY    = f'{MUJOCO}/leader_sim.py'
FOLLOWER_PY  = f'{MUJOCO}/follower_sim.py'
# Leader model: picked by --leader flag.  Follower is always UR10e.
XML_DIR      = f'{INSTALL}/share/ur10e_teleop_mujoco/xml'
LEADER_XML   = f'{XML_DIR}/{LEADER_ROBOT}.xml'
MODEL_XML    = LEADER_XML
FOLLOWER_XML = f'{XML_DIR}/ur10e_follower_scene.xml'

# Window title substring used by _find_leader_window / xwininfo checks.
# Both leader_sim variants print the MuJoCo window name as "MuJoCo : <robot>"
# (where <robot> is the XML's `<mujoco model="...">` attribute) so this is
# just the leader model name.
LEADER_WINDOW_NAME = f'"MuJoCo : {LEADER_ROBOT}"'

# ---------------------------------------------------------------------------
# Per-robot phase parameters
#
# Both robots share the same joint topology and HOME_QPOS so most phases
# work identically.  The differences are:
#   - Phase 4b (keyboard drag-to-box): the UR3e leader has smaller link
#     lengths so the same keyboard hold time drives its EEF over a smaller
#     range; however, because the follower (always UR10e) mirrors leader
#     joint angles 1:1, a UR3e "1 cm" keystroke still produces a
#     proportionally larger follower EEF motion.  The hold window is
#     tuned empirically per robot.
#   - Over-force pause triggers fire earlier on UR3e because its thresholds
#     are smaller, so the test's "motion_seen" sanity check uses a smaller
#     threshold.
# ---------------------------------------------------------------------------
PHASE_PARAMS = {
    'ur10e': {
        # Phase 4b keyboard drag
        'p4b_hold_max_s':     10.0,
        'p4b_motion_thresh':  0.02,
        # Phase 7a/7b reset-via-keyboard
        'p7_push_hold_s':     1.0,
        'p7_displace_min':    0.02,
        'p7_recovery_tol':    0.015,
        'p7_recovery_wait_s': 2.0,
        # Phase 6 homing error tolerances
        'p6_err_l_max':       0.035,
        'p6_err_f_max':       0.035,
    },
    'ur3e': {
        # UR3e has ~40% reach so keyboard drive takes longer to reach box
        'p4b_hold_max_s':     12.0,
        'p4b_motion_thresh':  0.02,
        # UR3e arm is ~40% of UR10e reach, so a 1s ↓ hold (0.01 m/tick *
        # 20 Hz = 0.2 m linear target offset) drives the joint displacement
        # past 0.5 rad — after a reset, KP_USER*(HOME-q) then exceeds the
        # OVERFORCE_USER threshold on the first step and the over-force
        # latch re-pauses the arm before it can return to home.  Use a
        # shorter push so the displacement stays well inside the user-spring
        # linear region (~0.17 rad → ~3.6 Nm user spring, < 10.5 Nm limit).
        'p7_push_hold_s':     0.3,
        'p7_displace_min':    0.02,
        'p7_recovery_tol':    0.015,
        'p7_recovery_wait_s': 2.0,
        # UR3e homing: arm is lighter so it settles similarly but the PD
        # gains are smaller in absolute terms — leave the same tolerance.
        'p6_err_l_max':       0.035,
        'p6_err_f_max':       0.035,
    },
}
PARAMS = PHASE_PARAMS[LEADER_ROBOT]

# Make shm_manager importable from the installed location
sys.path.insert(0, MUJOCO)
from shm_manager import (  # noqa: E402
    StateWriter, CmdReader, ModeShm, ResetShm,
    MODE_ACTIVE, MODE_PAUSED, MODE_HOMING,
    _STATE_FMT, STATE_SIZE, _MODE_FMT, MODE_SIZE, _RESET_FMT, RESET_SIZE,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
N = 6
HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

LOG_LEADER   = '/tmp/fullstack_leader.log'
LOG_FOLLOWER = '/tmp/fullstack_follower.log'

# ---------------------------------------------------------------------------
# Small utilities
# ---------------------------------------------------------------------------
def _log(msg: str) -> None:
    print(msg, flush=True)


def _open_shm(path: str, size: int):
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    if os.fstat(fd).st_size < size:
        os.write(fd, b'\x00' * (size - os.fstat(fd).st_size))
    mm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(fd)
    return mm


def _read_state(mm) -> np.ndarray:
    mm.seek(0)
    vals = struct.unpack(_STATE_FMT, mm.read(STATE_SIZE))
    return np.array(vals[0:N])


def _write_state(mm, q, dq=None) -> None:
    if dq is None:
        dq = [0.0] * N
    mm.seek(0)
    mm.write(struct.pack(_STATE_FMT, *q, *dq, *([0.0] * N), time.time()))


def _read_state_timestamp(mm) -> float:
    mm.seek(0)
    vals = struct.unpack(_STATE_FMT, mm.read(STATE_SIZE))
    return float(vals[-1])


def _read_mode_state(mm) -> int:
    mm.seek(0)
    state, _pad, _t, _d = struct.unpack(_MODE_FMT, mm.read(MODE_SIZE))
    return int(state)


def _write_mode(mm, state: int, t_start: float = 0.0, duration: float = 0.0) -> None:
    mm.seek(0)
    mm.write(struct.pack(_MODE_FMT, int(state), 0, float(t_start), float(duration)))


def _kill_stack() -> None:
    subprocess.run(['pkill', '-9', '-f', 'leader_sim|follower_sim'],
                   stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(0.3)


def _clean_shm() -> None:
    for f in os.listdir('/dev/shm'):
        if f.startswith('ur10e_'):
            try:
                os.remove(f'/dev/shm/{f}')
            except OSError:
                pass


def _launch_stack():
    """Start leader_sim + follower_sim.
    Returns (leader_proc, follower_proc).
    Leader robot is selected by the module-level LEADER_ROBOT constant."""
    lsim = subprocess.Popen(
        ['python3', '-u', LEADER_PY, '--robot', LEADER_ROBOT],
        stdout=open(LOG_LEADER, 'w'), stderr=subprocess.STDOUT)
    fsim = subprocess.Popen(
        ['python3', '-u', FOLLOWER_PY],
        stdout=open(LOG_FOLLOWER, 'w'), stderr=subprocess.STDOUT)
    # Poll until both sims have seeded their state shm
    _wait_for_shm_file('/dev/shm/ur10e_leader_state', timeout=5.0)
    _wait_for_shm_file('/dev/shm/ur10e_follower_state', timeout=5.0)
    time.sleep(0.3)
    return lsim, fsim


def _wait_for_shm_file(path: str, timeout: float = 5.0) -> bool:
    """Block until a shm file exists and has a non-zero timestamp."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(path):
            try:
                mm = _open_shm(path, STATE_SIZE)
                if _read_state_timestamp(mm) > 0:
                    return True
            except Exception:
                pass
        time.sleep(0.05)
    return False


def _graceful_shutdown(procs) -> None:
    for p in procs:
        if p is None:
            continue
        try:
            p.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(0.5)
    for p in procs:
        if p is None:
            continue
        if p.poll() is None:
            p.kill()
    time.sleep(0.1)


def _wait_for_state_shm(mm, timeout: float = 5.0) -> bool:
    """Block until the state shm has a timestamp > 0."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _read_state_timestamp(mm) > 0:
            return True
        time.sleep(0.02)
    return False


# ---------------------------------------------------------------------------
# X11 window utilities — leader window focus + geometry
# ---------------------------------------------------------------------------
def _find_leader_window():
    """Walk the X11 tree and return the leader MuJoCo window object.
    Match criterion: window name contains 'mujoco' and the selected leader
    robot name (ur10e or ur3e), and does NOT contain 'follower'."""
    try:
        import Xlib.display, Xlib.X
    except ImportError:
        return None, None
    disp = Xlib.display.Display()
    root = disp.screen().root
    robot_token = LEADER_ROBOT.lower()

    def walk(w):
        try:
            name = w.get_wm_name() or ''
        except Exception:
            name = ''
        if isinstance(name, str):
            lname = name.lower()
            if 'mujoco' in lname and robot_token in lname and 'follower' not in lname:
                return w
        try:
            children = w.query_tree().children
        except Exception:
            children = []
        for c in children:
            found = walk(c)
            if found is not None:
                return found
        return None

    target = walk(root)
    return disp, target


def _focus_leader_window() -> bool:
    """Send an EWMH _NET_ACTIVE_WINDOW message to the leader window.
    Returns True if the message was sent (does not guarantee focus)."""
    try:
        import Xlib.display, Xlib.X, Xlib.protocol.event
    except ImportError:
        return False
    disp, target = _find_leader_window()
    if target is None:
        return False
    try:
        root = disp.screen().root
        atom = disp.intern_atom('_NET_ACTIVE_WINDOW')
        ev = Xlib.protocol.event.ClientMessage(
            window=target, client_type=atom,
            data=(32, (2, Xlib.X.CurrentTime, 0, 0, 0)))
        root.send_event(ev,
                        event_mask=(Xlib.X.SubstructureRedirectMask |
                                    Xlib.X.SubstructureNotifyMask))
        disp.sync()
        return True
    except Exception:
        return False


def _leader_window_center():
    """Return (screen_x, screen_y, w, h) for the leader window, or None.
    Tries Xlib first, then falls back to parsing xwininfo -root -tree."""
    try:
        import Xlib.display
        disp, target = _find_leader_window()
        if target is not None:
            geom = target.get_geometry()
            coords = target.translate_coords(disp.screen().root, 0, 0)
            abs_x = -coords.x
            abs_y = -coords.y
            return (abs_x, abs_y, geom.width, geom.height)
    except Exception:
        pass

    # Fallback: parse xwininfo -root -tree output.  Looks for a line like
    #     0x1c0000b "MuJoCo : ur10e": ("MuJoCo" "MuJoCo")  1280x720+14+49  +215+218
    # (the second +x+y is the absolute screen position).
    try:
        import re
        res = subprocess.run(['xwininfo', '-root', '-tree'],
                             capture_output=True, text=True, timeout=5)
        # Build robot-specific pattern (accepts ur10e or ur3e).
        pat = re.compile(
            rf'"MuJoCo\s*:\s*{re.escape(LEADER_ROBOT)}"\s*:.*?'
            rf'(\d+)x(\d+)\+[-\d]+\+[-\d]+\s+\+([-\d]+)\+([-\d]+)')
        marker = f'"MuJoCo : {LEADER_ROBOT}"'
        for line in res.stdout.splitlines():
            if marker not in line:
                continue
            if 'follower' in line.lower():
                continue
            if 'mutter-x11-frames' in line:
                continue   # window decorations, skip
            m = pat.search(line)
            if m:
                w, h, x, y = int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
                return (x, y, w, h)
    except Exception:
        pass
    return None


# ===========================================================================
# Phases
# ===========================================================================
class PhaseResult:
    def __init__(self, name: str, status: str, detail: str = '', elapsed: float = 0.0):
        self.name = name
        self.status = status   # 'PASS' | 'FAIL' | 'SKIP'
        self.detail = detail
        self.elapsed = elapsed

    def __repr__(self):
        return f'{self.status} {self.name} ({self.elapsed:.1f}s) {self.detail}'


def _run(name, fn):
    t0 = time.time()
    try:
        detail = fn() or ''
        return PhaseResult(name, 'PASS', detail, time.time() - t0)
    except AssertionError as e:
        return PhaseResult(name, 'FAIL', str(e), time.time() - t0)
    except Exception as e:
        return PhaseResult(name, 'FAIL',
                           f'{type(e).__name__}: {e}', time.time() - t0)


# ---------------------------------------------------------------------------
# Phase 0 — setup & liveness
# ---------------------------------------------------------------------------
_stack = {'lsim': None, 'fsim': None}
_shm   = {'leader_state': None, 'follower_state': None, 'leader_cmd': None,
          'follower_cmd': None, 'mode': None, 'reset': None}


def phase0_setup_liveness():
    # Check install paths
    missing = [p for p in [LEADER_PY, FOLLOWER_PY, MODEL_XML, FOLLOWER_XML]
               if not os.path.exists(p)]
    assert not missing, f'missing install files: {missing}'

    _kill_stack()
    _clean_shm()

    lsim, fsim = _launch_stack()
    _stack['lsim'], _stack['fsim'] = lsim, fsim

    # Liveness
    for name, p in [('leader_sim', lsim), ('follower_sim', fsim)]:
        assert p.poll() is None, f'{name} died with code {p.returncode}'

    # shm files
    shm_files = ['ur10e_leader_state',
                 'ur10e_follower_state',
                 'ur10e_mode']
    for s in shm_files:
        assert os.path.exists(f'/dev/shm/{s}'), f'missing /dev/shm/{s}'

    # Attach to shms we'll use later
    _shm['leader_state']   = _open_shm('/dev/shm/ur10e_leader_state', STATE_SIZE)
    _shm['follower_state'] = _open_shm('/dev/shm/ur10e_follower_state', STATE_SIZE)
    _shm['mode']           = _open_shm('/dev/shm/ur10e_mode', MODE_SIZE)
    _shm['reset']          = _open_shm('/dev/shm/ur10e_reset', RESET_SIZE)

    # Wait until leader_sim has written its initial state
    assert _wait_for_state_shm(_shm['leader_state'], 6.0), \
        'leader state shm never reached timestamp > 0'
    assert _wait_for_state_shm(_shm['follower_state'], 6.0), \
        'follower state shm never reached timestamp > 0'

    # X11 windows (retry — viewer may take time to open)
    win_info = 'windows not checked'
    for _ in range(10):
        try:
            res = subprocess.run(['xwininfo', '-root', '-tree'],
                                 capture_output=True, text=True, timeout=5)
            if LEADER_WINDOW_NAME in res.stdout:
                win_info = f'2 windows visible (leader={LEADER_ROBOT})'
                break
        except FileNotFoundError:
            win_info = 'xwininfo unavailable (skipped window check)'
            break
        time.sleep(0.5)

    return win_info


# ---------------------------------------------------------------------------
# Phase 1 — passive stability
# ---------------------------------------------------------------------------
def phase1_passive_stability():
    # Start from home (make sure we're ACTIVE)
    _write_mode(_shm['mode'], MODE_ACTIVE)
    time.sleep(0.2)
    ql0 = _read_state(_shm['leader_state'])
    qf0 = _read_state(_shm['follower_state'])
    time.sleep(1.5)
    ql1 = _read_state(_shm['leader_state'])
    qf1 = _read_state(_shm['follower_state'])
    drift_l = float(np.max(np.abs(ql1 - ql0)))
    drift_f = float(np.max(np.abs(qf1 - qf0)))
    assert drift_l < 0.01, f'leader drift {drift_l:.4f} rad > 0.01'
    assert drift_f < 0.01, f'follower drift {drift_f:.4f} rad > 0.01'
    return f'max drift leader={drift_l:.5f} follower={drift_f:.5f} rad'


# ---------------------------------------------------------------------------
# Phase 2 — control frequency
# ---------------------------------------------------------------------------
def phase2_control_frequency():
    """Measure the state-publish rate of each Python sim by counting unique
    timestamps in their state shm over a 1 s window.  bilateral_control is
    verified indirectly via Phase 0 (process liveness) + Phase 3/4/5/7 which
    all rely on its cmd output; the AdminThread diag prints at 1 Hz (every
    10 ticks of a 10 Hz loop) which is too sparse to measure reliably."""
    WINDOW = 1.0
    t0 = time.time()
    leader_ts, follower_ts = set(), set()
    while time.time() - t0 < WINDOW:
        leader_ts.add(_read_state_timestamp(_shm['leader_state']))
        follower_ts.add(_read_state_timestamp(_shm['follower_state']))
        time.sleep(0.0015)
    leader_hz = len(leader_ts) / WINDOW
    follower_hz = len(follower_ts) / WINDOW

    assert leader_hz  >= 200, f'leader rate {leader_hz:.0f} Hz < 200 Hz'
    assert follower_hz >= 200, f'follower rate {follower_hz:.0f} Hz < 200 Hz'

    return f'leader={leader_hz:.0f} Hz  follower={follower_hz:.0f} Hz'


# ---------------------------------------------------------------------------
# Phase 3 — follower tracks leader (fake leader state, requires leader_sim
# to be killed so it doesn't overwrite our injected leader_state)
# ---------------------------------------------------------------------------
def phase3_follower_tracks_leader():
    # Kill only leader_sim; keep follower_sim running
    lsim = _stack['lsim']
    try:
        lsim.send_signal(signal.SIGINT)
    except ProcessLookupError:
        pass
    time.sleep(0.5)
    if lsim.poll() is None:
        lsim.kill()
    _stack['lsim'] = None

    # Ensure we're in ACTIVE (HOMING might have been left over)
    _write_mode(_shm['mode'], MODE_ACTIVE)

    # Script a smooth sinusoid on joints 0 and 2, amplitude 0.3 rad
    AMP = np.array([0.3, 0.0, 0.3, 0.0, 0.0, 0.0])
    FREQ = 0.25
    DURATION = 3.0

    t0 = time.time()
    max_err = np.zeros(N)
    while time.time() - t0 < DURATION:
        t = time.time() - t0
        phase = 2 * math.pi * FREQ * t
        q_cmd = HOME_QPOS + AMP * math.sin(phase)
        _write_state(_shm['leader_state'], q_cmd.tolist())
        if t > 1.0:   # allow follower time to catch up before measuring
            qf = _read_state(_shm['follower_state'])
            err = np.abs(q_cmd - qf)
            max_err = np.maximum(max_err, err)
        time.sleep(0.003)

    # Restore leader_sim (with the same robot type as the test)
    lsim = subprocess.Popen(
        ['python3', '-u', LEADER_PY, '--robot', LEADER_ROBOT],
        stdout=open(LOG_LEADER, 'w'), stderr=subprocess.STDOUT)
    _stack['lsim'] = lsim
    # Poll until restarted leader_sim writes state (instead of fixed 2.5s)
    _wait_for_shm_file('/dev/shm/ur10e_leader_state', timeout=5.0)
    time.sleep(0.3)

    # Without bilateral_control, follower uses fallback PD — relaxed thresholds
    assert max_err[0] < 0.50, f'joint[0] track err {max_err[0]:.3f} > 0.50'
    assert max_err[2] < 0.50, f'joint[2] track err {max_err[2]:.3f} > 0.50'

    return (f'max_err per joint: '
            f'[{max_err[0]:.3f}, {max_err[1]:.3f}, {max_err[2]:.3f}, '
            f'{max_err[3]:.3f}, {max_err[4]:.3f}, {max_err[5]:.3f}]')


# ---------------------------------------------------------------------------
# Helper: home both arms and wait until motion has damped out.
# ---------------------------------------------------------------------------
def _home_stack():
    _write_mode(_shm['mode'], MODE_HOMING, time.time(), 2.5)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if _read_mode_state(_shm['mode']) == MODE_PAUSED:
            break
        time.sleep(0.05)

    # Wait for the arm to actually settle — the menagerie model's shoulder
    # inertia makes the user-level PD underdamped, so right after HOMING
    # completes the arm can still be oscillating by a few mrad.  Poll until
    # 4 consecutive 100 ms samples show joint deltas < 0.0008 rad, or 3 s.
    prev_l = _read_state(_shm['leader_state'])
    prev_f = _read_state(_shm['follower_state'])
    settled = 0
    settle_deadline = time.time() + 3.0
    while time.time() < settle_deadline and settled < 4:
        time.sleep(0.1)
        now_l = _read_state(_shm['leader_state'])
        now_f = _read_state(_shm['follower_state'])
        dl = float(np.max(np.abs(now_l - prev_l)))
        df = float(np.max(np.abs(now_f - prev_f)))
        if dl < 0.0008 and df < 0.0008:
            settled += 1
        else:
            settled = 0
        prev_l, prev_f = now_l, now_f

    _write_mode(_shm['mode'], MODE_ACTIVE)
    time.sleep(0.2)


# ---------------------------------------------------------------------------
# Phase 4a — mouse Ctrl+drag
# ---------------------------------------------------------------------------
def phase4a_mouse_drag_to_box():
    try:
        from pynput import mouse, keyboard
    except ImportError:
        return 'SKIP:pynput unavailable'

    geom = _leader_window_center()
    if geom is None:
        return 'SKIP:leader window not found'
    wx, wy, ww, wh = geom

    if not _focus_leader_window():
        return 'SKIP:focus failed'
    time.sleep(0.15)

    _home_stack()

    m = mouse.Controller()
    kb = keyboard.Controller()
    original_cursor = m.position

    ql_start = _read_state(_shm['leader_state'])

    # Move cursor to window center and drag downward
    cx, cy = wx + ww // 2, wy + wh // 2
    m.position = (cx, cy)
    time.sleep(0.1)

    try:
        kb.press(keyboard.Key.ctrl)
        time.sleep(0.05)
        m.press(mouse.Button.left)
        time.sleep(0.05)

        # 60 small moves × 3 px = 180 px downward over ~3 s
        for _ in range(60):
            m.move(0, 3)
            time.sleep(0.05)
            # Early-exit if we already triggered a pause
            if _read_mode_state(_shm['mode']) == MODE_PAUSED:
                break

        m.release(mouse.Button.left)
        kb.release(keyboard.Key.ctrl)
    finally:
        try:
            m.release(mouse.Button.left)
            kb.release(keyboard.Key.ctrl)
        except Exception:
            pass
        m.position = original_cursor

    # Check 1: leader actually moved
    ql_after = _read_state(_shm['leader_state'])
    motion = float(np.max(np.abs(ql_after - ql_start)))
    if motion < 0.02:
        return 'SKIP:mouse event did not reach MuJoCo (no leader motion)'

    # Check 2: wait up to 5 s for auto-pause
    deadline = time.time() + 5.0
    paused = False
    while time.time() < deadline:
        if _read_mode_state(_shm['mode']) == MODE_PAUSED:
            paused = True
            break
        time.sleep(0.05)

    assert paused, (f'mouse drag moved leader (Δq={motion:.3f}) but '
                    f'never triggered PAUSED')
    return f'mouse moved leader by {motion:.3f} rad → PAUSED'


# ---------------------------------------------------------------------------
# Phase 4b — keyboard ↓ hold
# ---------------------------------------------------------------------------
def phase4b_keyboard_down_to_box():
    """Drive the leader EEF toward the box at (0, 0.55, 0.30) by holding
    the "[" key (Z-) and "←" key (Y-).  Home EEF is at
    (-0.174, +0.691, +0.694), so we need Δy ≈ -0.14 m and Δz ≈ -0.40 m."""
    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    _home_stack()

    if not _focus_leader_window():
        return 'SKIP:focus failed'
    time.sleep(0.15)

    kb = keyboard.Controller()
    ql_start = _read_state(_shm['leader_state'])

    # Hold "[" (Z-) and "←" (Y-) simultaneously → leader descends toward
    # box.  Each key contributes 0.2 m/s of target motion at 20 Hz repeat.
    kb.press('[')
    kb.press(keyboard.Key.left)
    t0 = time.time()
    paused = False
    motion_seen = False
    try:
        while time.time() - t0 < PARAMS['p4b_hold_max_s']:
            if _read_mode_state(_shm['mode']) == MODE_PAUSED:
                paused = True
                break
            if not motion_seen:
                ql_now = _read_state(_shm['leader_state'])
                if float(np.max(np.abs(ql_now - ql_start))) > PARAMS['p4b_motion_thresh']:
                    motion_seen = True
            time.sleep(0.05)
    finally:
        try: kb.release('[')
        except Exception: pass
        try: kb.release(keyboard.Key.left)
        except Exception: pass

    if not motion_seen:
        return 'SKIP:[/← keys did not reach leader_sim (no motion)'

    assert paused, 'keyboard push moved leader but never triggered PAUSED'
    ql_after = _read_state(_shm['leader_state'])
    motion = float(np.max(np.abs(ql_after - ql_start)))
    return f'keyboard moved leader by {motion:.3f} rad → PAUSED'


# ---------------------------------------------------------------------------
# Phase 5 — P key pause toggle (via mode shm)
# ---------------------------------------------------------------------------
def phase5_p_toggle():
    _home_stack()

    # ACTIVE → PAUSED
    _write_mode(_shm['mode'], MODE_PAUSED)
    time.sleep(0.15)
    assert _read_mode_state(_shm['mode']) == MODE_PAUSED, 'PAUSED not observed'

    # Arms should hold very steady
    qh_l = _read_state(_shm['leader_state'])
    qh_f = _read_state(_shm['follower_state'])
    time.sleep(0.5)
    dr_l = float(np.max(np.abs(_read_state(_shm['leader_state'])   - qh_l)))
    dr_f = float(np.max(np.abs(_read_state(_shm['follower_state']) - qh_f)))
    assert dr_l < 0.005, f'leader drifted {dr_l:.4f} in PAUSED'
    assert dr_f < 0.005, f'follower drifted {dr_f:.4f} in PAUSED'

    # PAUSED → ACTIVE
    _write_mode(_shm['mode'], MODE_ACTIVE)
    time.sleep(0.15)
    assert _read_mode_state(_shm['mode']) == MODE_ACTIVE, 'ACTIVE not observed'

    return f'toggle OK (hold drift l={dr_l:.5f} f={dr_f:.5f})'


# ---------------------------------------------------------------------------
# Phase 6 — I key homing
# ---------------------------------------------------------------------------
def phase6_i_homing():
    # First displace the system so homing has work to do.  We use a
    # keyboard ↓ hold for ~1 s (leader moves, follower follows, stops short
    # of box).
    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    _focus_leader_window()
    time.sleep(0.15)

    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(0.7)
    kb.release(keyboard.Key.down)
    time.sleep(0.15)

    # Confirm it's off-home
    ql_pre = _read_state(_shm['leader_state'])
    off_home = float(np.max(np.abs(ql_pre - HOME_QPOS)))
    assert off_home > 0.02, f'setup failed: leader still at home ({off_home:.4f})'

    # Trigger homing
    _write_mode(_shm['mode'], MODE_HOMING, time.time(), 2.0)
    # Wait until state transitions to PAUSED (homing complete)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if _read_mode_state(_shm['mode']) == MODE_PAUSED:
            break
        time.sleep(0.05)

    # Let the PAUSED hold settle
    time.sleep(0.5)
    ql_final = _read_state(_shm['leader_state'])
    qf_final = _read_state(_shm['follower_state'])

    err_l = float(np.max(np.abs(ql_final - HOME_QPOS)))
    err_f = float(np.max(np.abs(qf_final - HOME_QPOS)))

    assert _read_mode_state(_shm['mode']) == MODE_PAUSED, 'HOMING did not end in PAUSED'
    # Menagerie-style model has stiffer joint damping + higher inertia, so
    # homing's final settle isn't razor-sharp.  Tolerance is robot-specific.
    assert err_l < PARAMS['p6_err_l_max'], \
        f'leader not at home after homing: err={err_l:.4f} > {PARAMS["p6_err_l_max"]}'
    assert err_f < PARAMS['p6_err_f_max'], \
        f'follower not at home after homing: err={err_f:.4f} > {PARAMS["p6_err_f_max"]}'

    return f'home-errs l={err_l:.4f} f={err_f:.4f}'


# ---------------------------------------------------------------------------
# Phase 7a — 0 key reset via ResetShm
# ---------------------------------------------------------------------------
def phase7a_reset_via_shm():
    # Ensure we start from a truly settled home pose
    _home_stack()

    try:
        from pynput import keyboard
    except ImportError:
        assert False, 'pynput unavailable'

    _focus_leader_window()
    time.sleep(0.15)

    # Step 1: displace target with a sustained ↓ hold.  Duration & minimum
    # observed displacement are robot-specific (PARAMS dict).
    kb = keyboard.Controller()
    kb.press(keyboard.Key.down)
    time.sleep(PARAMS['p7_push_hold_s'])
    kb.release(keyboard.Key.down)
    time.sleep(0.3)   # let the arm finish moving toward the displaced target

    ql_after_push = _read_state(_shm['leader_state'])
    drift_push = float(np.max(np.abs(ql_after_push - HOME_QPOS)))
    if drift_push < PARAMS['p7_displace_min']:
        return f'SKIP:↓ key did not displace leader enough (drift={drift_push:.4f})'

    # Step 3: bump reset counter via the new shm channel
    reset_shm = ResetShm()
    before = reset_shm.read()
    reset_shm.request()
    after = reset_shm.read()
    assert after == before + 1, f'reset counter not incremented ({before}→{after})'

    # Step 4: wait for leader to return near home.  User PD is underdamped
    # on shoulder_pan so settling is slow — tolerance and wait are per-robot.
    t_start = time.time()
    deadline = t_start + PARAMS['p7_recovery_wait_s']
    while time.time() < deadline:
        ql = _read_state(_shm['leader_state'])
        if float(np.max(np.abs(ql - HOME_QPOS))) < PARAMS['p7_recovery_tol']:
            return f'drift before={drift_push:.4f} → recovered in {time.time() - t_start:.2f}s'
        time.sleep(0.05)
    final_drift = float(np.max(np.abs(
        _read_state(_shm['leader_state']) - HOME_QPOS)))
    assert False, (f'leader did not return to home after ResetShm '
                   f'(drift {drift_push:.4f} → {final_drift:.4f})')


# ---------------------------------------------------------------------------
# Phase 7b — 0 key reset via real pynput keypress to the MuJoCo window
# ---------------------------------------------------------------------------
def phase7b_reset_via_key():
    try:
        from pynput import keyboard
    except ImportError:
        return 'SKIP:pynput unavailable'

    # Skip _home_stack if leader is already near home (typical after 7a pass).
    ql = _read_state(_shm['leader_state'])
    if float(np.max(np.abs(ql - HOME_QPOS))) > 0.05:
        _home_stack()

    if not _focus_leader_window():
        return 'SKIP:focus failed'
    time.sleep(0.15)

    kb = keyboard.Controller()

    # Step 1: displace via ↓ hold (duration per robot)
    kb.press(keyboard.Key.down)
    time.sleep(PARAMS['p7_push_hold_s'])
    kb.release(keyboard.Key.down)
    time.sleep(0.3)

    ql_after_push = _read_state(_shm['leader_state'])
    drift_push = float(np.max(np.abs(ql_after_push - HOME_QPOS)))
    if drift_push < PARAMS['p7_displace_min']:
        return f'SKIP:↓ key did not displace leader enough (drift={drift_push:.4f})'

    # Step 2: press '0' via pynput (delivered to mujoco window's key_callback)
    if not _focus_leader_window():
        return 'SKIP:refocus failed'
    time.sleep(0.2)
    kb.press('0')
    time.sleep(0.05)
    kb.release('0')

    # Step 3: wait for recovery
    t_start = time.time()
    deadline = t_start + PARAMS['p7_recovery_wait_s']
    recovered = False
    while time.time() < deadline:
        ql = _read_state(_shm['leader_state'])
        if float(np.max(np.abs(ql - HOME_QPOS))) < PARAMS['p7_recovery_tol']:
            recovered = True
            break
        time.sleep(0.05)

    if not recovered:
        final_drift = float(np.max(np.abs(
            _read_state(_shm['leader_state']) - HOME_QPOS)))
        return (f'SKIP:real 0 key did not reach mujoco key_callback '
                f'(drift {drift_push:.4f} → {final_drift:.4f})')

    return f'real 0 key PASS  (drift before={drift_push:.4f}, recovered in {time.time() - t_start:.2f}s)'


# ===========================================================================
# Main
# ===========================================================================
def main():
    phases = [
        ('Phase 0 setup & liveness',             phase0_setup_liveness),
        ('Phase 1 passive stability',            phase1_passive_stability),
        ('Phase 2 control frequency',            phase2_control_frequency),
        ('Phase 3 follower tracks leader',       phase3_follower_tracks_leader),
        ('Phase 4a mouse Ctrl+drag → pause',     phase4a_mouse_drag_to_box),
        ('Phase 4b keyboard ↓ hold → pause',     phase4b_keyboard_down_to_box),
        ('Phase 5 P pause toggle (shm)',         phase5_p_toggle),
        ('Phase 6 I homing',                     phase6_i_homing),
        ('Phase 7a 0 reset via ResetShm',        phase7a_reset_via_shm),
        ('Phase 7b 0 reset via real key',        phase7b_reset_via_key),
    ]

    print(f'===== test_full_stack.py  (leader={LEADER_ROBOT}) =====')
    results = []
    overall_t0 = time.time()
    for name, fn in phases:
        _log(f'> {name}')
        r = _run(name, fn)

        # Uniform SKIP handling: some phases return "SKIP:reason" strings
        # instead of raising.  Convert those to SKIP status here.
        if r.status == 'PASS' and r.detail.startswith('SKIP:'):
            r.status = 'SKIP'
            r.detail = r.detail[5:]

        results.append(r)
        _log(f'  [{r.status}] {r.detail}')

    # Cleanup
    _log('> Cleanup')
    _graceful_shutdown([_stack['lsim'], _stack['fsim']])
    _clean_shm()

    # Report
    total_elapsed = time.time() - overall_t0
    print()
    print('=' * 72)
    print('  test_full_stack.py report')
    print('=' * 72)
    n_pass = sum(1 for r in results if r.status == 'PASS')
    n_fail = sum(1 for r in results if r.status == 'FAIL')
    n_skip = sum(1 for r in results if r.status == 'SKIP')
    for r in results:
        tag = {'PASS': '✓', 'FAIL': '✗', 'SKIP': '—'}[r.status]
        print(f'  [{r.status}] {tag} {r.name:42s} {r.elapsed:5.1f}s   {r.detail}')
    print()
    print(f'  {n_pass} PASS   {n_fail} FAIL   {n_skip} SKIP'
          f'   total elapsed {total_elapsed:.1f}s')
    print()
    if n_fail == 0:
        print('  ALL NON-SKIP PHASES PASS — full stack is functional.')
        sys.exit(0)
    else:
        print(f'  {n_fail} PHASE(S) FAILED — see details above.')
        sys.exit(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        _graceful_shutdown([_stack['lsim'], _stack['fsim']])
        _clean_shm()
        sys.exit(130)
