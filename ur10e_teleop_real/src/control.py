"""
control.py — RTDE-based UR robot client (URControl).

Same interface as DummyControl. Uses RTDEConnection to communicate with
a real UR robot or ur_server_dummy.py via RTDE binary protocol.

Usage:
    robot = URControl(robot_ip='127.0.0.1', robot_name='ur10e')
    robot.connect()
    q, dq = robot.read_joint_state()
    robot.write_torque(tau)
    robot.disconnect()
"""

import socket
import threading
import time

import numpy as np

from rtde_connection import RTDEConnection

N = 6

HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# UR Secondary Interface port (accepts URScript text, runs on controller)
UR_SECONDARY_PORT = 30002

# URScript uploaded to the UR controller at connect() time.
# Reads torque commands from RTDE input registers and applies via direct_torque().
# REQUIREMENTS:
#   - UR e-Series firmware >= 5.22 (for direct_torque)
#   - UR in Remote Control mode
#   - Robot initialized (brakes released)
# SAFETY:
#   - When mode register (int_register_0) != 1, applies zero torque.
#   - PC must write mode=0 before disconnect to release the arm.
URSCRIPT_TORQUE_CONTROL = """\
def rtde_torque_ctrl():
  textmsg("[rtde_torque_ctrl] START")
  zero_tau = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  iter_count = 0
  seen_active = False
  last_mode = -1
  while True:
    mode = read_input_integer_register(0)
    if mode != last_mode:
      textmsg("[rtde_torque_ctrl] mode changed to ", mode)
      last_mode = mode
    end
    if mode == 1:
      tau0 = read_input_float_register(0)
      tau1 = read_input_float_register(1)
      tau2 = read_input_float_register(2)
      tau3 = read_input_float_register(3)
      tau4 = read_input_float_register(4)
      tau5 = read_input_float_register(5)
      tau = [tau0, tau1, tau2, tau3, tau4, tau5]
      if not seen_active:
        textmsg("[rtde_torque_ctrl] first active cmd, tau0=", tau0)
        seen_active = True
      end
      direct_torque(tau)
    else:
      direct_torque(zero_tau)
    end
    iter_count = iter_count + 1
    if iter_count == 500:
      textmsg("[rtde_torque_ctrl] 500 iters  mode=", mode)
      iter_count = 0
    end
  end
end

rtde_torque_ctrl()
"""

URSCRIPT_STOP = """\
def rtde_stop():
  textmsg("rtde_stop")
  stopj(2.0)
end

rtde_stop()
"""

# UR robot_mode enum (from UR documentation)
_ROBOT_MODE_NAMES = {
    -1: 'UNKNOWN',
    0: 'NO_CONTROLLER', 1: 'DISCONNECTED', 2: 'CONFIRM_SAFETY',
    3: 'BOOTING', 4: 'POWER_OFF', 5: 'POWER_ON', 6: 'IDLE',
    7: 'RUNNING', 8: 'BACKDRIVE', 9: 'UPDATING_FIRMWARE',
}

_SAFETY_MODE_NAMES = {
    -1: 'UNKNOWN',
    1: 'NORMAL', 2: 'REDUCED', 3: 'PROTECTIVE_STOP',
    4: 'RECOVERY', 5: 'SAFEGUARD_STOP', 6: 'SYSTEM_EMERGENCY_STOP',
    7: 'ROBOT_EMERGENCY_STOP', 8: 'VIOLATION', 9: 'FAULT',
    10: 'VALIDATE_JOINT_ID', 11: 'UNDEFINED_SAFETY_MODE',
}


def _robot_mode_name(m):
    return _ROBOT_MODE_NAMES.get(m, f'?{m}?')


def _safety_mode_name(m):
    return _SAFETY_MODE_NAMES.get(m, f'?{m}?')


_ROBOT_PARAMS = {
    'ur10e': {
        'act_lo': np.array([-330.0, -330.0, -150.0, -56.0, -56.0, -56.0]),
        'act_hi': np.array([ 330.0,  330.0,  150.0,  56.0,  56.0,  56.0]),
    },
    'ur3e': {
        'act_lo': np.array([-54.0, -54.0, -28.0, -9.0, -9.0, -9.0]),
        'act_hi': np.array([ 54.0,  54.0,  28.0,  9.0,  9.0,  9.0]),
    },
}


class URControl:
    """RTDE-based UR robot client. Same interface as DummyControl.

    Connects to a UR robot (or ur_server_dummy.py) via RTDE binary protocol.
    A background thread continuously receives state at 500Hz.
    """

    def __init__(self, robot_ip: str = '127.0.0.1', robot_name: str = 'ur10e',
                 timestep: float = 0.002, port: int = 30004,
                 upload_urscript: bool = None,
                 secondary_port: int = UR_SECONDARY_PORT, **kwargs):
        params = _ROBOT_PARAMS[robot_name]
        self.robot_name = robot_name
        self.timestep = timestep
        self.act_lo = params['act_lo'].copy()
        self.act_hi = params['act_hi'].copy()

        self._conn = RTDEConnection(robot_ip, port=port)
        self._lock = threading.Lock()
        self._q = HOME_QPOS.copy()
        self._dq = np.zeros(N)
        self._current = np.zeros(N)
        self._tau_contact = np.zeros(N)
        self._timestamp = 0.0
        self._connected = False
        self._robot_mode = -1
        self._safety_mode = -1
        self._recv_count = 0
        # write_torque stats
        self._write_count = 0
        self._last_tau = np.zeros(N)

        self._recv_thread = None
        self._stop_event = threading.Event()

        # URScript upload: auto-disable for dummy server (localhost)
        if upload_urscript is None:
            upload_urscript = robot_ip not in ('127.0.0.1', 'localhost')
        self._upload_urscript = upload_urscript
        self._secondary_port = secondary_port

    def connect(self) -> bool:
        if not self._conn.connect():
            print(f'[URControl] Failed to connect to {self._conn.host}:{self._conn.port}')
            return False
        self._connected = True
        self._stop_event.clear()

        # Start background receive thread
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True, name='rtde-recv')
        self._recv_thread.start()

        print(f'[URControl] Connected to {self.robot_name} at '
              f'{self._conn.host}:{self._conn.port}')

        # Wait for first state packet to confirm stream is alive
        t0 = time.time()
        while self._recv_count == 0 and time.time() - t0 < 2.0:
            time.sleep(0.05)
        if self._recv_count == 0:
            print('[URControl] WARNING: no RTDE state packets received yet')
        else:
            with self._lock:
                q_now = self._q.copy()
                mode = self._robot_mode
                safety = self._safety_mode
            print(f'[URControl] first packet received '
                  f'(recv_count={self._recv_count})')
            print(f'[URControl] actual_q     = {q_now.round(4).tolist()}')
            print(f'[URControl] robot_mode   = {mode} ({_robot_mode_name(mode)})')
            print(f'[URControl] safety_mode  = {safety} '
                  f'({_safety_mode_name(safety)})')
            if mode != 7:
                print(f'[URControl] WARNING: robot not in RUNNING mode '
                      f'(need mode=7 for torque control; current={mode})')
            if safety != 1:
                print(f'[URControl] WARNING: safety_mode != NORMAL '
                      f'(current={safety})')

        # Upload URScript torque control loop to UR controller.
        # Without this, torques written to RTDE registers are never applied
        # — the robot would stay idle.
        if self._upload_urscript:
            # Initialize mode register to 0 (zero torque) BEFORE uploading,
            # so the script starts in a safe state.
            try:
                self._conn.send_input(np.zeros(N), control_mode=0)
            except Exception as e:
                print(f'[URControl] initial zero-tau send failed: {e}')
            print(f'[URControl] uploading URScript torque loop '
                  f'({len(URSCRIPT_TORQUE_CONTROL)} bytes) '
                  f'→ {self._conn.host}:{self._secondary_port}')
            if self._send_urscript(URSCRIPT_TORQUE_CONTROL):
                print('[URControl] URScript upload OK '
                      '(check UR pendant log for "[rtde_torque_ctrl] START")')
                # Give UR a moment to start the script and see if robot_mode
                # changes (running a URScript program sets mode to RUNNING=7)
                time.sleep(0.8)
                with self._lock:
                    mode_after = self._robot_mode
                print(f'[URControl] robot_mode after upload = {mode_after} '
                      f'({_robot_mode_name(mode_after)})')
            else:
                print('[URControl] WARNING: URScript upload FAILED — '
                      'robot will NOT respond to torque commands.')
        else:
            print('[URControl] URScript upload skipped (dummy/localhost)')

        return True

    def disconnect(self) -> bool:
        # Safe stop: zero torque first, then halt the URScript
        if self._connected:
            try:
                self._conn.send_input(np.zeros(N), control_mode=0)
                time.sleep(0.05)
            except Exception:
                pass
            if self._upload_urscript:
                self._send_urscript(URSCRIPT_STOP)
        self._stop_event.set()
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        self._conn.disconnect()
        self._connected = False
        print(f'[URControl] Disconnected from {self.robot_name}')
        return True

    # ---- URScript upload (UR Secondary Interface) --------------------------

    def _send_urscript(self, script: str, timeout: float = 3.0) -> bool:
        """Upload URScript text to UR Secondary Interface (port 30002).

        UR controller accepts raw URScript text on this port and immediately
        runs it, replacing any currently executing program.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((self._conn.host, self._secondary_port))
            payload = script.encode('utf-8')
            if not payload.endswith(b'\n'):
                payload += b'\n'
            sock.sendall(payload)
            sock.close()
            return True
        except Exception as e:
            print(f'[URControl] URScript send failed '
                  f'({self._conn.host}:{self._secondary_port}): {e}')
            return False

    def read_joint_state(self) -> tuple:
        """Returns (q [N], dq [N])."""
        with self._lock:
            return self._q.copy(), self._dq.copy()

    def read_gravity_compensation(self) -> np.ndarray:
        """Returns tau_grav [N].
        For real UR: firmware handles gravity internally (return zeros).
        """
        return np.zeros(N)

    def read_contact_forces(self) -> np.ndarray:
        """Returns tau_contact [N].
        Currently zeros — TODO: derive from actual_TCP_force when available.
        """
        with self._lock:
            return self._tau_contact.copy()

    def read_eef_pose(self) -> tuple:
        """Placeholder FK — TODO: implement from UR RTDE actual_TCP_pose."""
        return np.array([0.0, 0.0, 0.5]), np.eye(3)

    def write_torque(self, tau: np.ndarray) -> bool:
        """Send torque command to robot via RTDE input data package."""
        if not self._connected:
            return False
        ok = self._conn.send_input(tau, control_mode=1)
        # Log every ~500 writes (=1s at 500Hz) + first call
        self._write_count += 1
        self._last_tau = np.asarray(tau).copy()
        if self._write_count == 1:
            print(f'[URControl] first write_torque  tau='
                  f'{self._last_tau.round(3).tolist()}  ok={ok}')
        elif self._write_count % 500 == 0:
            with self._lock:
                mode = self._robot_mode
                safety = self._safety_mode
                recv = self._recv_count
            print(f'[URControl] writes={self._write_count}  '
                  f'|tau|max={np.abs(self._last_tau).max():.2f}  '
                  f'recv={recv}  robot_mode={mode}  safety={safety}')
        return ok

    def read_status(self) -> dict:
        """Return robot_mode, safety_mode, recv_count, write_count."""
        with self._lock:
            return {
                'robot_mode': self._robot_mode,
                'safety_mode': self._safety_mode,
                'recv_count': self._recv_count,
                'write_count': self._write_count,
                'last_tau': self._last_tau.copy(),
            }

    # ---- Background receive thread -----------------------------------------

    def _recv_loop(self):
        """Continuously receive RTDE output data and cache it."""
        while not self._stop_event.is_set():
            data = self._conn.receive_output()
            if data is not None:
                with self._lock:
                    if 'actual_q' in data:
                        self._q[:] = data['actual_q']
                    if 'actual_qd' in data:
                        self._dq[:] = data['actual_qd']
                    if 'actual_current' in data:
                        self._current[:] = data['actual_current']
                    if 'timestamp' in data:
                        self._timestamp = data['timestamp']
                    if 'robot_mode' in data:
                        self._robot_mode = data['robot_mode']
                    if 'safety_mode' in data:
                        self._safety_mode = data['safety_mode']
                    self._recv_count += 1
