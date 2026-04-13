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

import threading
import time

import numpy as np

from rtde_connection import RTDEConnection

N = 6

HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

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
                 timestep: float = 0.002, port: int = 30004, **kwargs):
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

        self._recv_thread = None
        self._stop_event = threading.Event()

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
        return True

    def disconnect(self) -> bool:
        self._stop_event.set()
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        self._conn.disconnect()
        self._connected = False
        print(f'[URControl] Disconnected from {self.robot_name}')
        return True

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
        return self._conn.send_input(tau, control_mode=1)

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
