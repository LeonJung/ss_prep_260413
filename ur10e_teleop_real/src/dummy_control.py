"""
dummy_control.py — Dummy robot client for testing without hardware.

Simulates a UR robot with simple rigid-body dynamics (M*ddq = tau - g - contact).
Uses EnvironmentEmulator for gravity and contact force computation.
Replace this class with a real RTDE/URScript client for actual hardware.

Usage:
    from environment_sensing_data_emulator import EnvironmentEmulator
    emu = EnvironmentEmulator('ur10e')
    emu.add_wall(0, 0.0, stiffness=500.0)

    robot = DummyControl(robot_name='ur10e', emulator=emu)
    robot.connect()
    q, dq = robot.read_joint_state()
    robot.write_torque(tau)
    robot.disconnect()
"""

import threading
import numpy as np

N = 6

HOME_QPOS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

_ROBOT_PARAMS = {
    'ur10e': {
        'act_lo': np.array([-330.0, -330.0, -150.0, -56.0, -56.0, -56.0]),
        'act_hi': np.array([ 330.0,  330.0,  150.0,  56.0,  56.0,  56.0]),
        'inertia': np.array([2.8, 6.8, 2.2, 0.14, 0.11, 0.10]),
        'damping': np.array([10.0, 10.0, 8.0, 4.0, 4.0, 2.0]),
    },
    'ur3e': {
        'act_lo': np.array([-54.0, -54.0, -28.0, -9.0, -9.0, -9.0]),
        'act_hi': np.array([ 54.0,  54.0,  28.0,  9.0,  9.0,  9.0]),
        'inertia': np.array([0.5, 1.2, 0.4, 0.03, 0.02, 0.02]),
        'damping': np.array([5.0, 5.0, 3.0, 1.0, 1.0, 0.5]),
    },
}


class DummyControl:
    """Dummy robot client with environment emulator integration.

    Thread-safe: read/write can be called from different threads.
    """

    def __init__(self, robot_name: str = 'ur10e', timestep: float = 0.002,
                 emulator=None):
        """
        Args:
            robot_name: 'ur10e' or 'ur3e'
            timestep: Control loop timestep in seconds
            emulator: EnvironmentEmulator instance (optional).
                      If None, gravity is constant and contact forces are zero.
        """
        params = _ROBOT_PARAMS[robot_name]
        self.robot_name = robot_name
        self.timestep = timestep
        self.act_lo = params['act_lo'].copy()
        self.act_hi = params['act_hi'].copy()
        self._inertia = params['inertia'].copy()
        self._damping = params['damping'].copy()
        self._emulator = emulator

        self._lock = threading.Lock()
        self._q = HOME_QPOS.copy()
        self._dq = np.zeros(N)
        self._tau_applied = np.zeros(N)
        self._tau_contact = np.zeros(N)
        self._connected = False

    def connect(self) -> bool:
        self._connected = True
        emu_str = 'with emulator' if self._emulator else 'no emulator'
        print(f'[DummyControl] Connected to {self.robot_name} '
              f'(dummy, {emu_str})')
        return True

    def disconnect(self) -> bool:
        self._connected = False
        print(f'[DummyControl] Disconnected from {self.robot_name}')
        return True

    def read_joint_state(self) -> tuple:
        """Returns (q [N], dq [N])."""
        with self._lock:
            return self._q.copy(), self._dq.copy()

    def read_gravity_compensation(self) -> np.ndarray:
        """Returns tau_grav [N] — gravity torques at current pose."""
        with self._lock:
            q = self._q.copy()
        if self._emulator is not None:
            return self._emulator.compute_gravity(q)
        # Fallback: constant approximation
        return np.array([0.0, -80.0, -30.0, -5.0, 0.0, 0.0])

    def read_contact_forces(self) -> np.ndarray:
        """Returns tau_contact [N] — contact forces from virtual walls."""
        with self._lock:
            return self._tau_contact.copy()

    def read_eef_pose(self) -> tuple:
        """Returns (xyz [3], mat [3,3]) — placeholder FK."""
        xyz = np.array([0.0, 0.0, 0.5])
        mat = np.eye(3)
        return xyz, mat

    def write_torque(self, tau: np.ndarray) -> bool:
        """Apply control torques and step dynamics.

        Integrates: ddq = (tau + tau_contact - tau_grav - damping*dq) / inertia
        """
        dt = self.timestep
        with self._lock:
            q = self._q
            dq = self._dq

            # Gravity from emulator or constant
            if self._emulator is not None:
                tau_grav = self._emulator.compute_gravity(q)
                tau_contact = self._emulator.compute_contact_forces(q)
            else:
                tau_grav = np.array([0.0, -80.0, -30.0, -5.0, 0.0, 0.0])
                tau_contact = np.zeros(N)

            self._tau_contact = tau_contact.copy()

            # Net torque
            tau_net = tau + tau_contact - tau_grav - self._damping * dq
            ddq = tau_net / self._inertia
            dq += ddq * dt
            q += dq * dt

            self._tau_applied = tau.copy()
        return True
