"""
environment_sensing_data_emulator.py — Virtual environment for dummy robot testing.

Provides:
  1. Contact force simulation — virtual walls at configurable joint limits
  2. Pose-dependent gravity compensation (approximate)
  3. Motor echo — current state accessible for monitoring

Usage:
    emu = EnvironmentEmulator(robot_name='ur10e')
    emu.add_wall(joint_idx=0, limit=0.0, stiffness=500.0)  # wall at q[0] > 0
    tau_contact = emu.compute_contact_forces(q)
    tau_grav    = emu.compute_gravity(q)
"""

import numpy as np

N = 6

# Approximate gravity parameters per robot (mass * g * lever_arm at key poses)
_GRAVITY_PARAMS = {
    'ur10e': {
        # Masses and lever arms for gravity approximation
        # Gravity primarily affects joints 1 (shoulder_lift) and 2 (elbow)
        # tau_grav[i] ≈ sum of (mass_j * g * lever_arm_j) for all links beyond joint i
        'link_mass': np.array([0.0, 20.0, 12.0, 3.0, 2.0, 1.0]),   # kg approx
        'link_cog_dist': np.array([0.0, 0.4, 0.3, 0.1, 0.05, 0.02]),  # m from joint
        'g': 9.81,
    },
    'ur3e': {
        'link_mass': np.array([0.0, 4.0, 2.5, 0.6, 0.4, 0.2]),
        'link_cog_dist': np.array([0.0, 0.15, 0.12, 0.05, 0.03, 0.01]),
        'g': 9.81,
    },
}


class EnvironmentEmulator:
    """Virtual environment that generates contact forces and gravity."""

    def __init__(self, robot_name: str = 'ur10e'):
        self.robot_name = robot_name
        self._walls = []  # [(joint_idx, limit_rad, direction, stiffness)]
        self._grav_params = _GRAVITY_PARAMS[robot_name]

    def add_wall(self, joint_idx: int, limit: float,
                 stiffness: float = 500.0, direction: str = 'upper'):
        """Add a virtual wall at a joint limit.

        Args:
            joint_idx: Joint index (0-5)
            limit: Joint angle limit in radians
            stiffness: Wall stiffness in Nm/rad (spring constant)
            direction: 'upper' (wall above limit) or 'lower' (wall below limit)
        """
        self._walls.append((joint_idx, limit, stiffness, direction))

    def clear_walls(self):
        """Remove all virtual walls."""
        self._walls.clear()

    def compute_contact_forces(self, q: np.ndarray) -> np.ndarray:
        """Compute contact forces from virtual walls.

        Args:
            q: Current joint positions [N]

        Returns:
            tau_contact [N] — reaction torques from wall contacts
        """
        tau = np.zeros(N)
        for idx, limit, stiffness, direction in self._walls:
            if direction == 'upper':
                penetration = q[idx] - limit
                if penetration > 0:
                    tau[idx] = -stiffness * penetration
            else:  # lower
                penetration = limit - q[idx]
                if penetration > 0:
                    tau[idx] = stiffness * penetration
        return tau

    def compute_gravity(self, q: np.ndarray) -> np.ndarray:
        """Compute approximate gravity compensation torques.

        Simplified model: gravity primarily loads joints 1 and 2 based on
        shoulder_lift and elbow angles.  Joint 0 (shoulder_pan, vertical axis)
        has negligible gravity torque.

        Args:
            q: Current joint positions [N]

        Returns:
            tau_grav [N] — torques needed to hold against gravity
        """
        g = self._grav_params['g']
        mass = self._grav_params['link_mass']
        dist = self._grav_params['link_cog_dist']

        tau = np.zeros(N)

        # Joint 0: shoulder_pan (vertical axis) — no gravity torque
        tau[0] = 0.0

        # Joint 1: shoulder_lift — supports all downstream links
        # Torque depends on cos(q[1]) (gravity component perpendicular to link)
        total_downstream_1 = np.sum(mass[1:] * dist[1:])
        tau[1] = -g * total_downstream_1 * np.cos(q[1])

        # Joint 2: elbow — supports links 2 onwards
        total_downstream_2 = np.sum(mass[2:] * dist[2:])
        tau[2] = -g * total_downstream_2 * np.cos(q[1] + q[2])

        # Joint 3: wrist_1 — small gravity effect
        total_downstream_3 = np.sum(mass[3:] * dist[3:])
        tau[3] = -g * total_downstream_3 * np.cos(q[1] + q[2] + q[3])

        # Joints 4-5: negligible
        tau[4] = 0.0
        tau[5] = 0.0

        return tau
