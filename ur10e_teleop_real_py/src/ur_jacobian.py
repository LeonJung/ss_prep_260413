"""
ur_jacobian.py — analytical geometric Jacobian + forward kinematics for UR
e-Series robots (UR3e / UR10e / UR5e etc.).

No external deps beyond numpy. Used by the follower node to:
  (a) compute TCP position from actual_q
  (b) map a Cartesian contact/virtual-wall force back to joint-space
      torque (J^T · F) for haptic feedback

DH parameters from Universal Robots documentation.
"""

import numpy as np


# ----------------------------------------------------------------------------
# DH parameters per robot (modified DH / standard — using standard here)
#   alpha[i], a[i], d[i], theta[i] per joint
# Source: UR support articles; theta[i] = q[i] + theta_offset (zero for all).
# ----------------------------------------------------------------------------
_DH = {
    'ur3e': dict(
        a     = [0.0, -0.24355, -0.21325, 0.0, 0.0, 0.0],
        d     = [0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921],
        alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0],
    ),
    'ur10e': dict(
        a     = [0.0, -0.6127, -0.5716, 0.0, 0.0, 0.0],
        d     = [0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655],
        alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0],
    ),
    'ur5e': dict(
        a     = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0],
        d     = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996],
        alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0],
    ),
}


def _dh_T(a, alpha, d, theta):
    """Standard-DH transform for one link: T = Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta),  np.sin(theta)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def forward_kinematics(q, robot='ur10e'):
    """Return (T_0_6, [T_0_1, T_0_2, ..., T_0_6]) for the given joint vector."""
    dh = _DH[robot]
    T  = np.eye(4)
    Ts = []
    for i in range(6):
        Ti = _dh_T(dh['a'][i], dh['alpha'][i], dh['d'][i], q[i])
        T = T @ Ti
        Ts.append(T.copy())
    return T, Ts


def ur_jacobian(q, robot='ur10e'):
    """Geometric Jacobian J (6x6) at the end-effector.
    Columns [i]: [z_{i-1} × (p_n - p_{i-1}); z_{i-1}]  for revolute joints.
    Returns J such that  [v; w] = J @ qdot,  where v/w are TCP linear/angular
    velocity expressed in the base frame.
    """
    _, Ts = forward_kinematics(q, robot)
    p_n = Ts[-1][:3, 3]
    J = np.zeros((6, 6))

    # joint 0 axis is base's +z
    z_prev = np.array([0.0, 0.0, 1.0])
    p_prev = np.zeros(3)
    for i in range(6):
        J[:3, i] = np.cross(z_prev, p_n - p_prev)
        J[3:, i] = z_prev
        # advance to next joint's frame
        z_prev = Ts[i][:3, 2]
        p_prev = Ts[i][:3, 3]
    return J


def tcp_position(q, robot='ur10e'):
    """Shortcut returning just the 3-vector TCP position in base frame."""
    T, _ = forward_kinematics(q, robot)
    return T[:3, 3].copy()
