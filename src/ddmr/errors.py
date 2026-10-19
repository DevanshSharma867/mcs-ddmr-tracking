"""Tracking error in the robot frame (paper Sect. 3.1).

    p_e = [x_e, y_e, phi_e]^T = R(phi)^T (p_r - p)                       eq. (14)
        x_e   =  cos(phi) (x_r - x) + sin(phi) (y_r - y)
        y_e   = -sin(phi) (x_r - x) + cos(phi) (y_r - y)
        phi_e = wrap(phi_r - phi)    in (-pi, pi]   (wrapping ASSUMED, M11)

    p_e_dot = F + G z                                                     eq. (18)-(19)
        F = [v_r cos(phi_e), v_r sin(phi_e), omega_r]^T
        G = [[-1, y_e], [0, -x_e], [0, -1]]
"""

from __future__ import annotations

import math

import numpy as np

from ddmr.trajectories import RefSample


def wrap_angle(a: float | np.ndarray) -> float | np.ndarray:
    """Wrap an angle [rad] to (-pi, pi]."""
    return math.pi - np.mod(math.pi - a, 2 * math.pi)


def tracking_error(pose: np.ndarray, ref_pose: np.ndarray) -> np.ndarray:
    """Robot-frame error p_e = [x_e, y_e, phi_e] of eq. (14) for pose = [x, y, phi]."""
    x, y, phi = pose[:3]
    dx, dy = ref_pose[0] - x, ref_pose[1] - y
    c, s = math.cos(phi), math.sin(phi)
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap_angle(ref_pose[2] - phi)])


def error_rate(p_e: np.ndarray, z: np.ndarray, ref: RefSample) -> np.ndarray:
    """Error dynamics p_e_dot = F + G z, eq. (18)-(19), for body velocities z = [v, omega]."""
    x_e, y_e, phi_e = p_e
    v, w = z
    return np.array([ref.v * math.cos(phi_e) - v + y_e * w,
                     ref.v * math.sin(phi_e) - x_e * w,
                     ref.omega - w])
