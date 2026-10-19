"""Robust sliding-mode kinematic control (RSMKC), outer loop, paper eq. (58) (Nath et al. [15]).

    s_r = [k_r1 phi_e + v_r atan(y_e),  x_e]^T
    z_d = -B_r^{-1} (A_r F_r + D_r + K_r sgn(s_r))
    A_r = [[0, v_r/(1+y_e^2), k_r1], [1, 0, 0]]
    B_r = [[0, -(k_r1 + v_r x_e/(1+y_e^2))], [-1, y_e]]
    F_r = [v_r cos(phi_e), v_r sin(phi_e), omega_r]^T,  D_r = [v_r_dot atan(y_e), 0]^T,
    K_r = diag(k_r2, k_r3)

With the error dynamics (18), s_r_dot = A_r F_r + B_r z + D_r, so z = z_d gives
s_r_dot = -K_r sgn(s_r). Paper gains: k_r1 = 0.40, k_r2 = k_r3 = 0.02. Pure sgn (M9), sgn(0) = 0.
B_r is singular where k_r1 + v_r x_e/(1+y_e^2) = 0; this raises instead of returning NaN.
"""

from __future__ import annotations

import math

import numpy as np

from ddmr.controllers.base import Controller
from ddmr.errors import tracking_error
from ddmr.trajectories import RefSample


class RSMKC(Controller):
    output = "velocity"

    def __init__(self, k_r1: float, k_r2: float, k_r3: float) -> None:
        self.k_r1 = k_r1
        self.K_r = np.diag([k_r2, k_r3])

    def reset(self) -> None:
        pass

    def sliding_variable(self, p_e: np.ndarray, ref: RefSample) -> np.ndarray:
        x_e, y_e, phi_e = p_e
        return np.array([self.k_r1 * phi_e + ref.v * math.atan(y_e), x_e])

    def compute(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        p_e = tracking_error(state, ref.pose)
        x_e, y_e, phi_e = p_e
        q = 1.0 + y_e**2
        b12 = -(self.k_r1 + ref.v * x_e / q)
        if abs(b12) < 1e-6:
            raise ZeroDivisionError(f"RSMKC: B_r singular at t={t} (x_e={x_e:.3f})")
        A_r = np.array([[0.0, ref.v / q, self.k_r1], [1.0, 0.0, 0.0]])
        B_r = np.array([[0.0, b12], [-1.0, y_e]])
        F_r = np.array([ref.v * math.cos(phi_e), ref.v * math.sin(phi_e), ref.omega])
        D_r = np.array([ref.v_dot * math.atan(y_e), 0.0])
        s_r = self.sliding_variable(p_e, ref)
        return -np.linalg.solve(B_r, A_r @ F_r + D_r + self.K_r @ np.sign(s_r))
