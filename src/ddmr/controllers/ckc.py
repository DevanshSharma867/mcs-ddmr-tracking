"""Classic kinematic control (CKC), Kanayama-type outer loop, paper eq. (56)-(57) [19].

    z_d = K_E p_e + K_D,   K_E = [[k_e1, 0, 0], [0, v_r k_e2, 0]],
                           K_D = [v_r cos(phi_e), v_r k_e3 sin(phi_e) + omega_r]^T
    =>  v_d     = v_r cos(phi_e) + k_e1 x_e
        omega_d = omega_r + k_e2 v_r y_e + k_e3 v_r sin(phi_e)

Paper gains: k_e1 = 1.7, k_e2 = 1.7, k_e3 = 1.5. Outputs velocities: run alone it is the ideal
kinematic reference (perfect velocity tracking); wrapped in `CascadePI` it is CKC+PI.
"""

from __future__ import annotations

import math

import numpy as np

from ddmr.controllers.base import Controller
from ddmr.errors import tracking_error
from ddmr.trajectories import RefSample


class CKC(Controller):
    output = "velocity"

    def __init__(self, k_e1: float, k_e2: float, k_e3: float) -> None:
        self.k_e1, self.k_e2, self.k_e3 = k_e1, k_e2, k_e3

    def reset(self) -> None:
        pass

    def compute(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        x_e, y_e, phi_e = tracking_error(state, ref.pose)
        return np.array([ref.v * math.cos(phi_e) + self.k_e1 * x_e,
                         ref.omega + self.k_e2 * ref.v * y_e + self.k_e3 * ref.v * math.sin(phi_e)])
