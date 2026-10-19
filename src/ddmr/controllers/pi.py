"""PI velocity inner loop for the cascaded baselines (CKC+PI, RSMKC+PI), paper eq. (59).

    z_e = z_d - z,   u = k_P z_e + k_I * integral(z_e dt),   tau = B^{-1} u

u = [u_v, u_omega] is a generalized force/torque in the (v, omega) channels of eq. (8); the wheel
inputs are tau = B^{-1} u with the nominal B of eq. (9) (only r, b; no m, I). This mapping is
ASSUMED (docs/reproduction_log.md M12): the literal "u1 -> right, u2 -> left" is unstable in our
model units. Paper gains: k_P = 30, k_I = 250. Rectangular integration at Ts.
Clamping anti-windup: if the new output exceeds u_max on either wheel, that tick's integrator
update is discarded.
"""

from __future__ import annotations

import numpy as np

from ddmr.controllers.base import Controller
from ddmr.model import RobotParams
from ddmr.trajectories import RefSample


class CascadePI(Controller):
    output = "torque"

    def __init__(self, outer: Controller, params: RobotParams, k_P: float, k_I: float,
                 Ts: float, u_max: float | None) -> None:
        assert outer.output == "velocity", "outer loop must output [v_d, omega_d]"
        self.outer, self.k_P, self.k_I, self.Ts, self.u_max = outer, k_P, k_I, Ts, u_max
        self.B_inv = np.linalg.inv(params.B)
        self.integral = np.zeros(2)

    def reset(self) -> None:
        self.outer.reset()
        self.integral = np.zeros(2)

    def compute(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        z_e = self.outer.compute(t, state, ref) - state[3:5]
        new_integral = self.integral + z_e * self.Ts
        tau = self.B_inv @ (self.k_P * z_e + self.k_I * new_integral)
        if self.u_max is not None and np.any(np.abs(tau) > self.u_max):
            tau = self.B_inv @ (self.k_P * z_e + self.k_I * self.integral)
        else:
            self.integral = new_integral
        return tau
