"""Conventional sliding-mode dynamic control (CSMDC), combined controller, paper eq. (60).

    s_csm = lambda_c xi + xi_dot
    tau_eq2 = -(H G M0^-1 B)^-1 (lambda_c xi_dot + H_dot p_e_dot + H (F_dot + G_dot z))
    tau_r2  = -(H G M0^-1 B)^-1 K_c sgn(s_csm)
    tau     = tau_eq2 + tau_r2

xi, H, ... from eq. (22)-(26) (`xi_terms`); the printed "G_dot V" is read as G_dot z.
With no uncertainty this gives s_dot = -K_c sgn(s). Paper gains: K_c = diag(21, 16),
lambda_c = diag(3, 4). rho = 0.8 is ASSUMED (the paper gives rho only for AFNTSMDC).
Pure sgn, no boundary layer (M9). Uses nominal parameters m0, I0, r, b.

Diagnostic variant (NOT the paper baseline): boundary_layer = varrho replaces sgn(s) with
sat(s / varrho) of eq. (55), using AFNTSMDC's varrho = 0.04, to separate the boundary-layer effect
from the terminal surface / adaptation in Phase 4.
"""

from __future__ import annotations

import numpy as np

from ddmr.controllers.base import Controller
from ddmr.controllers.switching import sat
from ddmr.controllers.xi_terms import xi_terms
from ddmr.errors import tracking_error
from ddmr.model import RobotParams
from ddmr.trajectories import RefSample


class CSMDC(Controller):
    output = "torque"

    def __init__(self, params: RobotParams, k_c: tuple[float, float],
                 lambda_c: tuple[float, float], rho: float,
                 boundary_layer: float | None = None) -> None:
        self.params, self.rho, self.boundary_layer = params, rho, boundary_layer
        self.K_c, self.lambda_c = np.diag(k_c), np.diag(lambda_c)

    def reset(self) -> None:
        self._diag: dict[str, float] = {}

    def sliding_variable(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        xt = xi_terms(tracking_error(state, ref.pose), state[3:5], ref, self.rho, self.params)
        return self.lambda_c @ xt.xi + xt.xi_dot

    def compute(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        xt = xi_terms(tracking_error(state, ref.pose), state[3:5], ref, self.rho, self.params)
        s = self.lambda_c @ xt.xi + xt.xi_dot
        switch = np.sign(s) if self.boundary_layer is None else sat(s, self.boundary_layer)
        rhs = self.lambda_c @ xt.xi_dot + xt.drift + self.K_c @ switch
        self._diag = {"s1": s[0], "s2": s[1], "L1": switch[0], "L2": switch[1]}
        return -np.linalg.solve(xt.D, rhs)

    def diagnostics(self) -> dict[str, float]:
        return self._diag
