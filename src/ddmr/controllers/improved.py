"""Our improvement over the reproduced AFNTSMDC (Phase 6; docs/phase6_preregistration.md).

Two optional components on top of `AFNTSMDC` (sign-corrected, paper gains):

ESO (extended state observer, ADRC literature; bandwidth parameterization), per channel j in {v, omega},
on the model z_dot = w + f with w = M0^-1 B sat(tau) and lumped f = -M0^-1 delta (eq. 11):
    z_hat_{k+1} = z_hat_k + Ts (w_k + f_hat_k + beta1 (y_k - z_hat_k))
    f_hat_{k+1} = f_hat_k + Ts beta2 (y_k - z_hat_k),    beta1 = 2 omega_o, beta2 = omega_o^2
(discrete poles 1 - omega_o Ts, double). Compensation tau_eso = -B^-1 M0 f_hat, so z_dot = ... + f - f_hat
and the residual enters s_dot as HG (f - f_hat) for any HG. The observer is fed the saturated command
(u_max known to the controller), so it does not wind up.

PS (projection + bounded C_hat + sized boundary layer):
    c_hat_i in [c_min_i, c_max_i]  (adaptive law (37)-(38) + dead zone, update clipped at the bounds)
    C_hat_j = min(c_j + c_{j+2} |z_j|, C_max)
    switching term in the general form tau_sw = -D^-1 L C_hat (s_dot gets exactly -L C_hat)
    varrho_j = kappa_p C_max Ts / (1 - kappa_p K1_j Ts)  <=>  kappa_p (K1_j + C_max / varrho_j) Ts <= 1,
i.e. monotone (sign-preserving) discrete convergence of s inside the layer for plant/model gain
ratios up to kappa_p.
"""

from __future__ import annotations

from typing import Literal

import numpy as np

from ddmr.controllers.afntsm import AFNTSMDC
from ddmr.model import RobotParams


def sized_boundary_layer(C_max: float, K1: np.ndarray, Ts: float, kappa_p: float) -> np.ndarray:
    """varrho_j = kappa_p C_max Ts / (1 - kappa_p K1_j Ts); requires kappa_p K1_j Ts < 1."""
    K1 = np.asarray(K1, float)
    if np.any(kappa_p * K1 * Ts >= 1.0):
        raise ValueError("kappa_p K1 Ts must be < 1 for a monotone boundary layer")
    return kappa_p * C_max * Ts / (1.0 - kappa_p * K1 * Ts)


class ImprovedAFNTSMDC(AFNTSMDC):
    """AFNTSMDC + optional ESO compensation + optional projection / bounded C_hat / sized layer."""

    def __init__(self, params: RobotParams, Ts: float, u_max: float | None, afntsmdc: dict,
                 use_eso: bool, use_ps: bool, switching: Literal["corrected", "general", "none"],
                 eso_omega: float, c_min: tuple[float, ...], c_max: tuple[float, ...],
                 c0_ps: tuple[float, ...], C_max: float, kappa_p: float) -> None:
        g = dict(afntsmdc)
        g["switching"] = switching
        if use_ps:
            g["c0"] = c0_ps
            g["boundary_layer"] = tuple(sized_boundary_layer(C_max, g["K1"], Ts, kappa_p))
        self.use_eso, self.use_ps, self.u_max = use_eso, use_ps, u_max
        self.eso_omega = eso_omega
        self.c_min, self.c_max, self.C_max = np.asarray(c_min, float), np.asarray(c_max, float), C_max
        self.M0_inv_B = np.diag([1.0 / params.m, 1.0 / params.I]) @ params.B
        super().__init__(params, Ts, **g)

    def reset(self) -> None:
        super().reset()
        self.z_hat: np.ndarray | None = None
        self.f_hat = np.zeros(2)

    def C_hat(self, v: float, w: float) -> np.ndarray:
        C = super().C_hat(v, w)
        return np.minimum(C, self.C_max) if self.use_ps else C

    def adapt(self, s: np.ndarray, v: float, w: float) -> None:
        super().adapt(s, v, w)
        if self.use_ps:
            self.c = np.clip(self.c, self.c_min, self.c_max)     # projection onto the box

    def extra_torque(self, state: np.ndarray, tau: np.ndarray) -> np.ndarray:
        if not self.use_eso:
            return np.zeros(2)
        y = state[3:5]
        if self.z_hat is None:
            self.z_hat = y.copy()
        tau_eso = -self.B_inv_M0 @ self.f_hat
        u = tau + tau_eso
        if self.u_max is not None:
            u = np.clip(u, -self.u_max, self.u_max)
        w_in = self.M0_inv_B @ u
        e = y - self.z_hat
        self._diag = {**self._diag, "f_hat1": self.f_hat[0], "f_hat2": self.f_hat[1]}
        self.z_hat = self.z_hat + self.Ts * (w_in + self.f_hat + 2.0 * self.eso_omega * e)
        self.f_hat = self.f_hat + self.Ts * self.eso_omega**2 * e
        return tau_eso
