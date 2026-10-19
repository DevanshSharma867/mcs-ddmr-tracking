"""Reproduced controller: adaptive fast nonsingular terminal sliding mode dynamic control (AFNTSMDC).

Source: Xie et al., "Finite-time tracking control for nonholonomic wheeled mobile robot using
adaptive fast nonsingular terminal sliding mode", Nonlinear Dyn. 110(2), 1437-1453, 2022.
Combined controller: pose + velocity errors -> wheel inputs (Fig. 2). Uses nominal m0, I0, r, b.

    xi, xi_dot, H, drift = H_dot p_e_dot + H (F_dot + G_dot z), D = H G M0^-1 B   eq. (22)-(26)
    s      = xi_dot + alpha xi + beta sig(xi)^gamma                                 eq. (27)
    tau_eq = -D^-1 (drift + alpha xi_dot + beta gamma |xi|^(gamma-1) xi_dot)        eq. (31)
    tau_r  = -D^-1 (K1 s + K2 sig(s)^mu) + tau_sw                                    eq. (32)
    L      = diag(sat(s1), sat(s2))   (eq. 33 with the boundary layer of eq. 55, M9)
    C_hat  = [c1 + c3 |v|, c2 + c4 |omega|]^T                                       eq. (34)
    tau    = tau_eq + tau_r                                                         eq. (39)
Adaptive law (37)-(38) with dead zone (53)-(54), forward Euler at Ts, applied after tau:
    c1 += Ts zeta1 |s1|,  c3 += Ts zeta3 |v| |s1|     only if |s1| > eps
    c2 += Ts zeta2 |s2|,  c4 += Ts zeta4 |omega| |s2| only if |s2| > eps
Paper gains (Sect. 3.3): alpha = diag(7, 7), beta = diag(7, 6), gamma = 1.67, rho = 0.8,
K1 = diag(9, 4), K2 = diag(15, 5), mu = 0.8, c(0) = (14, 14, 8, 6) [c4(0) ASSUMED, M13],
zeta = (0.3, 0.4, 0.4, 0.5), eps = 0.05, varrho = 0.04.
Numerical guards (M14): sig and |.|^a clip |x| at 1e-6, sgn(0) = 0 (see `switching`).
boundary_layer=None uses pure sgn in L (diagnostic only).

Adaptive switching term tau_sw (DEVIATION, docs/reproduction_log.md "AFNTSMDC switching sign"):
    "printed":   tau_sw = -B^-1 M0 L C_hat  -> s_dot gets -HG L C_hat ~ +L C_hat (HG ~ -I near
                 the origin): destabilizing. Literal reproduction only.
    "corrected": tau_sw = +B^-1 M0 L C_hat  -> s_dot gets +HG L C_hat ~ -L C_hat. Sign-corrected;
                 THE reproduced controller (valid where HG ~ -I).
    "general":   tau_sw = -D^-1 L C_hat     -> s_dot gets exactly -L C_hat for any HG.
    "none":      tau_sw = 0 (diagnostic; adaptation still runs and is logged).
"""

from __future__ import annotations

from typing import Literal

import numpy as np

from ddmr.controllers.base import Controller
from ddmr.controllers.switching import abs_pow, sat, sig
from ddmr.controllers.xi_terms import xi_terms
from ddmr.errors import tracking_error
from ddmr.model import RobotParams
from ddmr.trajectories import RefSample


class AFNTSMDC(Controller):
    output = "torque"

    def __init__(self, params: RobotParams, Ts: float, alpha: tuple[float, float],
                 beta: tuple[float, float], gamma: float, rho: float, K1: tuple[float, float],
                 K2: tuple[float, float], mu: float, c0: tuple[float, float, float, float],
                 zeta: tuple[float, float, float, float], eps: float,
                 boundary_layer: float | tuple[float, float] | None,
                 switching: Literal["printed", "corrected", "general", "none"] = "corrected") -> None:
        if switching not in ("printed", "corrected", "general", "none"):
            raise ValueError(f"unknown switching variant {switching!r}")
        self.params, self.Ts, self.rho, self.switching = params, Ts, rho, switching
        self.alpha, self.beta, self.gamma = np.asarray(alpha, float), np.asarray(beta, float), gamma
        self.K1, self.K2, self.mu = np.asarray(K1, float), np.asarray(K2, float), mu
        self.c0, self.zeta = np.asarray(c0, float), np.asarray(zeta, float)
        self.eps = eps
        self.boundary_layer = None if boundary_layer is None else np.asarray(boundary_layer, float)
        self.B_inv_M0 = np.linalg.inv(params.B) @ np.diag([params.m, params.I])
        self.reset()

    def reset(self) -> None:
        self.c = self.c0.copy()
        self._diag: dict[str, float] = {}

    def sliding_variable(self, xi: np.ndarray, xi_dot: np.ndarray) -> np.ndarray:
        """Fast nonsingular terminal sliding surface, eq. (27)."""
        return xi_dot + self.alpha * xi + self.beta * sig(xi, self.gamma)

    def compute(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        v, w = state[3:5]
        xt = xi_terms(tracking_error(state, ref.pose), state[3:5], ref, self.rho, self.params)
        s = self.sliding_variable(xt.xi, xt.xi_dot)

        eq_rhs = (xt.drift + self.alpha * xt.xi_dot
                  + self.beta * self.gamma * abs_pow(xt.xi, self.gamma - 1.0) * xt.xi_dot)
        tau_eq = -np.linalg.solve(xt.D, eq_rhs)
        L = np.sign(s) if self.boundary_layer is None else sat(s, self.boundary_layer)
        C_hat = self.C_hat(v, w)
        LC = L * C_hat
        if self.switching == "printed":
            tau_sw = -self.B_inv_M0 @ LC
        elif self.switching == "corrected":
            tau_sw = self.B_inv_M0 @ LC
        elif self.switching == "general":
            tau_sw = -np.linalg.solve(xt.D, LC)
        else:
            tau_sw = np.zeros(2)
        tau_r = -np.linalg.solve(xt.D, self.K1 * s + self.K2 * sig(s, self.mu)) + tau_sw
        tau = tau_eq + tau_r + self.extra_torque(state, tau_eq + tau_r)

        self._diag = {**self._diag, "s1": s[0], "s2": s[1], "c1": self.c[0], "c2": self.c[1], "c3": self.c[2],
                      "c4": self.c[3], "L1": L[0], "L2": L[1], "sw_R": tau_sw[0], "sw_L": tau_sw[1],
                      "eq_R": tau_eq[0], "eq_L": tau_eq[1], "r_R": tau_r[0], "r_L": tau_r[1]}

        self.adapt(s, v, w)
        return tau

    def C_hat(self, v: float, w: float) -> np.ndarray:
        """Uncertainty bound estimate, eq. (34)."""
        return np.array([self.c[0] + self.c[2] * abs(v), self.c[1] + self.c[3] * abs(w)])

    def extra_torque(self, state: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """Additional input on top of eq. (39); none in the paper's law (hook for Phase 6)."""
        return np.zeros(2)

    def adapt(self, s: np.ndarray, v: float, w: float) -> None:
        """Adaptive law (37)-(38) with dead zone (53)-(54), forward Euler at Ts."""
        active = np.abs(s) > self.eps
        if active[0]:
            self.c[0] += self.Ts * self.zeta[0] * abs(s[0])
            self.c[2] += self.Ts * self.zeta[2] * abs(v) * abs(s[0])
        if active[1]:
            self.c[1] += self.Ts * self.zeta[1] * abs(s[1])
            self.c[3] += self.Ts * self.zeta[3] * abs(w) * abs(s[1])

    def diagnostics(self) -> dict[str, float]:
        return self._diag
