"""Plant model of the differential-drive robot (paper Sect. 2).

State: [x, y, phi, v, omega] (m, m, rad, m/s, rad/s); (x, y) is the axle midpoint O_l.

Kinematics, eq. (4)-(5):
    x_dot = v cos(phi),  y_dot = v sin(phi),  phi_dot = omega

Dynamics, eq. (8)-(9), with wheel inputs u = [u_R, u_L] in model units (paper: V; see
docs/reproduction_log.md M1) and an additive per-wheel input disturbance d (M3, M7):
    m v_dot     = (u_R + d_R + u_L + d_L) / r                 [- m h omega^2     if use_h_terms]
    I omega_dot = b (u_R + d_R - u_L - d_L) / r                [+ m h v omega     if use_h_terms]

The optional h-terms come from reducing the full model (1)-(2) with S^T; the paper drops them
in (8). Sign convention: the mass centre O_m lies a distance h *behind* the axle (Fig. 1).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass(frozen=True)
class RobotParams:
    """Physical parameters (Table 1): mass m [kg], inertia I [kg m^2], wheel radius r [m],
    half track width b [m], mass-centre offset h behind the axle [m]."""

    m: float
    I: float
    r: float
    b: float
    h: float = 0.0

    @property
    def B(self) -> np.ndarray:
        """Input matrix of eq. (9): B = (1/r) [[1, 1], [b, -b]]."""
        return np.array([[1.0, 1.0], [self.b, -self.b]]) / self.r


@dataclass(frozen=True)
class InputDisturbance:
    """Additive per-wheel input disturbance, active for t_start <= t <= t_end.

    Value = amplitude * shape(t) * wheel_signs, where shape is
      constant:  1                                  (a step if t_start > 0)
      sinusoid:  sin(2 pi freq (t - t_start) + phase)
      half_sine: |sin(2 pi freq (t - t_start))|     (eq. 61 with freq = 1 Hz, t in [12, 12.5])
    """

    kind: Literal["constant", "sinusoid", "half_sine"]
    amplitude: float
    wheel_signs: tuple[float, float] = (1.0, 1.0)
    t_start: float = 0.0
    t_end: float = math.inf
    freq: float = 0.0
    phase: float = 0.0

    def __call__(self, t: float) -> np.ndarray:
        if not self.t_start <= t <= self.t_end:
            return np.zeros(2)
        tau = t - self.t_start
        if self.kind == "constant":
            shape = 1.0
        elif self.kind == "sinusoid":
            shape = math.sin(2 * math.pi * self.freq * tau + self.phase)
        elif self.kind == "half_sine":
            shape = abs(math.sin(2 * math.pi * self.freq * tau))
        else:
            raise ValueError(f"unknown disturbance kind {self.kind!r}")
        return self.amplitude * shape * np.asarray(self.wheel_signs, dtype=float)


@dataclass(frozen=True)
class Plant:
    """True robot: parameters, input saturation (u_max=None: off), h-terms, disturbances."""

    params: RobotParams
    u_max: float | None = None
    use_h_terms: bool = False
    disturbances: tuple[InputDisturbance, ...] = ()

    def disturbance(self, t: float) -> np.ndarray:
        """Total per-wheel input disturbance [d_R, d_L] at time t."""
        return sum((d(t) for d in self.disturbances), np.zeros(2))

    def saturate(self, u: np.ndarray) -> np.ndarray:
        """Clip each wheel input to [-u_max, u_max] (applied before the disturbance)."""
        u = np.asarray(u, dtype=float)
        return u if self.u_max is None else np.clip(u, -self.u_max, self.u_max)

    def deriv(self, t: float, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        """State derivative for an already-saturated wheel input u = [u_R, u_L]."""
        p = self.params
        _, _, phi, v, w = state
        u_R, u_L = np.asarray(u, dtype=float) + self.disturbance(t)
        force = (u_R + u_L) / p.r
        torque = p.b * (u_R - u_L) / p.r
        if self.use_h_terms:
            force -= p.m * p.h * w**2
            torque += p.m * p.h * v * w
        return np.array([v * math.cos(phi), v * math.sin(phi), w, force / p.m, torque / p.I])


def kinematics(state: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Derivative of [x, y, phi] for body velocities z = [v, omega], eq. (4)."""
    phi = state[2]
    v, w = z
    return np.array([v * math.cos(phi), v * math.sin(phi), w])


@dataclass
class MeasurementNoise:
    """Additive Gaussian noise on [x, y, phi, v, omega] with standard deviations std."""

    std: np.ndarray
    rng: np.random.Generator

    def measure(self, state: np.ndarray) -> np.ndarray:
        if not np.any(self.std):
            return state.copy()
        return state + self.std * self.rng.standard_normal(state.shape)
