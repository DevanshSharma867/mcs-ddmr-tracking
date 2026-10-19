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

Optional actuator lag (ASSUMED, sensitivity check only): a per-wheel first-order low-pass
    a_dot = (u - a) / actuator_tau
between the saturated command u and the wheel input a; the disturbance is added after it, so the
wheel input is a + d. It is a phenomenological stand-in for unmodelled actuation (motor electrical
lag, PWM/driver filtering, gearbox compliance, compute/update delays), not a motor model. With
the lag on, the state is [x, y, phi, v, omega, a_R, a_L].

Optional voltage-input DC motor per wheel (plant (c), ASSUMED structure; docs/reproduction_log.md):
the wheel input u (+ disturbance) is a motor voltage [V] and the wheel torque is
    tau_w = n (k_t/R) (u_v - k_e n omega_w) - n^2 J_m omega_w_dot,   omega_w = (v +/- b omega) / r
(inductance, friction and gear losses neglected). Equivalently, with c = 2 n^2 k_t k_e / (R r^2):
    (m + 2 n^2 J_m / r^2)       v_dot     = n (k_t/R) (u_R + u_L) / r     - c v
    (I + 2 n^2 J_m b^2 / r^2)   omega_dot = b n (k_t/R) (u_R - u_L) / r   - c b^2 omega

Optional viscous wheel friction (Phase 5 grid): tau_f = -c_w omega_w per wheel, i.e. an extra
-2 c_w v / r^2 on the force and -2 c_w b^2 omega / r^2 on the torque (both plant types).
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
class BandlimitedDisturbance:
    """Seeded band-limited noise on each wheel input (independent per wheel): white Gaussian noise
    through a first-order low-pass with cutoff [Hz], stationary std = amplitude, sampled every dt
    and held (zero-order) in between:
        y_k = a y_{k-1} + sqrt(1 - a^2) amplitude w_k,   a = exp(-2 pi cutoff dt),   y_0 ~ N(0, amplitude^2)
    """

    amplitude: float
    cutoff: float
    duration: float
    seed: int
    dt: float = 1e-3

    def __post_init__(self) -> None:
        rng = np.random.default_rng(self.seed)
        n = int(math.ceil(self.duration / self.dt)) + 1
        a = math.exp(-2.0 * math.pi * self.cutoff * self.dt)
        w = rng.standard_normal((n, 2))
        y = np.empty((n, 2))
        y[0] = self.amplitude * w[0]
        g = math.sqrt(1.0 - a * a) * self.amplitude
        for k in range(1, n):
            y[k] = a * y[k - 1] + g * w[k]
        object.__setattr__(self, "_samples", y)

    def __call__(self, t: float) -> np.ndarray:
        k = min(max(int(t / self.dt + 1e-9), 0), len(self._samples) - 1)
        return self._samples[k].copy()


@dataclass(frozen=True)
class MotorParams:
    """Geared DC motor per wheel: torque constant kt [N m/A], back-EMF constant ke [V s/rad],
    terminal resistance R [ohm], gear ratio n [-], rotor inertia J [kg m^2]."""

    kt: float
    ke: float
    R: float
    n: float
    J: float


@dataclass(frozen=True)
class Plant:
    """True robot: parameters, input saturation (u_max=None: off), h-terms, disturbances,
    optional actuator lag time constant [s] (None: off), optional DC motor (None: torque input)."""

    params: RobotParams
    u_max: float | None = None
    use_h_terms: bool = False
    disturbances: tuple[InputDisturbance | BandlimitedDisturbance, ...] = ()
    actuator_tau: float | None = None
    motor: MotorParams | None = None
    wheel_friction: float = 0.0

    def __post_init__(self) -> None:
        if self.motor is not None and self.actuator_tau is not None:
            raise ValueError("actuator lag and DC motor are alternative plants; set only one")

    @property
    def n_states(self) -> int:
        return 5 if self.actuator_tau is None else 7

    def disturbance(self, t: float) -> np.ndarray:
        """Total per-wheel input disturbance [d_R, d_L] at time t."""
        return sum((d(t) for d in self.disturbances), np.zeros(2))

    def saturate(self, u: np.ndarray) -> np.ndarray:
        """Clip each wheel input to [-u_max, u_max] (applied before the disturbance)."""
        u = np.asarray(u, dtype=float)
        return u if self.u_max is None else np.clip(u, -self.u_max, self.u_max)

    def deriv(self, t: float, state: np.ndarray, u: np.ndarray) -> np.ndarray:
        """State derivative for an already-saturated wheel command u = [u_R, u_L]."""
        p = self.params
        _, _, phi, v, w = state[:5]
        u = np.asarray(u, dtype=float)
        lag = None
        if self.actuator_tau is not None:
            lag = (u - state[5:7]) / self.actuator_tau
            u = state[5:7]
        u_R, u_L = u + self.disturbance(t)
        m_eff, I_eff = p.m, p.I
        if self.motor is not None:
            mo = self.motor
            k = mo.n * mo.kt / mo.R                          # wheel torque per volt [N m/V]
            c = 2.0 * mo.n**2 * mo.kt * mo.ke / (mo.R * p.r**2)   # back-EMF damping [N s/m]
            force = k * (u_R + u_L) / p.r - c * v
            torque = p.b * k * (u_R - u_L) / p.r - c * p.b**2 * w
            m_eff += 2.0 * mo.n**2 * mo.J / p.r**2
            I_eff += 2.0 * mo.n**2 * mo.J * p.b**2 / p.r**2
        else:
            force = (u_R + u_L) / p.r
            torque = p.b * (u_R - u_L) / p.r
        if self.wheel_friction:
            force -= 2.0 * self.wheel_friction * v / p.r**2
            torque -= 2.0 * self.wheel_friction * p.b**2 * w / p.r**2
        if self.use_h_terms:
            force -= p.m * p.h * w**2
            torque += p.m * p.h * v * w
        body = np.array([v * math.cos(phi), v * math.sin(phi), w, force / m_eff, torque / I_eff])
        return body if lag is None else np.concatenate([body, lag])


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
