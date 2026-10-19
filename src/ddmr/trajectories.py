"""Reference trajectories with analytic derivatives (SI units).

Each trajectory supplies x_r(t), y_r(t) and their first three time derivatives; the base class
derives the remaining reference quantities:
    v_r       = sqrt(x_dot^2 + y_dot^2)                                   eq. (16)
    omega_r   = (x_dot y_ddot - x_ddot y_dot) / v_r^2                     eq. (17)
    v_r_dot   = (x_dot x_ddot + y_dot y_ddot) / v_r
    omega_r_dot = (x_dot y_dddot - x_dddot y_dot) / v_r^2 - 2 omega_r v_r_dot / v_r
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RefSample:
    """Reference at time t: pose, body velocities and their derivatives, Cartesian derivatives."""

    t: float
    x: float
    y: float
    phi: float
    v: float
    omega: float
    v_dot: float
    omega_dot: float
    x_dot: float
    y_dot: float
    x_ddot: float
    y_ddot: float

    @property
    def pose(self) -> np.ndarray:
        return np.array([self.x, self.y, self.phi])


class Trajectory(ABC):
    """Base class: subclasses return (x, y, x', y', x'', y'', x''', y''') at time t."""

    @abstractmethod
    def flat(self, t: float) -> tuple[float, ...]:
        """Position and its first three derivatives: (x, y, xd, yd, xdd, ydd, xddd, yddd)."""

    def heading(self, t: float, xd: float, yd: float) -> float:
        """Reference heading; override for a continuous (unwrapped) analytic form."""
        return math.atan2(yd, xd)

    def sample(self, t: float) -> RefSample:
        x, y, xd, yd, xdd, ydd, xddd, yddd = self.flat(t)
        v = math.hypot(xd, yd)
        if v < 1e-9:
            raise ValueError(f"reference speed is zero at t={t}; heading undefined")
        omega = (xd * ydd - xdd * yd) / v**2
        v_dot = (xd * xdd + yd * ydd) / v
        omega_dot = (xd * yddd - xddd * yd) / v**2 - 2.0 * omega * v_dot / v
        return RefSample(t, x, y, self.heading(t, xd, yd), v, omega, v_dot, omega_dot,
                         xd, yd, xdd, ydd)


@dataclass(frozen=True)
class Circle(Trajectory):
    """Circle of given radius [m] about center, traversed at angular rate omega [rad/s].

    Position angle theta(t) = start_angle + s omega t (s = +1 CCW, -1 CW);
    heading phi_r = theta + s pi/2 (continuous).
    """

    center: tuple[float, float]
    radius: float
    omega: float
    start_angle: float = -math.pi / 2
    ccw: bool = True

    def _theta(self, t: float) -> tuple[float, float]:
        s = 1.0 if self.ccw else -1.0
        return self.start_angle + s * self.omega * t, s

    def flat(self, t: float) -> tuple[float, ...]:
        th, s = self._theta(t)
        R, W = self.radius, s * self.omega
        c, n = math.cos(th), math.sin(th)
        return (self.center[0] + R * c, self.center[1] + R * n,
                -R * W * n, R * W * c,
                -R * W**2 * c, -R * W**2 * n,
                R * W**3 * n, -R * W**3 * c)

    def heading(self, t: float, xd: float, yd: float) -> float:
        th, s = self._theta(t)
        return th + s * math.pi / 2


def paper_circle() -> Circle:
    """Paper Sect. 4.3: R = 0.60 m, 0.70 rad/s. Centre (0, 0.6), start (0, 0), heading 0, CCW
    are read from Fig. 5a/b (ASSUMED, docs/reproduction_log.md)."""
    return Circle(center=(0.0, 0.6), radius=0.6, omega=0.7, start_angle=-math.pi / 2, ccw=True)


@dataclass(frozen=True)
class FigureEight(Trajectory):
    """Figure-8: x = cx + A sin(W t), y = cy + B sin(2 W t). Speed is never zero."""

    A: float
    B: float
    omega: float
    center: tuple[float, float] = (0.0, 0.0)

    def flat(self, t: float) -> tuple[float, ...]:
        A, B, W = self.A, self.B, self.omega
        s1, c1 = math.sin(W * t), math.cos(W * t)
        s2, c2 = math.sin(2 * W * t), math.cos(2 * W * t)
        return (self.center[0] + A * s1, self.center[1] + B * s2,
                A * W * c1, 2 * B * W * c2,
                -A * W**2 * s1, -4 * B * W**2 * s2,
                -A * W**3 * c1, -8 * B * W**3 * c2)


@dataclass(frozen=True)
class StraightLine(Trajectory):
    """Straight line from start at constant speed [m/s] along heading [rad]."""

    start: tuple[float, float]
    heading_angle: float
    speed: float

    def flat(self, t: float) -> tuple[float, ...]:
        c, s = math.cos(self.heading_angle), math.sin(self.heading_angle)
        return (self.start[0] + self.speed * t * c, self.start[1] + self.speed * t * s,
                self.speed * c, self.speed * s, 0.0, 0.0, 0.0, 0.0)

    def heading(self, t: float, xd: float, yd: float) -> float:
        return self.heading_angle
