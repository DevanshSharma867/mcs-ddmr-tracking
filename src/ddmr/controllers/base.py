"""Common interface for all tracking controllers.

A controller maps the measured state and the reference at time t to a control command.
Outer-loop (kinematic) controllers return body-velocity commands (v [m/s], omega [rad/s]);
inner-loop (dynamic) controllers return wheel torques (tau_R, tau_L) [N*m]. Which one a
controller produces is declared by its `output` attribute so the simulator can route it.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import ClassVar, Literal

import numpy as np

from ddmr.trajectories import RefSample

OutputKind = Literal["torque", "velocity"]


class Controller(ABC):
    """Base class for kinematic and dynamic tracking controllers."""

    output: ClassVar[OutputKind]

    @abstractmethod
    def reset(self) -> None:
        """Clear internal state (integrators, adaptive gains, filters) before a new run."""

    @abstractmethod
    def compute(self, t: float, state: np.ndarray, ref: RefSample) -> np.ndarray:
        """Return the control command at time t [s].

        Args:
            t: simulation time [s].
            state: measured state [x, y, phi, v, omega].
            ref: reference sample at t (pose, velocities and their derivatives).

        Returns:
            np.ndarray of shape (2,): (tau_R, tau_L) if `output == "torque"`,
            (v, omega) if `output == "velocity"`.
        """

    def diagnostics(self) -> dict[str, float]:
        """Internal signals from the last `compute` call, logged by the simulator every tick."""
        return {}
