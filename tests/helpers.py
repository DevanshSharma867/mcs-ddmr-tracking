"""Small controllers shared by the tests."""

import numpy as np

from ddmr.controllers.base import Controller


class ConstantInput(Controller):
    """Torque controller returning a fixed wheel input; records call times."""

    output = "torque"

    def __init__(self, u=(0.0, 0.0)):
        self.u = np.asarray(u, dtype=float)
        self.calls: list[float] = []

    def reset(self):
        self.calls = []

    def compute(self, t, state, ref):
        self.calls.append(t)
        return self.u


class IdealVelocity(Controller):
    """Velocity controller that feeds the reference velocities forward."""

    output = "velocity"

    def reset(self):
        pass

    def compute(self, t, state, ref):
        return np.array([ref.v, ref.omega])
