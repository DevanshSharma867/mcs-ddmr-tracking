"""Simulator timing, zero-order hold, noise and velocity mode."""

import numpy as np
import pytest

from ddmr.controllers.base import Controller
from ddmr.model import MeasurementNoise, Plant, RobotParams
from ddmr.sim import simulate, steps_ratio
from ddmr.trajectories import paper_circle
from tests.helpers import ConstantInput, IdealVelocity

P = RobotParams(m=4.5, I=0.56, r=0.042, b=0.186)


def test_controller_called_every_Ts_with_exact_times():
    c = ConstantInput()
    df = simulate(Plant(P), c, paper_circle(), np.zeros(5), T=2.0, Ts=0.01, dt=0.001)
    assert len(c.calls) == len(df) == 201
    assert c.calls == [k * 10 * 0.001 for k in range(201)]
    assert df["t"].iloc[-1] == pytest.approx(2.0)


def test_steps_ratio_rejects_non_integer():
    assert steps_ratio(0.01, 0.001, "Ts/dt") == 10
    with pytest.raises(AssertionError):
        steps_ratio(0.0105, 0.001, "Ts/dt")


class Ramp(Controller):
    """Input changes every tick: u_k = 0.01 k on both wheels."""

    output = "torque"

    def reset(self):
        self.k = 0

    def compute(self, t, state, ref):
        u = 0.01 * self.k
        self.k += 1
        return np.array([u, u])


def test_zero_order_hold():
    df = simulate(Plant(P), Ramp(), paper_circle(), np.zeros(5), T=0.5, Ts=0.01, dt=0.001)
    dv = np.diff(df["v"].to_numpy())
    expected = 2 * df["ua_R"].to_numpy()[:-1] / (P.m * P.r) * 0.01   # u_k held over [t_k, t_k+1)
    assert np.allclose(dv, expected, atol=1e-12)


def test_measurement_noise_seeded_and_zero_by_default():
    def run(std, seed):
        noise = MeasurementNoise(np.asarray(std, float), np.random.default_rng(seed))
        return simulate(Plant(P), ConstantInput((0.1, 0.05)), paper_circle(), np.zeros(5),
                        T=1.0, Ts=0.01, dt=0.001, noise=noise)

    std = [0.01, 0.01, 0.005, 0.0, 0.0]
    a, b = run(std, 1), run(std, 1)
    assert a.equals(b)
    assert not np.allclose(a["x_meas"], a["x"])
    z = run([0.0] * 5, 1)
    assert np.array_equal(z["x_meas"], z["x"]) and np.array_equal(z["phi_meas"], z["phi"])


def test_velocity_mode_tracks_reference_exactly():
    df = simulate(Plant(P), IdealVelocity(), paper_circle(), np.zeros(5), T=20.0, Ts=0.01, dt=0.001)
    assert np.max(np.abs(df[["x_e", "y_e", "phi_e"]].to_numpy())) < 1e-9
    assert np.isfinite(df.select_dtypes("number").to_numpy()).all()
