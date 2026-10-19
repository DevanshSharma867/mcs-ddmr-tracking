"""Plant model sanity checks (CLAUDE.md 'Testing expectations': Model)."""

import math

import numpy as np
import pytest

from ddmr.config import build_plant, load_config
from ddmr.model import InputDisturbance, Plant, RobotParams
from ddmr.sim import simulate
from ddmr.trajectories import StraightLine, paper_circle
from tests.helpers import ConstantInput

P = RobotParams(m=4.5, I=0.56, r=0.042, b=0.186, h=0.11)
LINE = StraightLine((0.0, 0.0), 0.0, 0.5)


def run(plant, x0, u=(0.0, 0.0), T=5.0, traj=LINE):
    return simulate(plant, ConstantInput(u), traj, np.asarray(x0, float), T, 0.01, 0.001)


def test_zero_input_stays_put():
    df = run(Plant(P), [0.3, -0.2, 0.7, 0.0, 0.0])
    assert np.allclose(df[["x", "y", "phi", "v", "omega"]].iloc[-1], [0.3, -0.2, 0.7, 0, 0])


def test_constant_v_straight_line():
    df = run(Plant(P), [0.0, 0.0, 0.5, 0.4, 0.0])
    assert np.allclose(df["x"], 0.4 * df["t"] * math.cos(0.5))
    assert np.allclose(df["y"], 0.4 * df["t"] * math.sin(0.5))
    assert np.allclose(df["phi"], 0.5)


def test_constant_v_omega_circle_radius_v_over_omega():
    v, w = 0.42, 0.7
    df = run(Plant(P), [0.0, 0.0, 0.0, v, w], T=20.0, traj=paper_circle())
    R = v / w
    assert np.max(np.abs(np.hypot(df["x"], df["y"] - R) - R)) < 1e-9


def test_constant_input_accelerations():
    u = 0.1
    df = run(Plant(P), [0, 0, 0, 0, 0], u=(u, u), T=1.0)
    assert df["v"].iloc[-1] == pytest.approx(2 * u / (P.m * P.r) * 1.0)
    assert np.allclose(df["omega"], 0.0)
    df = run(Plant(P), [0, 0, 0, 0, 0], u=(u, -u), T=1.0)
    assert df["omega"].iloc[-1] == pytest.approx(2 * P.b * u / (P.r * P.I) * 1.0)
    assert np.allclose(df["v"], 0.0)


def test_saturation():
    assert np.array_equal(Plant(P, u_max=8.0).saturate([10.0, -9.0]), [8.0, -8.0])
    assert np.array_equal(Plant(P).saturate([10.0, -9.0]), [10.0, -9.0])
    df = run(Plant(P, u_max=8.0), [0, 0, 0, 0, 0], u=(20.0, -3.0), T=0.1)
    assert (df["ua_R"] == 8.0).all() and (df["ua_L"] == -3.0).all() and (df["u_R"] == 20.0).all()


def test_half_sine_shock_eq61():
    d = InputDisturbance("half_sine", 6.0, (1.0, 1.0), t_start=12.0, t_end=12.5, freq=1.0)
    assert np.allclose(d(11.99), 0) and np.allclose(d(12.51), 0)
    assert np.allclose(d(12.0), 0, atol=1e-12) and np.allclose(d(12.5), 0, atol=1e-12)
    assert np.allclose(d(12.25), [6.0, 6.0])
    ts = np.linspace(12.0, 12.5, 5001)
    vals = np.array([d(t)[0] for t in ts])
    assert (vals[1:-1] > 0).all()                       # exactly one hump, no zero inside
    assert np.trapezoid(vals, ts) == pytest.approx(6.0 / math.pi, rel=1e-6)
    # matches 6|sin(2 pi t)| written in absolute time, as printed in the paper
    assert np.allclose(vals, 6 * np.abs(np.sin(2 * np.pi * ts)))
    opp = InputDisturbance("half_sine", 6.0, (1.0, -1.0), 12.0, 12.5, freq=1.0)
    assert np.allclose(opp(12.25), [6.0, -6.0])


def test_disturbance_enters_after_saturation():
    shock = InputDisturbance("constant", 1.0)
    plant = Plant(P, u_max=8.0, disturbances=(shock,))
    xdot = plant.deriv(0.0, np.zeros(5), plant.saturate([20.0, 20.0]))
    assert xdot[3] == pytest.approx(2 * 9.0 / (P.r * P.m))


def test_h_terms_off_by_default_and_signs():
    s = np.array([0.0, 0.0, 0.0, 0.4, 0.7])
    assert np.allclose(Plant(P).deriv(0.0, s, np.zeros(2))[3:], 0.0)
    xdot = Plant(P, use_h_terms=True).deriv(0.0, s, np.zeros(2))
    assert xdot[3] == pytest.approx(-P.h * 0.7**2)
    assert xdot[4] == pytest.approx(P.m * P.h * 0.4 * 0.7 / P.I)


def test_paper_config_builds():
    plant = build_plant(load_config("configs/robot_paper.yaml"))
    assert plant.params == P and plant.u_max == 8.0 and not plant.use_h_terms
    assert plant.disturbances == ()


def test_scenario_configs_build_expected_plants():
    """Regression: a bare YAML `true:` key parses as boolean True and was silently ignored."""
    s2 = build_plant(load_config("configs/s2_payload.yaml"))
    assert (s2.params.m, s2.params.I) == (8.1, 1.008) and s2.disturbances == ()
    s3 = build_plant(load_config("configs/s3_payload_shock.yaml"))
    assert (s3.params.m, s3.params.I) == (8.1, 1.008)
    assert s3.disturbances[0].kind == "half_sine" and s3.disturbances[0].t_end == 12.5
    from ddmr.config import nominal_params
    assert nominal_params(load_config("configs/s2_payload.yaml")).m == 4.5
