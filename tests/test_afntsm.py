"""AFNTSMDC unit checks: sliding dynamics eq. (42), guards, adaptive law with dead zone."""

import numpy as np
import pytest

from ddmr.config import build_controller, load_config, nominal_params
from ddmr.controllers.afntsm import AFNTSMDC
from ddmr.controllers.switching import abs_pow, sig
from ddmr.controllers.xi_terms import xi_terms
from ddmr.errors import tracking_error
from ddmr.model import Plant
from ddmr.trajectories import FigureEight, paper_circle

CFG = load_config("configs/robot_paper.yaml")
P = nominal_params(CFG)
STATE = np.array([-0.08, 0.05, 0.3, 0.35, 0.5])
T0, H = 1.7, 1e-6


def make(zeta=(0.0, 0.0, 0.0, 0.0), boundary_layer=1e3, **kw):
    g = load_config("configs/controllers_paper.yaml")["afntsmdc"]
    g = {k: (tuple(v) if isinstance(v, list) else v) for k, v in g.items()}
    g.update(zeta=zeta, boundary_layer=boundary_layer, **kw)
    return AFNTSMDC(P, 0.01, **g)


def s_of(c, state, t, traj):
    ref = traj.sample(t)
    xt = xi_terms(tracking_error(state, ref.pose), state[3:5], ref, c.rho, P)
    return c.sliding_variable(xt.xi, xt.xi_dot), xt


@pytest.mark.parametrize("switching", ["printed", "corrected", "general", "none"])
@pytest.mark.parametrize("traj", [paper_circle(), FigureEight(1.0, 0.5, 0.4)])
def test_sliding_dynamics_eq42(traj, switching):
    """Printed law gives eq. (42) exactly: s_dot = -K1 s - K2 sig(s)^mu - HG L C (delta = 0);
    corrected flips the last sign, general gives -L C, none drops it."""
    c = make(switching=switching)       # frozen adaptation, wide layer: L = s / varrho
    c.reset()
    tau = c.compute(T0, STATE, traj.sample(T0))
    f = Plant(P).deriv(T0, STATE, tau)
    s_dot = (s_of(c, STATE + H * f, T0 + H, traj)[0] - s_of(c, STATE - H * f, T0 - H, traj)[0]) / (2 * H)
    s, xt = s_of(c, STATE, T0, traj)
    x_e, y_e, _ = tracking_error(STATE, traj.sample(T0).pose)
    G = np.array([[-1.0, y_e],
                  [0.0, -x_e], [0.0, -1.0]])
    v, w = STATE[3:5]
    C_hat = np.array([c.c0[0] + c.c0[2] * abs(v), c.c0[1] + c.c0[3] * abs(w)])
    LC = (s / 1e3) * C_hat
    term = {"printed": -xt.H @ G @ LC, "corrected": xt.H @ G @ LC, "general": -LC,
            "none": np.zeros(2)}[switching]
    expected = -c.K1 * s - c.K2 * sig(s, c.mu) + term
    assert s_dot == pytest.approx(expected, abs=1e-4)


def test_guards_finite_at_zero():
    assert np.array_equal(sig(np.zeros(2), 0.8), np.zeros(2))
    assert np.all(np.isfinite(abs_pow(np.zeros(2), -0.5)))
    assert sig(np.array([-4.0]), 0.5)[0] == pytest.approx(-2.0)
    assert abs_pow(np.array([-8.0]), 1 / 3)[0] == pytest.approx(2.0)


def test_adaptive_law_dead_zone_and_increments():
    zeta = (0.3, 0.4, 0.4, 0.5)
    c = make(zeta=zeta, boundary_layer=0.04)
    c.reset()
    ref = paper_circle().sample(T0)
    c.compute(T0, STATE, ref)
    d = c.diagnostics()
    s = np.array([d["s1"], d["s2"]])
    v, w = STATE[3:5]
    expect = c.c0.copy()
    if abs(s[0]) > c.eps:
        expect[[0, 2]] += 0.01 * np.array([zeta[0], zeta[2] * abs(v)]) * abs(s[0])
    if abs(s[1]) > c.eps:
        expect[[1, 3]] += 0.01 * np.array([zeta[1], zeta[3] * abs(w)]) * abs(s[1])
    assert c.c == pytest.approx(expect)
    assert np.all(c.c >= c.c0)
    big = make(zeta=zeta, eps=1e9)          # everything inside the dead zone: frozen
    big.reset()
    big.compute(T0, STATE, ref)
    assert np.array_equal(big.c, big.c0)


def test_builder_uses_paper_gains():
    c = build_controller("afntsmdc", load_config("configs/controllers_paper.yaml"), CFG)
    assert c.boundary_layer == 0.04 and tuple(c.c0) == (14.0, 14.0, 8.0, 6.0) and c.gamma == 1.67
    assert c.switching == "corrected"
