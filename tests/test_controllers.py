"""Unit checks of the baseline control laws against their defining equations."""

import math

import numpy as np
import pytest

from ddmr.config import load_config, nominal_params
from ddmr.controllers.ckc import CKC
from ddmr.controllers.csmdc import CSMDC
from ddmr.controllers.pi import CascadePI
from ddmr.controllers.rsmkc import RSMKC
from ddmr.controllers.xi_terms import xi_terms
from ddmr.errors import tracking_error
from ddmr.model import Plant, kinematics
from ddmr.trajectories import FigureEight, paper_circle

P = nominal_params(load_config("configs/robot_paper.yaml"))
STATE = np.array([-0.08, 0.05, 0.3, 0.35, 0.5])
T0, H = 1.7, 1e-6
TRAJS = [paper_circle(), FigureEight(1.0, 0.5, 0.4)]


def test_ckc_law_eq57():
    ref = paper_circle().sample(0.0)
    x_e, y_e, phi_e = tracking_error(np.array([-0.1, -0.1, 0.2]), ref.pose)
    z = CKC(1.7, 1.7, 1.5).compute(0.0, np.array([-0.1, -0.1, 0.2, 0, 0]), ref)
    assert z[0] == pytest.approx(0.42 * math.cos(phi_e) + 1.7 * x_e)
    assert z[1] == pytest.approx(0.7 + 1.7 * 0.42 * y_e + 1.5 * 0.42 * math.sin(phi_e))


@pytest.mark.parametrize("traj", TRAJS)
def test_rsmkc_gives_sliding_dynamics(traj):
    c = RSMKC(0.40, 0.02, 0.03)
    z = c.compute(T0, STATE, traj.sample(T0))
    pose = STATE[:3]
    step = kinematics(pose, z) * H

    def s_at(p, t):
        return c.sliding_variable(tracking_error(p, traj.sample(t).pose), traj.sample(t))

    s_dot = (s_at(pose + step, T0 + H) - s_at(pose - step, T0 - H)) / (2 * H)
    s = s_at(pose, T0)
    assert s_dot == pytest.approx(-np.array([0.02, 0.03]) * np.sign(s), abs=1e-6)


def _xi(state, t, traj):
    ref = traj.sample(t)
    return xi_terms(tracking_error(state, ref.pose), state[3:5], ref, 0.8, P)


@pytest.mark.parametrize("traj", TRAJS)
def test_xi_dot_and_xi_ddot_match_finite_differences(traj):
    plant = Plant(P)
    tau = np.array([0.7, -0.4])
    f = plant.deriv(T0, STATE, tau)
    plus, minus = STATE + H * f, STATE - H * f
    xt = _xi(STATE, T0, traj)
    xi_dot_fd = (_xi(plus, T0 + H, traj).xi - _xi(minus, T0 - H, traj).xi) / (2 * H)
    assert xt.xi_dot == pytest.approx(xi_dot_fd, abs=1e-6)
    xi_ddot_fd = (_xi(plus, T0 + H, traj).xi_dot - _xi(minus, T0 - H, traj).xi_dot) / (2 * H)
    assert xt.drift + xt.D @ tau == pytest.approx(xi_ddot_fd, abs=1e-5)   # eq. (26), delta = 0


@pytest.mark.parametrize("traj", TRAJS)
def test_csmdc_gives_sliding_dynamics(traj):
    c = CSMDC(P, (21.0, 16.0), (3.0, 4.0), 0.8)
    tau = c.compute(T0, STATE, traj.sample(T0))
    f = Plant(P).deriv(T0, STATE, tau)
    s_dot = (c.sliding_variable(T0 + H, STATE + H * f, traj.sample(T0 + H))
             - c.sliding_variable(T0 - H, STATE - H * f, traj.sample(T0 - H))) / (2 * H)
    s = c.sliding_variable(T0, STATE, traj.sample(T0))
    assert s_dot == pytest.approx(-np.array([21.0, 16.0]) * np.sign(s), abs=1e-4)


def test_decoupling_matrix_determinant_eq52():
    xt = _xi(STATE, T0, paper_circle())
    x_e, y_e, _ = tracking_error(STATE, paper_circle().sample(T0).pose)
    bracket = 0.8 * x_e / ((abs(x_e) + 0.8) * (1 + y_e**2)) + 1.0
    assert np.linalg.det(xt.D) == pytest.approx(-2 * P.b / (P.r**2 * P.m * P.I) * bracket)


class Fixed:
    output = "velocity"

    def reset(self):
        pass

    def compute(self, t, state, ref):
        return np.array([1.0, 0.0])


def test_pi_maps_through_B_inverse_and_clamps():
    ref = paper_circle().sample(0.0)
    pi = CascadePI(Fixed(), P, 30.0, 250.0, 0.01, u_max=None)
    pi.reset()
    tau = pi.compute(0.0, np.zeros(5), ref)
    assert P.B @ tau == pytest.approx([30.0 + 250.0 * 0.01, 0.0])
    sat = CascadePI(Fixed(), P, 30.0, 250.0, 0.01, u_max=0.1)
    sat.reset()
    for k in range(5):
        sat.compute(k * 0.01, np.zeros(5), ref)
    assert np.array_equal(sat.integral, np.zeros(2))   # output always saturated -> no windup


def test_csmdc_boundary_layer_inside_layer_is_linear():
    c = CSMDC(P, (21.0, 16.0), (3.0, 4.0), 0.8, boundary_layer=1e3)   # wide layer: |s| << varrho
    traj = paper_circle()
    tau = c.compute(T0, STATE, traj.sample(T0))
    f = Plant(P).deriv(T0, STATE, tau)
    s_dot = (c.sliding_variable(T0 + H, STATE + H * f, traj.sample(T0 + H))
             - c.sliding_variable(T0 - H, STATE - H * f, traj.sample(T0 - H))) / (2 * H)
    s = c.sliding_variable(T0, STATE, traj.sample(T0))
    assert s_dot == pytest.approx(-np.array([21.0, 16.0]) * s / 1e3, abs=1e-6)
