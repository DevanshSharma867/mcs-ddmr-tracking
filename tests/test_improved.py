"""Phase 6 improved controller: ESO convergence, projection, bounded C_hat, sized boundary layer."""

import numpy as np
import pytest

from ddmr.config import build_controller, deep_merge, load_config, nominal_params
from ddmr.controllers.improved import ImprovedAFNTSMDC, sized_boundary_layer

CFG = load_config("configs/robot_paper.yaml")
GAINS = load_config("configs/controllers_paper.yaml")
P = nominal_params(CFG)


def make(**over) -> ImprovedAFNTSMDC:
    return build_controller("afntsmdc_improved", deep_merge(GAINS, {"improved": over}), CFG)


def test_sized_boundary_layer_matches_preregistration():
    rho = sized_boundary_layer(3.0, np.array([9.0, 4.0]), 0.01, 1.5)
    assert rho == pytest.approx([0.0520, 0.0479], abs=1e-4)
    assert np.all(1.5 * (np.array([9.0, 4.0]) + 3.0 / rho) * 0.01 <= 1.0 + 1e-12)
    with pytest.raises(ValueError):
        sized_boundary_layer(3.0, np.array([80.0, 4.0]), 0.01, 1.5)


def test_eso_converges_to_constant_lumped_disturbance():
    c = make(use_ps=False)
    c.reset()
    f_true = np.array([1.3, -0.7])                         # m/s^2, rad/s^2
    tau = np.array([0.2, 0.1])
    z = np.array([0.3, 0.2])
    for _ in range(300):                                   # 3 s at Ts = 0.01
        state = np.array([0, 0, 0, *z])
        u = tau + c.extra_torque(state, tau)
        z = z + 0.01 * (c.M0_inv_B @ u + f_true)           # exact model z_dot = w + f (ZOH, Euler)
    assert c.f_hat == pytest.approx(f_true, rel=1e-3)


def test_projection_and_bounded_C_hat():
    c = make()
    c.reset()
    for _ in range(10_000):
        c.adapt(np.array([5.0, -5.0]), 1.0, 2.0)            # far outside the dead zone
    assert np.all(c.c <= c.c_max) and np.all(c.c >= c.c_min)
    assert np.all(c.C_hat(10.0, 10.0) <= 3.0)


def test_arms_configuration():
    a1 = make(use_ps=False, switching="corrected")
    assert a1.switching == "corrected" and float(a1.boundary_layer.max()) == 0.04 and a1.c0[0] == 14.0
    a4 = make(use_ps=False, switching="none")
    assert a4.switching == "none" and a4.use_eso
    a2 = make(use_eso=False)
    assert not a2.use_eso and a2.switching == "general"
    tau_eso = a2.extra_torque(np.zeros(5), np.zeros(2))
    assert np.array_equal(tau_eso, np.zeros(2))
