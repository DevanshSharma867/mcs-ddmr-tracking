"""Analytic reference derivatives vs central finite differences."""

import math

import pytest

from ddmr.errors import wrap_angle
from ddmr.trajectories import Circle, FigureEight, StraightLine, paper_circle

TRAJS = [
    paper_circle(),
    Circle((1.0, -2.0), 1.5, 0.3, start_angle=0.4, ccw=False),
    FigureEight(A=1.0, B=0.5, omega=0.4, center=(0.2, 0.1)),
    StraightLine((0.5, -0.5), 0.8, 0.3),
]
TIMES = [0.0, 1.3, 4.7, 11.0]
H = 1e-5


@pytest.mark.parametrize("traj", TRAJS, ids=lambda t: type(t).__name__)
@pytest.mark.parametrize("t", TIMES)
def test_derivatives_match_finite_differences(traj, t):
    a, b, c = traj.sample(t - H), traj.sample(t), traj.sample(t + H)

    def fd(name):
        return (getattr(c, name) - getattr(a, name)) / (2 * H)

    assert b.x_dot == pytest.approx(fd("x"), abs=1e-7)
    assert b.y_dot == pytest.approx(fd("y"), abs=1e-7)
    assert b.x_ddot == pytest.approx(fd("x_dot"), abs=1e-7)
    assert b.y_ddot == pytest.approx(fd("y_dot"), abs=1e-7)
    assert b.v_dot == pytest.approx(fd("v"), abs=1e-7)
    assert b.omega == pytest.approx(wrap_angle(c.phi - a.phi) / (2 * H), abs=1e-7)
    assert b.omega_dot == pytest.approx(fd("omega"), abs=1e-7)
    assert b.v == pytest.approx(math.hypot(b.x_dot, b.y_dot))


def test_paper_circle_start():
    r = paper_circle().sample(0.0)
    assert (r.x, r.y, r.phi) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert (r.v, r.omega, r.v_dot, r.omega_dot) == pytest.approx((0.42, 0.7, 0.0, 0.0), abs=1e-12)
    q = paper_circle().sample(math.pi / 0.7)        # half a period: top of the circle
    assert (q.x, q.y) == pytest.approx((0.0, 1.2), abs=1e-12)
