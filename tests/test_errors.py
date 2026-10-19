"""Robot-frame tracking error (eq. 14) and error dynamics (eq. 18)."""

import math

import numpy as np
import pytest

from ddmr.errors import error_rate, tracking_error, wrap_angle
from ddmr.model import kinematics
from ddmr.trajectories import FigureEight, paper_circle


def test_zero_error_at_reference():
    ref = paper_circle().sample(3.0)
    assert np.allclose(tracking_error(ref.pose, ref.pose), 0.0)


def test_rotation_into_robot_frame():
    # robot at origin facing +y; reference 1 m ahead (+y) and 2 m to its right (+x)
    pe = tracking_error(np.array([0.0, 0.0, math.pi / 2]), np.array([2.0, 1.0, math.pi / 2]))
    assert pe == pytest.approx([1.0, -2.0, 0.0])


def test_wrap_across_pi():
    pe = tracking_error(np.array([0.0, 0.0, -math.pi + 0.01]), np.array([0.0, 0.0, math.pi - 0.01]))
    assert pe[2] == pytest.approx(-0.02)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert -math.pi < wrap_angle(-7.0) <= math.pi


@pytest.mark.parametrize("traj", [paper_circle(), FigureEight(1.0, 0.5, 0.4)])
def test_error_rate_matches_finite_difference(traj):
    t, h = 2.3, 1e-5
    p = np.array([0.1, -0.2, 0.4])
    z = np.array([0.3, -0.5])
    pe = tracking_error(p, traj.sample(t).pose)
    step = kinematics(p, z) * h
    fd = (tracking_error(p + step, traj.sample(t + h).pose)
          - tracking_error(p - step, traj.sample(t - h).pose)) / (2 * h)
    assert error_rate(pe, z, traj.sample(t)) == pytest.approx(fd, abs=1e-7)
