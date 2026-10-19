"""Metrics on synthetic logs with known answers."""

import numpy as np
import pandas as pd
import pytest

from ddmr.metrics import input_metrics, settling_time, tracking_metrics


def test_settling_time():
    t = np.arange(0, 1.0001, 0.1)
    assert settling_time(t, np.zeros_like(t), 0.01) == 0.0
    e = np.where(t < 0.45, 0.5, 0.0)
    assert settling_time(t, e, 0.01) == pytest.approx(0.5)
    assert np.isnan(settling_time(t, np.full_like(t, 0.5), 0.01))
    e = np.where((t < 0.25) | (np.isclose(t, 0.7)), 0.5, 0.0)       # re-exits once
    assert settling_time(t, e, 0.01) == pytest.approx(0.8)


def _log(t, e, u, u_cmd=None):
    u_cmd = u if u_cmd is None else u_cmd
    return pd.DataFrame({"t": t, "x_e": e, "y_e": -e, "phi_e": e / 10,
                         "ua_R": u[:, 0], "ua_L": u[:, 1], "u_R": u_cmd[:, 0], "u_L": u_cmd[:, 1]})


def test_tracking_metrics_rms_max_window():
    t = np.arange(0, 2.0, 0.01)                              # exactly two periods
    e = 0.03 * np.sin(2 * np.pi * t)
    m = tracking_metrics(_log(t, e, np.zeros((len(t), 2))))
    assert m["x_e"]["rms"] == pytest.approx(0.03 / np.sqrt(2), rel=1e-3)
    assert m["y_e"]["max"] == pytest.approx(0.03, rel=1e-3)
    w = tracking_metrics(_log(t, np.where(t < 0.95, 1.0, 0.0), np.zeros((len(t), 2))), window=(1.0, 2.0))
    assert w["x_e"]["max"] == 0.0 and w["x_e"]["settle"] == 0.0


def test_input_metrics():
    Ts = 0.01
    t = np.arange(0, 1.0, Ts)
    u = np.tile([2.0, -1.0], (len(t), 1))
    u[::2, 0] = 3.0                                           # right wheel toggles 3, 2, 3, ...
    cmd = u.copy()
    cmd[:25, 1] = -9.0                                        # 25 % of ticks above u_max = 8
    m = input_metrics(_log(t, np.zeros_like(t), u, cmd), Ts, 8.0)
    assert m["effort"] == pytest.approx(np.sum(u**2) * Ts)
    assert m["chattering"] == pytest.approx((len(t) - 1) * 1.0 / 1.0)
    assert m["saturated_pct"] == pytest.approx(25.0)
    assert input_metrics(_log(t, np.zeros_like(t), u, cmd), Ts, None)["saturated_pct"] == 0.0
