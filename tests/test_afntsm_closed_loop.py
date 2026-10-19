"""Closed-loop acceptance of the reproduced AFNTSMDC on scenario S1 (paper circle, offset).

Thresholds fixed from the paper's Table 2 AFNTSMDC row before running (same rule as the
baselines): settle into the M6 band no later than 2x the paper's settling time, MAX no more than
2x the paper's MAX. Run on the ideal plant and on the plant with a 20 ms actuator lag.
Failures are reported, not loosened.
"""

import numpy as np
import pytest

from ddmr.config import build_controller, load_config, simulate_from_config
from ddmr.metrics import tracking_metrics
from ddmr.paper_tables import TABLE2

MARGIN = 2.0
PLANTS = {"ideal": None, "lag20ms": 0.020}


@pytest.fixture(scope="module")
def runs():
    gains = load_config("configs/controllers_paper.yaml")
    out = {}
    for name, tau in PLANTS.items():
        cfg = load_config("configs/robot_paper.yaml")
        cfg["plant"]["actuator_tau"] = tau
        out[name] = simulate_from_config(cfg, build_controller("afntsmdc", gains, cfg))
    return out


@pytest.mark.parametrize("plant", PLANTS)
def test_no_nan_or_inf(runs, plant):
    assert np.isfinite(runs[plant].select_dtypes("number").to_numpy()).all()


@pytest.mark.parametrize("plant", PLANTS)
@pytest.mark.parametrize("err", ["x_e", "y_e", "phi_e"])
def test_within_paper_margin(runs, plant, err):
    m = tracking_metrics(runs[plant])[err]
    _, paper_max, paper_settle = TABLE2["afntsmdc"][err]
    scale = 100.0 if err != "phi_e" else 1.0
    assert not np.isnan(m["settle"]), f"{plant} {err} never settles into the band"
    assert m["settle"] <= MARGIN * paper_settle, f"{plant} {err} settles at {m['settle']:.2f} s"
    assert m["max"] * scale <= MARGIN * paper_max, f"{plant} {err} MAX {m['max'] * scale:.3f}"


def test_as_printed_law_diverges():
    """Expected finding (reproduction_log "AFNTSMDC switching sign"): the literal eq. (32) sign
    makes the adaptive switching term push s away from 0 (HG ~ -I), so the robot leaves the
    trajectory and the adaptive gains ramp. Criterion fixed in advance: late (t >= 15 s) position
    error > 10 cm and c4(20 s) > 2 c4(0)."""
    gains = load_config("configs/controllers_paper.yaml")
    gains["afntsmdc"]["switching"] = "printed"
    cfg = load_config("configs/robot_paper.yaml")
    df = simulate_from_config(cfg, build_controller("afntsmdc", gains, cfg))
    late = df[df["t"] >= 15.0]
    assert np.hypot(late["x_e"], late["y_e"]).max() > 0.10
    assert df["c4"].iloc[-1] > 2 * df["c4"].iloc[0]
