"""Closed-loop acceptance of the baselines on scenario S1 (paper circle, initial offset).

Thresholds are fixed from the paper's Table 2 (NOT from our own runs): for each baseline and
each error, the error must settle into the M6 band (1 cm / 0.02 rad) no later than 2x the paper's
settling time, and its MAX must not exceed 2x the paper's MAX. The ideal kinematic reference uses
CKC's row (same law, perfect velocity tracking). Failures are reported, not loosened.
"""

import numpy as np
import pytest

from ddmr.config import build_controller, load_config, simulate_from_config
from ddmr.metrics import tracking_metrics
from ddmr.paper_tables import TABLE2

MARGIN = 2.0
CASES = {"ideal": "ckc", "ckc_pi": "ckc", "rsmkc_pi": "rsmkc", "csmdc": "csmdc"}
ERRORS = ["x_e", "y_e", "phi_e"]

# Known failure, kept as specified by the paper (see docs/reproduction_log.md, "CSMDC finding"):
# pure sgn sampled at Ts = 0.01 s gives discrete quasi-sliding with a band ~ K_c Ts around an
# off-centre mean, hence a steady xi offset; y_e never settles and phi_e settles late.
# strict=True: if these start passing, the suite fails so the finding is revisited.
CSMDC_XFAIL = pytest.mark.xfail(
    strict=True, reason="CSMDC discrete quasi-sliding offset at Ts = 0.01 s (reproduction_log)")
MARGIN_CASES = [
    pytest.param(name, err, id=f"{err}-{name}",
                 marks=CSMDC_XFAIL if (name, err) in {("csmdc", "y_e"), ("csmdc", "phi_e")} else ())
    for name in CASES for err in ERRORS
]


@pytest.fixture(scope="module")
def runs():
    cfg = load_config("configs/robot_paper.yaml")
    gains = load_config("configs/controllers_paper.yaml")
    return {name: simulate_from_config(cfg, build_controller(name, gains, cfg)) for name in CASES}


@pytest.mark.parametrize("name", CASES)
def test_no_nan_or_inf(runs, name):
    assert np.isfinite(runs[name].select_dtypes("number").to_numpy()).all()


@pytest.mark.parametrize(("name", "err"), MARGIN_CASES)
def test_within_paper_margin(runs, name, err):
    m = tracking_metrics(runs[name])[err]
    _, paper_max, paper_settle = TABLE2[CASES[name]][err]
    scale = 100.0 if err != "phi_e" else 1.0          # paper prints x_e, y_e in cm
    assert not np.isnan(m["settle"]), f"{name} {err} never settles into the band"
    assert m["settle"] <= MARGIN * paper_settle, f"{name} {err} settles at {m['settle']:.2f} s"
    assert m["max"] * scale <= MARGIN * paper_max, f"{name} {err} MAX {m['max'] * scale:.3f}"
