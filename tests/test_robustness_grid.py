"""Robustness grid definition: job count, plant restrictions, disturbance equivalence (option B)."""

import pytest

from ddmr.config import build_plant, load_config
from experiments.robustness import accel_to_input, build_cfg, jobs

EXP = load_config("configs/robustness.yaml")


def test_job_count_and_lag_only_on_torque_plant():
    J = jobs(EXP)
    phase5 = {"ckc_pi", "rsmkc_pi", "csmdc", "afntsmdc", "afntsmdc_bl015"}
    assert sum(cid in phase5 for *_, cid, _ in J) == 1800          # Phase 5 grid, unchanged
    assert len(J) == 1800 + 1440                                   # + 4 Phase 6 arms (pre-registered)
    assert not any(p == "motor12V" and c.startswith("lag") for _, p, _, c, _, _ in J)


def test_equivalent_acceleration_conversion():
    robot = load_config("configs/robot_paper.yaml")["robot"]
    m0, r = 4.5, 0.042
    assert accel_to_input(2.0, {}, robot) == pytest.approx(2.0 * m0 * r / 2)            # 0.189
    mo = EXP["plants"]["motor12V"]["motor"]
    du = accel_to_input(2.0, EXP["plants"]["motor12V"], robot)
    assert 2 * mo["n"] * mo["kt"] * du / (mo["R"] * r * m0) == pytest.approx(2.0)        # 0.257 V


def test_condition_overrides_reach_the_plant():
    cfg = build_cfg(EXP, "motor12V", "figure8", "friction_30", 3)
    plant = build_plant(cfg)
    assert plant.wheel_friction == 0.08413 and plant.motor is not None and cfg["seed"] == 3
    assert cfg["trajectory"]["type"] == "figure_eight" and "radius" not in cfg["trajectory"]
    cfg = build_cfg(EXP, "torque", "circle", "dist_yaw", 0)
    d = build_plant(cfg).disturbances[0]
    assert d.wheel_signs == (1, -1) and d.amplitude == pytest.approx(0.189)
