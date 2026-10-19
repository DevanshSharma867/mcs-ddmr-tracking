"""Build plant, trajectory, noise and simulation settings from a YAML experiment config.

See configs/robot_paper.yaml for the schema. `robot.true` overrides `robot.nominal` key by key
(plant = true parameters, controllers = nominal parameters).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

from ddmr.controllers.base import Controller
from ddmr.model import InputDisturbance, MeasurementNoise, Plant, RobotParams
from ddmr.sim import simulate
from ddmr.trajectories import Circle, FigureEight, StraightLine, Trajectory

TRAJECTORIES = {"circle": Circle, "figure_eight": FigureEight, "straight_line": StraightLine}


def load_config(path: str | Path) -> dict[str, Any]:
    with open(path) as f:
        return yaml.safe_load(f)


def nominal_params(cfg: dict) -> RobotParams:
    """Parameters the controllers are designed with (m0, I0, ...)."""
    return RobotParams(**cfg["robot"]["nominal"])


def true_params(cfg: dict) -> RobotParams:
    """Parameters of the simulated plant: nominal, overridden by robot.true."""
    return RobotParams(**{**cfg["robot"]["nominal"], **(cfg["robot"].get("true") or {})})


def build_plant(cfg: dict) -> Plant:
    pc = cfg["plant"]
    dists = tuple(
        InputDisturbance(**{**d, "wheel_signs": tuple(d.get("wheel_signs", (1.0, 1.0)))})
        for d in pc.get("disturbances") or []
    )
    return Plant(true_params(cfg), pc.get("u_max"), bool(pc.get("use_h_terms", False)), dists)


def build_trajectory(tc: dict) -> Trajectory:
    kwargs = {k: (tuple(v) if isinstance(v, list) else v) for k, v in tc.items() if k != "type"}
    return TRAJECTORIES[tc["type"]](**kwargs)


def build_noise(cfg: dict) -> MeasurementNoise:
    return MeasurementNoise(np.asarray(cfg["noise"]["std"], dtype=float),
                            np.random.default_rng(cfg["seed"]))


def simulate_from_config(cfg: dict, controller: Controller) -> pd.DataFrame:
    """Run `controller` on the plant, trajectory and settings described by cfg."""
    s = cfg["sim"]
    return simulate(build_plant(cfg), controller, build_trajectory(cfg["trajectory"]),
                    np.asarray(cfg["initial_state"], dtype=float), s["T"], s["Ts"], s["dt"],
                    build_noise(cfg))
