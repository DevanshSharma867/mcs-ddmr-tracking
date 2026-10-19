"""Build plant, trajectory, noise and simulation settings from a YAML experiment config.

See configs/robot_paper.yaml for the schema. `robot.actual` overrides `robot.nominal` key by key
(plant = true parameters, controllers = nominal parameters). A config may name a `base:` file
(relative to itself); the base is loaded first and this file's keys are deep-merged over it
(mappings merge recursively, everything else, including lists, is replaced).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

from ddmr.controllers.afntsm import AFNTSMDC
from ddmr.controllers.base import Controller
from ddmr.controllers.ckc import CKC
from ddmr.controllers.csmdc import CSMDC
from ddmr.controllers.pi import CascadePI
from ddmr.controllers.rsmkc import RSMKC
from ddmr.model import InputDisturbance, MeasurementNoise, Plant, RobotParams
from ddmr.sim import simulate
from ddmr.trajectories import Circle, FigureEight, StraightLine, Trajectory

TRAJECTORIES = {"circle": Circle, "figure_eight": FigureEight, "straight_line": StraightLine}


def deep_merge(base: dict, over: dict) -> dict:
    """Recursive dict merge; values in `over` win, lists are replaced, inputs are not mutated."""
    out = dict(base)
    for k, v in over.items():
        out[k] = deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def load_config(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    with open(path) as f:
        cfg = yaml.safe_load(f)
    base = cfg.pop("base", None)
    return deep_merge(load_config(path.parent / base), cfg) if base else cfg


def nominal_params(cfg: dict) -> RobotParams:
    """Parameters the controllers are designed with (m0, I0, ...)."""
    return RobotParams(**cfg["robot"]["nominal"])


def actual_params(cfg: dict) -> RobotParams:
    """Parameters of the simulated plant: nominal, overridden by robot.actual."""
    if True in cfg["robot"] or False in cfg["robot"]:
        raise ValueError("robot config has a boolean key (YAML parses bare true/false); use `actual`")
    return RobotParams(**{**cfg["robot"]["nominal"], **(cfg["robot"].get("actual") or {})})


def build_plant(cfg: dict) -> Plant:
    pc = cfg["plant"]
    dists = tuple(
        InputDisturbance(**{**d, "wheel_signs": tuple(d.get("wheel_signs", (1.0, 1.0)))})
        for d in pc.get("disturbances") or []
    )
    return Plant(actual_params(cfg), pc.get("u_max"), bool(pc.get("use_h_terms", False)), dists,
                 pc.get("actuator_tau"))


def build_trajectory(tc: dict) -> Trajectory:
    kwargs = {k: (tuple(v) if isinstance(v, list) else v) for k, v in tc.items() if k != "type"}
    return TRAJECTORIES[tc["type"]](**kwargs)


def build_noise(cfg: dict) -> MeasurementNoise:
    return MeasurementNoise(np.asarray(cfg["noise"]["std"], dtype=float),
                            np.random.default_rng(cfg["seed"]))


def build_controller(name: str, gains: dict, cfg: dict) -> Controller:
    """Controller by name ("ideal", "ckc_pi", "rsmkc_pi", "csmdc", "csmdc_bl", "afntsmdc") with gains from a gains config
    (configs/controllers_paper.yaml) and nominal parameters / Ts / u_max from the scenario cfg."""
    params, Ts, u_max = nominal_params(cfg), cfg["sim"]["Ts"], cfg["plant"].get("u_max")

    def cascade(outer: Controller) -> Controller:
        return CascadePI(outer, params, gains["pi"]["k_P"], gains["pi"]["k_I"], Ts, u_max)

    def csmdc(boundary_layer: float | None) -> Controller:
        c = gains["csmdc"]
        return CSMDC(params, tuple(c["k_c"]), tuple(c["lambda_c"]), c["rho"], boundary_layer)

    builders = {
        "ideal": lambda: CKC(**gains["ckc"]),
        "ckc_pi": lambda: cascade(CKC(**gains["ckc"])),
        "rsmkc_pi": lambda: cascade(RSMKC(**gains["rsmkc"])),
        "csmdc": lambda: csmdc(None),
        "csmdc_bl": lambda: csmdc(gains["csmdc"]["diagnostic_boundary_layer"]),
        "afntsmdc": lambda: AFNTSMDC(params, Ts, **{k: (tuple(v) if isinstance(v, list) else v)
                                                    for k, v in gains["afntsmdc"].items()}),
    }
    return builders[name]()


def simulate_from_config(cfg: dict, controller: Controller) -> pd.DataFrame:
    """Run `controller` on the plant, trajectory and settings described by cfg."""
    s = cfg["sim"]
    return simulate(build_plant(cfg), controller, build_trajectory(cfg["trajectory"]),
                    np.asarray(cfg["initial_state"], dtype=float), s["T"], s["Ts"], s["dt"],
                    build_noise(cfg))
