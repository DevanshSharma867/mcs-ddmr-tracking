"""Phase 2 open-loop demo: zero wheel input from z0 = (v, omega) = (0.42, 0.7) on the nominal
plant (no friction) must trace a circle of radius v/omega = 0.6 m, i.e. the paper's reference.

Writes results/phase2/open_loop_circle.{png,pdf} and open_loop_circle.csv.
Run: python -m experiments.phase2_open_loop --config configs/robot_paper.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ddmr.config import build_noise, build_plant, build_trajectory, load_config
from ddmr.controllers.base import Controller
from ddmr.plotting import COLORS, save_fig, setup_style
from ddmr.sim import save_log, simulate

OUT = Path("results/phase2")


class ZeroInput(Controller):
    output = "torque"

    def reset(self) -> None:
        pass

    def compute(self, t, state, ref):
        return np.zeros(2)


def main(config: str = "configs/robot_paper.yaml") -> float:
    cfg = load_config(config)
    traj = build_trajectory(cfg["trajectory"])
    ref0 = traj.sample(0.0)
    v, w = ref0.v, ref0.omega
    x0 = np.array([ref0.x, ref0.y, ref0.phi, v, w])
    s = cfg["sim"]
    df = simulate(build_plant(cfg), ZeroInput(), traj, x0, s["T"], s["Ts"], s["dt"], build_noise(cfg))
    save_log(df, OUT / "open_loop_circle.csv")

    R = v / w
    cx, cy = traj.center
    radius_err = np.hypot(df["x"] - cx, df["y"] - cy) - R
    max_err = float(np.max(np.abs(radius_err)))

    setup_style()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))
    th = np.linspace(0, 2 * np.pi, 400)
    ax1.plot(cx + R * np.cos(th), cy + R * np.sin(th), "--", color=COLORS["reference"],
             label=f"analytic circle, R = v/ω = {R:.2f} m")
    ax1.plot(df["x"], df["y"], color=COLORS["afntsmdc"], lw=1.2, label="simulated, u = 0")
    ax1.plot(x0[0], x0[1], "o", color="k", ms=4)
    ax1.set(xlabel="x (m)", ylabel="y (m)", aspect="equal",
            title=f"Open loop: v = {v:.2f} m/s, ω = {w:.2f} rad/s")
    ax1.legend(loc="center", fontsize=8)
    ax2.plot(df["t"], radius_err * 1e3, color=COLORS["afntsmdc"])
    ax2.set(xlabel="time (s)", ylabel="radius error (mm)",
            title=f"max |radius error| = {max_err:.1e} m")
    save_fig(fig, OUT / "open_loop_circle")
    print(f"open-loop circle: R = {R:.4f} m, max |radius error| = {max_err:.3e} m -> {OUT}/")
    return max_err


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/robot_paper.yaml")
    main(ap.parse_args().config)
