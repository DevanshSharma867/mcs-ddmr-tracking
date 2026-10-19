"""Phase 3: baselines on scenario S1 (paper circle, initial offset (-0.1, -0.1, 0)).

Controllers: ideal kinematic reference (CKC, perfect velocity tracking), CKC+PI, RSMKC+PI, CSMDC,
plus "CSMDC+BL (diagnostic)" (CSMDC with AFNTSMDC's boundary layer; not a paper baseline).
Writes to results/baselines/: <controller>.csv logs, paths/errors/inputs figures (png+pdf, laid out
like the paper's Fig. 5a, 5c-e, 6), metrics.csv and metrics.md (with the paper's Table 2 alongside).
Run: python -m experiments.baselines --config configs/robot_paper.yaml --gains configs/controllers_paper.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ddmr.config import build_controller, build_trajectory, load_config, simulate_from_config
from ddmr.metrics import input_metrics, markdown_table, tracking_metrics
from ddmr.paper_tables import TABLE2
from ddmr.plotting import COLORS, INPUT_LABEL, save_fig, setup_style
from ddmr.sim import save_log

OUT = Path("results/baselines")
CONTROLLERS = {  # name -> (label, color key, paper Table 2 key)
    "ideal": ("Ideal kinematic (CKC, perfect z)", "kinematic", None),
    "ckc_pi": ("CKC+PI", "ckc", "ckc"),
    "rsmkc_pi": ("RSMKC+PI", "rsmkc", "rsmkc"),
    "csmdc": ("CSMDC", "csmdc", "csmdc"),
    "csmdc_bl": ("CSMDC+BL (diagnostic)", "csmdc_bl", None),
}
ERRORS = {"x_e": ("X-axis error (m)", 100.0, "cm"), "y_e": ("Y-axis error (m)", 100.0, "cm"),
          "phi_e": ("Orientation angle error (rad)", 1.0, "rad")}


def metrics_table(runs: dict[str, pd.DataFrame], Ts: float, u_max: float | None) -> pd.DataFrame:
    rows = []
    for name, df in runs.items():
        label, _, paper_key = CONTROLLERS[name]
        tm = tracking_metrics(df)
        im = (input_metrics(df, Ts, u_max) if name != "ideal"
              else {"effort": np.nan, "chattering": np.nan, "saturated_pct": np.nan})
        for err, (_, scale, unit) in ERRORS.items():
            paper = TABLE2[paper_key][err] if paper_key else (np.nan,) * 3
            rows.append({"controller": label, "error": err, "unit": unit,
                         "RMS": tm[err]["rms"] * scale, "MAX": tm[err]["max"] * scale,
                         "settle_s": tm[err]["settle"],
                         "paper_RMS": paper[0], "paper_MAX": paper[1], "paper_settle_s": paper[2],
                         **im})
    return pd.DataFrame(rows)


def plot(runs: dict[str, pd.DataFrame], traj) -> None:
    setup_style()
    t_ref = np.linspace(0, 2 * np.pi / traj.omega, 400)
    ref = np.array([[traj.sample(t).x, traj.sample(t).y] for t in t_ref])

    fig, ax = plt.subplots(figsize=(5.5, 5))
    for name, df in runs.items():
        label, color, _ = CONTROLLERS[name]
        ax.plot(df["x"], df["y"], color=COLORS[color], lw=1.2, label=label)
    ax.plot(ref[:, 0], ref[:, 1], "--", color=COLORS["reference"], lw=1, label="Reference")
    ax.set(xlabel="X-axis (m)", ylabel="Y-axis (m)", aspect="equal", title="S1: trajectories (cf. Fig. 5a)")
    ax.legend(loc="center", fontsize=8)
    save_fig(fig, OUT / "paths")

    fig, axes = plt.subplots(3, 1, figsize=(7, 7.5), sharex=True)
    for ax, (err, (ylabel, _, _)) in zip(axes, ERRORS.items()):
        for name, df in runs.items():
            label, color, _ = CONTROLLERS[name]
            ax.plot(df["t"], df[err], color=COLORS[color], lw=1.1, label=label)
        ax.set_ylabel(ylabel)
    axes[0].set_title("S1: tracking errors (cf. Fig. 5c-e)")
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, OUT / "errors")

    torque_runs = {n: df for n, df in runs.items() if n != "ideal"}
    fig, axes = plt.subplots(len(torque_runs), 1, figsize=(7, 2.2 * len(torque_runs)), sharex=True)
    for ax, (name, df) in zip(axes, torque_runs.items()):
        label, color, _ = CONTROLLERS[name]
        ax.plot(df["t"], df["ua_R"], color=COLORS[color], lw=0.7, label="right")
        ax.plot(df["t"], df["ua_L"], color=COLORS[color], lw=0.7, ls="--", alpha=0.7, label="left")
        ax.set_ylabel(f"{label}\ninput", fontsize=9)
        ax.legend(fontsize=7, loc="upper right")
    axes[0].set_title(f"S1: applied wheel inputs, {INPUT_LABEL} (cf. Fig. 6)", fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, OUT / "inputs")


def main(config: str = "configs/robot_paper.yaml", gains: str = "configs/controllers_paper.yaml") -> pd.DataFrame:
    cfg, g = load_config(config), load_config(gains)
    runs = {}
    for name in CONTROLLERS:
        runs[name] = simulate_from_config(cfg, build_controller(name, g, cfg))
        save_log(runs[name], OUT / f"{name}.csv")
    table = metrics_table(runs, cfg["sim"]["Ts"], cfg["plant"].get("u_max"))
    table.to_csv(OUT / "metrics.csv", index=False)
    (OUT / "metrics.md").write_text(
        "# S1 baselines: metrics (ours vs paper Table 2)\n\n"
        "x_e, y_e in cm; phi_e in rad; settling band 1 cm / 0.02 rad (M6), NaN = not settled.\n"
        "effort [model units^2 s], chattering [model units/s], saturated [% of ticks].\n\n"
        + markdown_table(table) + "\n")
    plot(runs, build_trajectory(cfg["trajectory"]))
    print(table.to_string(index=False, float_format=lambda v: f"{v:.3f}"))
    return table


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/robot_paper.yaml")
    ap.add_argument("--gains", default="configs/controllers_paper.yaml")
    a = ap.parse_args()
    main(a.config, a.gains)
