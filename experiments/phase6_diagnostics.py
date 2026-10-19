"""Phase 6 DIAGNOSTICS (explanatory only; not part of the pre-registered evaluation).

1. Actuator lag: A3 vs A3 with the ESO fed a model of the known lag (improved.eso_actuator_tau = tau_a),
   on the grid's lag_20ms / lag_40ms conditions (torque plant, circle + figure-8, seeds 0-4) and on
   reproduction S1 with the plant lag (40 ms).
2. motor18V: every controller on reproduction S1-S3 (from results/reproduction/summary.csv), to tell
   plant infeasibility from an observer-specific failure.
Writes results/phase6/diagnostics.md and diagnostics_{lag,motor18V}.csv. Run after experiments.reproduction.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from ddmr.config import build_controller, deep_merge, load_config, simulate_from_config
from ddmr.metrics import input_metrics, markdown_table
from experiments.robustness import build_cfg

OUT = Path("results/phase6")
VARIANTS = {"A3": {}, "A3 + lag-aware ESO (diagnostic)": None}   # None -> tau_a of the condition


def lag_diag() -> pd.DataFrame:
    exp = load_config("configs/robustness.yaml")
    base = load_config(exp["gains"])
    rows = []
    for cond in ("lag_20ms", "lag_40ms"):
        tau_a = exp["conditions"][cond]["plant"]["actuator_tau"]
        for traj in exp["trajectories"]:
            for name, over in VARIANTS.items():
                vals, chat = [], []
                for seed in exp["seeds"]:
                    cfg = build_cfg(exp, "torque", traj, cond, seed)
                    g = deep_merge(base, {"improved": {"eso_actuator_tau": tau_a}} if over is None else {})
                    df = simulate_from_config(cfg, build_controller("afntsmdc_improved", g, cfg))
                    late = df[df["t"] >= exp["T"] - 5.0]
                    vals.append(np.hypot(late["x_e"], late["y_e"]).max() * 100)
                    chat.append(input_metrics(df, cfg["sim"]["Ts"], cfg["plant"]["u_max"])["chattering"])
                rows.append({"setting": f"grid {cond}, {traj}", "controller": name,
                             "late err [cm] mean ± std": f"{np.mean(vals):.2f} ± {np.std(vals, ddof=1):.2f}",
                             "chattering": f"{np.mean(chat):.0f}"})
    for name, over in VARIANTS.items():
        cfg = load_config("configs/robot_paper.yaml")
        cfg["plant"]["actuator_tau"] = 0.040
        g = deep_merge(base, {"improved": {"eso_actuator_tau": 0.040}} if over is None else {})
        df = simulate_from_config(cfg, build_controller("afntsmdc_improved", g, cfg))
        late = df[df["t"] >= 15.0]
        rows.append({"setting": "reproduction S1, lag 40 ms (no noise)", "controller": name,
                     "late err [cm] mean ± std": f"{np.hypot(late['x_e'], late['y_e']).max() * 100:.2f}",
                     "chattering": f"{input_metrics(df, 0.01, 8.0)['chattering']:.0f}"})
    return pd.DataFrame(rows)


def motor18v_diag() -> pd.DataFrame:
    s = pd.read_csv("results/reproduction/summary.csv")
    order = ["ckc_pi", "rsmkc_pi", "csmdc", "afntsmdc", "afn_A1", "afn_A2", "afn_A3", "afn_A4"]
    d = s[(s.plant == "motor18V") & s.controller.isin(order)].copy()
    d["o"] = d.controller.map({c: i for i, c in enumerate(order)})
    d = d.sort_values(["scenario", "o"])
    return d[["scenario", "label", "late_pos_err_max_cm", "x_e_rms", "y_e_rms", "saturated_pct"]].round(2)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    lag, m18 = lag_diag(), motor18v_diag()
    lag.to_csv(OUT / "diagnostics_lag.csv", index=False)
    m18.to_csv(OUT / "diagnostics_motor18V.csv", index=False)
    (OUT / "diagnostics.md").write_text(
        "# Phase 6 diagnostics (explanatory only, not pre-registered)\n\n"
        "## 1. Actuator lag: ESO fed a model of the known lag\n\n" + markdown_table(lag) + "\n\n"
        "## 2. motor18V: all controllers, reproduction S1-S3\n\n"
        "late_pos_err = max position error for t >= 15 s [cm]; RMS in cm; saturated = % of ticks with "
        "|u_cmd| > 8 V.\n\n" + markdown_table(m18) + "\n")
    print(lag.to_string(index=False))
    print(m18.to_string(index=False))


if __name__ == "__main__":
    main()
