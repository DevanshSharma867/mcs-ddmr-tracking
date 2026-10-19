"""Phase 5 step 3: robustness grid (configs/robustness.yaml), one factor at a time around nominal.

Jobs = plants x trajectories x conditions x controllers x seeds, run in a process pool.
Writes to results/robustness/: runs.csv (metrics of every run), summary.csv / summary_<plant>_<traj>.md
(mean +/- std over seeds), heatmaps per plant x trajectory of position RMS, degradation vs nominal
and chattering, and seed-0 logs (reduced columns) under logs/.
Run: python -m experiments.robustness --config configs/robustness.yaml [--workers N]
"""

from __future__ import annotations

import argparse
import copy
import os
import time
from multiprocessing import Pool
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ddmr.config import build_controller, deep_merge, load_config, simulate_from_config
from ddmr.metrics import input_metrics, markdown_table, tracking_metrics
from ddmr.plotting import save_fig, setup_style
from ddmr.sim import save_log

OUT = Path("results/robustness")
LOG_COLS = ["t", "x", "y", "x_e", "y_e", "phi_e", "ua_R", "ua_L"]


def accel_to_input(a: float, plant_cfg: dict, robot: dict) -> float:
    """Per-wheel input giving equivalent acceleration a [m/s^2] on the nominal mass (log: option B).
    Torque plant: a = 2 du / (m0 r). Motor plant (stall force): a = 2 n kt du / (R r m0)."""
    m0, r = robot["nominal"]["m"], robot["nominal"]["r"]
    mo = plant_cfg.get("motor")
    return a * m0 * r / 2.0 if not mo else a * mo["R"] * r * m0 / (2.0 * mo["n"] * mo["kt"])


def build_cfg(exp: dict, plant: str, traj: str, cond: str, seed: int) -> dict:
    cfg = load_config(exp["scenario"])
    cfg["plant"] = deep_merge(cfg["plant"], exp["plants"][plant] or {})
    cfg["trajectory"] = dict(exp["trajectories"][traj]["trajectory"])        # replace, not merge
    cfg["initial_state"] = list(exp["trajectories"][traj]["initial_state"])
    c = exp["conditions"][cond]
    cfg = deep_merge(cfg, {k: v for k, v in c.items() if k in ("robot", "plant")})
    dists = []
    for d in c.get("accel_disturbances", []):
        amp = accel_to_input(d["accel"] * exp["a0"], cfg["plant"], cfg["robot"])
        spec = {k: v for k, v in d.items() if k != "accel"}
        dists.append({**spec, "amplitude": amp})
    cfg["plant"]["disturbances"] = dists
    cfg["noise"]["std"] = [s * c.get("noise_scale", 1.0) for s in exp["noise_std"]]
    cfg["sim"]["T"] = exp["T"]
    cfg["seed"] = seed
    return cfg


def job(args: tuple) -> dict:
    exp, plant, traj, cond, cid, seed = args
    cfg = build_cfg(exp, plant, traj, cond, seed)
    spec = exp["controllers"][cid]
    gains = deep_merge(load_config(exp["gains"]), spec.get("gains", {}))
    row = {"plant": plant, "trajectory": traj, "condition": cond,
           "factor": exp["conditions"][cond]["factor"], "controller": cid, "seed": seed}
    try:
        df = simulate_from_config(cfg, build_controller(spec["kind"], gains, cfg))
    except FloatingPointError:
        return row | {"diverged": True}
    if seed == exp["seeds"][0]:
        save_log(df[LOG_COLS], OUT / "logs" / f"{plant}_{traj}_{cond}_{cid}.csv.gz")
    tm, im = tracking_metrics(df), input_metrics(df, cfg["sim"]["Ts"], cfg["plant"]["u_max"])
    pos = np.hypot(df["x_e"], df["y_e"])
    late = df["t"] >= exp["T"] - 5.0
    row |= {"diverged": False, "pos_rms_cm": float(np.sqrt(np.mean(pos**2)) * 100),
            "pos_late_max_cm": float(pos[late].max() * 100),
            "phi_rms": tm["phi_e"]["rms"], "x_e_settle": tm["x_e"]["settle"],
            "y_e_settle": tm["y_e"]["settle"], "phi_e_settle": tm["phi_e"]["settle"], **im}
    if "c1" in df:
        row |= {f"c{i}_final": df[f"c{i}"].iloc[-1] for i in range(1, 5)}
    return row


def jobs(exp: dict) -> list[tuple]:
    out = []
    for plant in exp["plants"]:
        for traj in exp["trajectories"]:
            for cond, c in exp["conditions"].items():
                if plant not in c.get("only_plants", exp["plants"]):
                    continue
                out += [(exp, plant, traj, cond, cid, s) for cid in exp["controllers"] for s in exp["seeds"]]
    return out


def heatmap(tab: pd.DataFrame, title: str, stem: Path, fmt: str, log: bool, cmap: str) -> None:
    fig, ax = plt.subplots(figsize=(1.5 + 1.3 * tab.shape[1], 0.9 + 0.32 * tab.shape[0]))
    vals = tab.to_numpy(dtype=float)
    shown = np.log10(np.clip(vals, 1e-3, None)) if log else vals
    im = ax.imshow(shown, aspect="auto", cmap=cmap)
    for (i, j), v in np.ndenumerate(vals):
        ax.text(j, i, "div" if np.isnan(v) else format(v, fmt), ha="center", va="center", fontsize=6)
    ax.set_xticks(range(tab.shape[1]), tab.columns, rotation=30, ha="right", fontsize=7)
    ax.set_yticks(range(tab.shape[0]), tab.index, fontsize=7)
    ax.grid(False)
    fig.colorbar(im, ax=ax, label="log10" if log else "")
    ax.set_title(title, fontsize=9)
    save_fig(fig, stem)


def report(runs: pd.DataFrame, exp: dict) -> pd.DataFrame:
    labels = {c: s["label"] for c, s in exp["controllers"].items()}
    ok = runs[~runs.diverged]
    g = ok.groupby(["plant", "trajectory", "condition", "controller"], sort=False)
    summ = g[["pos_rms_cm", "pos_late_max_cm", "phi_rms", "chattering", "effort", "saturated_pct"]].agg(["mean", "std"])
    summ.columns = [f"{a}_{b}" for a, b in summ.columns]
    summ["n_diverged"] = runs.groupby(["plant", "trajectory", "condition", "controller"], sort=False)["diverged"].sum()
    summ = summ.reset_index()
    summ.to_csv(OUT / "summary.csv", index=False)
    setup_style()
    order = list(exp["conditions"])
    for plant in exp["plants"]:
        for traj in exp["trajectories"]:
            s = summ[(summ.plant == plant) & (summ.trajectory == traj)]
            piv = s.pivot(index="condition", columns="controller", values="pos_rms_cm_mean")
            piv = piv.reindex([c for c in order if c in piv.index])[list(exp["controllers"])].rename(columns=labels)
            chat = s.pivot(index="condition", columns="controller", values="chattering_mean").reindex(piv.index)[list(exp["controllers"])].rename(columns=labels)
            ratio = piv / piv.loc["nominal"]
            tag = f"{plant}_{traj}"
            heatmap(piv, f"{plant}, {traj}: position RMS [cm] (mean of {len(exp['seeds'])} seeds)", OUT / f"pos_rms_{tag}", ".2g", True, "viridis")
            heatmap(ratio, f"{plant}, {traj}: position RMS / nominal (same controller)", OUT / f"degradation_{tag}", ".2f", True, "coolwarm")
            heatmap(chat, f"{plant}, {traj}: chattering index (input TV/s)", OUT / f"chattering_{tag}", ".3g", True, "magma")
            cell = s.assign(v=s.pos_rms_cm_mean.map("{:.2f}".format) + " ± " + s.pos_rms_cm_std.map("{:.2f}".format)
                            + np.where(s.n_diverged > 0, " (div " + s.n_diverged.astype(int).astype(str) + ")", ""))
            md = cell.pivot(index="condition", columns="controller", values="v").reindex(piv.index)[list(exp["controllers"])].rename(columns=labels)
            (OUT / f"summary_{tag}.md").write_text(
                f"# {plant}, {traj}: position RMS [cm], mean ± std over seeds {exp['seeds']}\n\n"
                + markdown_table(md.reset_index()) + "\n")
    return summ


def main(config: str = "configs/robustness.yaml", workers: int | None = None) -> pd.DataFrame:
    exp = load_config(config)
    todo = jobs(exp)
    t0 = time.time()
    with Pool(workers or os.cpu_count()) as pool:
        rows = pool.map(job, todo, chunksize=4)
    runs = pd.DataFrame(rows)
    OUT.mkdir(parents=True, exist_ok=True)
    runs.to_csv(OUT / "runs.csv", index=False)
    print(f"{len(todo)} runs in {time.time() - t0:.0f} s ({int(runs.diverged.sum())} diverged)")
    return report(runs, exp)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/robustness.yaml")
    ap.add_argument("--workers", type=int, default=None)
    a = ap.parse_args()
    main(a.config, a.workers)
