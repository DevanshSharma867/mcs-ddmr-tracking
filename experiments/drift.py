"""Long-horizon drift of the adaptive gains c1..c4 under measurement noise (Phase 5 step 2; Phase 6 arms).

For each controller x scenario x plant: 5 noise seeds + 1 noise-free run of T = 200 s
(configs/drift.yaml), in a process pool. Writes to results/drift/:
<controller>_<scenario>_<plant>_<seed|clean>.csv.gz (t, errors, s, c), gains_<controller>_<scenario>.{png,pdf}
(c_i vs t: seed mean +/- std, noise-free dashed), drift_rates.{csv,md} (slope of c_i over the fit window,
mean +/- std over seeds; projected value at 1 h; % of ticks with |s_i| outside the dead zone; for
projected arms, % of ticks with c_i at its upper bound).
Run: python -m experiments.drift --config configs/drift.yaml
"""

from __future__ import annotations

import argparse
import os
import time
from multiprocessing import Pool
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ddmr.config import build_controller, deep_merge, load_config, simulate_from_config
from ddmr.metrics import markdown_table
from ddmr.plotting import save_fig, setup_style
from ddmr.sim import save_log

OUT = Path("results/drift")
GAINS = ["c1", "c2", "c3", "c4"]
KEEP = ["t", "x_e", "y_e", "phi_e", "s1", "s2", *GAINS]


def job(args: tuple) -> tuple[tuple[str, str, str, str], pd.DataFrame]:
    exp, cid, scen, plant, seed = args
    spec = exp["controllers"][cid]
    gains = deep_merge(load_config(exp["gains"]), spec.get("gains", {}))
    cfg = load_config(exp["scenarios"][scen])
    cfg["plant"] = deep_merge(cfg["plant"], load_config(exp["plants_from"])["plants"][plant] or {})
    cfg["sim"]["T"] = exp["T"]
    cfg["noise"]["std"] = [0.0] * 5 if seed == "clean" else exp["noise_std"]
    cfg["seed"] = 0 if seed == "clean" else seed
    df = simulate_from_config(cfg, build_controller(spec["kind"], gains, cfg))[KEEP]
    save_log(df, OUT / f"{cid}_{scen}_{plant}_{seed}.csv.gz")
    return (cid, scen, plant, str(seed)), df


def rates(runs: dict, exp: dict, gains: dict) -> pd.DataFrame:
    t0, t1 = exp["fit_window"]
    eps, c_max = gains["afntsmdc"]["eps"], np.asarray(gains["improved"]["c_max"], float)
    rows = []
    for (cid, scen, plant, seed), df in runs.items():
        w = df[(df["t"] >= t0) & (df["t"] <= t1)]
        row = {"controller": cid, "scenario": scen, "plant": plant, "seed": seed}
        for i, c in enumerate(GAINS):
            slope = np.polyfit(w["t"], w[c], 1)[0]
            row |= {f"{c}_rate": slope, f"{c}_200s": df[c].iloc[-1],
                    f"{c}_1h": df[c].iloc[-1] + slope * (3600.0 - df["t"].iloc[-1])}
            if exp["controllers"][cid].get("ps"):
                row[f"{c}_at_max_pct"] = 100.0 * float(np.mean(df[c] >= c_max[i] - 1e-9))
                row[f"{c}_out_of_bounds"] = int(np.sum(df[c] > c_max[i] + 1e-9))
        for s in ("s1", "s2"):
            row[f"{s}_outside_pct"] = 100.0 * float(np.mean(np.abs(df[s]) > eps))
        rows.append(row)
    return pd.DataFrame(rows)


def summarize(r: pd.DataFrame) -> pd.DataFrame:
    noisy = r[r.seed != "clean"]
    clean = r[r.seed == "clean"].set_index(["controller", "scenario", "plant"])
    rows = []
    for (cid, scen, plant), g in noisy.groupby(["controller", "scenario", "plant"], sort=False):
        row = {"controller": cid, "scenario": scen, "plant": plant}
        for c in GAINS:
            row[f"{c} rate [1/s] (noisy)"] = f"{g[f'{c}_rate'].mean():.2e} ± {g[f'{c}_rate'].std():.1e}"
            row[f"{c} rate (clean)"] = f"{clean.loc[(cid, scen, plant), f'{c}_rate']:.2e}"
            row[f"{c} at 1 h (noisy)"] = f"{g[f'{c}_1h'].mean():.1f}"
            if f"{c}_at_max_pct" in g and g[f"{c}_at_max_pct"].notna().all():
                row[f"{c} at max %"] = f"{g[f'{c}_at_max_pct'].mean():.0f}"
        row["|s1|>eps %"] = f"{g['s1_outside_pct'].mean():.0f} (clean {clean.loc[(cid, scen, plant), 's1_outside_pct']:.0f})"
        row["|s2|>eps %"] = f"{g['s2_outside_pct'].mean():.0f} (clean {clean.loc[(cid, scen, plant), 's2_outside_pct']:.0f})"
        rows.append(row)
    return pd.DataFrame(rows)


def plot(runs: dict, exp: dict) -> None:
    setup_style()
    for cid in exp["controllers"]:
        for scen in exp["scenarios"]:
            fig, axes = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
            for i, plant in enumerate(exp["plants"]):
                col = plt.cm.tab10(i)
                seeds = [runs[cid, scen, plant, str(s)] for s in exp["seeds"]]
                t = seeds[0]["t"].to_numpy()
                for ax, c in zip(axes, GAINS):
                    vals = np.stack([d[c].to_numpy() for d in seeds])
                    ax.plot(t, vals.mean(0), color=col, lw=1, label=f"{plant} (noise, mean ± std of {len(seeds)} seeds)")
                    ax.fill_between(t, vals.mean(0) - vals.std(0), vals.mean(0) + vals.std(0), color=col, alpha=0.2)
                    ax.plot(t, runs[cid, scen, plant, "clean"][c], color=col, lw=1, ls="--", label=f"{plant} (noise-free)")
                    ax.set_ylabel(f"ĉ{c[1]}")
            axes[0].legend(fontsize=6, ncol=2)
            axes[0].set_title(f"{cid}, {scen}: adaptive gains over {exp['T']:.0f} s", fontsize=10)
            axes[-1].set_xlabel("Time (s)")
            save_fig(fig, OUT / f"gains_{cid}_{scen}")


def main(config: str = "configs/drift.yaml", workers: int | None = None) -> pd.DataFrame:
    exp = load_config(config)
    todo = [(exp, cid, scen, plant, seed) for cid in exp["controllers"] for scen in exp["scenarios"]
            for plant in exp["plants"] for seed in [*exp["seeds"], "clean"]]
    t0 = time.time()
    with Pool(workers or os.cpu_count()) as pool:
        runs = dict(pool.map(job, todo))
    print(f"{len(runs)} runs in {time.time() - t0:.0f} s")
    gains = load_config(exp["gains"])
    r = rates(runs, exp, gains)
    r.to_csv(OUT / "drift_rates.csv", index=False)
    s = summarize(r)
    (OUT / "drift_rates.md").write_text(
        f"# Adaptive-gain drift, T = {exp['T']:.0f} s\n\n"
        f"Rate = slope of a linear fit over t in {exp['fit_window']} s, mean ± std over seeds "
        f"{exp['seeds']} (noise std {exp['noise_std']}). 'at 1 h' = c(200 s) + rate x 3400 s. "
        f"|s|>eps % = share of ticks outside the dead zone (eps = {gains['afntsmdc']['eps']}). "
        "'at max %' = share of ticks with c_i at its projection bound (projected arms only).\n\n"
        + markdown_table(s) + "\n")
    plot(runs, exp)
    return s


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/drift.yaml")
    main(ap.parse_args().config)
