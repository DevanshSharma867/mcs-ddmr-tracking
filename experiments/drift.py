"""Phase 5 step 2: long-horizon drift of AFNTSMDC's adaptive gains c1..c4 under measurement noise.

For each scenario x plant: 5 noise seeds + 1 noise-free run of T = 200 s (configs/drift.yaml).
Writes to results/drift/: <scenario>_<plant>_<seed|clean>.csv.gz (t, errors, s, c),
gains_<scenario>.{png,pdf} (c_i vs t: seed mean +/- std, noise-free dashed), drift_rates.{csv,md}
(slope of c_i over the fit window, mean +/- std over seeds; projected value at 1 h; fraction of
ticks with |s_i| outside the dead zone).
Run: python -m experiments.drift --config configs/drift.yaml
"""

from __future__ import annotations

import argparse
import time
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


def run(exp: dict) -> dict[tuple[str, str, str], pd.DataFrame]:
    gains = load_config(exp["gains"])
    plant_defs = load_config(exp["plants_from"])["plants"]
    runs = {}
    for scen, path in exp["scenarios"].items():
        for plant in exp["plants"]:
            for seed in [*exp["seeds"], "clean"]:
                cfg = load_config(path)
                cfg["plant"] = deep_merge(cfg["plant"], plant_defs[plant] or {})
                cfg["sim"]["T"] = exp["T"]
                cfg["noise"]["std"] = [0.0] * 5 if seed == "clean" else exp["noise_std"]
                cfg["seed"] = 0 if seed == "clean" else seed
                df = simulate_from_config(cfg, build_controller(exp["controller"], gains, cfg))[KEEP]
                save_log(df, OUT / f"{scen}_{plant}_{seed}.csv.gz")
                runs[scen, plant, str(seed)] = df
    return runs


def rates(runs: dict, exp: dict, eps: float) -> pd.DataFrame:
    t0, t1 = exp["fit_window"]
    rows = []
    for (scen, plant, seed), df in runs.items():
        w = df[(df["t"] >= t0) & (df["t"] <= t1)]
        row = {"scenario": scen, "plant": plant, "seed": seed}
        for c in GAINS:
            slope = np.polyfit(w["t"], w[c], 1)[0]
            row |= {f"{c}_rate": slope, f"{c}_200s": df[c].iloc[-1],
                    f"{c}_1h": df[c].iloc[-1] + slope * (3600.0 - df["t"].iloc[-1])}
        for s in ("s1", "s2"):
            row[f"{s}_outside_pct"] = 100.0 * float(np.mean(np.abs(df[s]) > eps))
        rows.append(row)
    return pd.DataFrame(rows)


def summarize(r: pd.DataFrame) -> pd.DataFrame:
    noisy, clean = r[r.seed != "clean"], r[r.seed == "clean"].set_index(["scenario", "plant"])
    rows = []
    for (scen, plant), g in noisy.groupby(["scenario", "plant"], sort=False):
        row = {"scenario": scen, "plant": plant}
        for c in GAINS:
            row[f"{c} rate [1/s] (noisy)"] = f"{g[f'{c}_rate'].mean():.2e} ± {g[f'{c}_rate'].std():.1e}"
            row[f"{c} rate (clean)"] = f"{clean.loc[(scen, plant), f'{c}_rate']:.2e}"
            row[f"{c} at 1 h (noisy)"] = f"{g[f'{c}_1h'].mean():.1f}"
        row["|s1|>eps %"] = f"{g['s1_outside_pct'].mean():.0f} (clean {clean.loc[(scen, plant), 's1_outside_pct']:.0f})"
        row["|s2|>eps %"] = f"{g['s2_outside_pct'].mean():.0f} (clean {clean.loc[(scen, plant), 's2_outside_pct']:.0f})"
        rows.append(row)
    return pd.DataFrame(rows)


def plot(runs: dict, exp: dict) -> None:
    setup_style()
    for scen in exp["scenarios"]:
        fig, axes = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
        for i, plant in enumerate(exp["plants"]):
            col = plt.cm.tab10(i)
            seeds = [runs[scen, plant, str(s)] for s in exp["seeds"]]
            t = seeds[0]["t"].to_numpy()
            for ax, c in zip(axes, GAINS):
                vals = np.stack([d[c].to_numpy() for d in seeds])
                ax.plot(t, vals.mean(0), color=col, lw=1, label=f"{plant} (noise, mean ± std of {len(seeds)} seeds)")
                ax.fill_between(t, vals.mean(0) - vals.std(0), vals.mean(0) + vals.std(0), color=col, alpha=0.2)
                ax.plot(t, runs[scen, plant, "clean"][c], color=col, lw=1, ls="--",
                        label=f"{plant} (noise-free)")
                ax.set_ylabel(f"ĉ{c[1]}")
        axes[0].legend(fontsize=6, ncol=2)
        axes[0].set_title(f"{scen}: AFNTSMDC adaptive gains over {exp['T']:.0f} s", fontsize=10)
        axes[-1].set_xlabel("Time (s)")
        save_fig(fig, OUT / f"gains_{scen}")


def main(config: str = "configs/drift.yaml") -> pd.DataFrame:
    exp = load_config(config)
    t0 = time.time()
    runs = run(exp)
    print(f"{len(runs)} runs in {time.time() - t0:.0f} s")
    eps = load_config(exp["gains"])["afntsmdc"]["eps"]
    r = rates(runs, exp, eps)
    r.to_csv(OUT / "drift_rates.csv", index=False)
    s = summarize(r)
    (OUT / "drift_rates.md").write_text(
        f"# AFNTSMDC adaptive-gain drift, T = {exp['T']:.0f} s\n\n"
        f"Rate = slope of a linear fit over t in {exp['fit_window']} s, mean ± std over seeds "
        f"{exp['seeds']} (noise std {exp['noise_std']}). 'at 1 h' = c(200 s) + rate x 3400 s. "
        f"|s|>eps % = share of ticks outside the dead zone (eps = {eps}).\n\n" + markdown_table(s) + "\n")
    plot(runs, exp)
    print(s.to_string(index=False))
    return s


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/drift.yaml")
    main(ap.parse_args().config)
