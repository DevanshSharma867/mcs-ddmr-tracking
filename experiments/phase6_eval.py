"""Phase 6: evaluate the pre-registered criteria P1-P7 (docs/phase6_preregistration.md) and the ablation.

Reads results/robustness/runs.csv, results/reproduction/ (summary.csv and S2/S3 logs),
results/drift/drift_rates.csv and results/phase6/omega_sweep/runs.csv; writes results/phase6/:
criteria.{md,csv}, ablation_<plant>_<traj>.md, ablation_<plant>_<traj>.{png,pdf}, omega_sweep.md.
Run after: experiments.robustness, experiments.reproduction, experiments.drift and
experiments.robustness --config configs/phase6_omega_sweep.yaml.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ddmr.config import load_config
from ddmr.metrics import markdown_table
from ddmr.plotting import COLORS, save_fig, setup_style

OUT = Path("results/phase6")
ARMS = ["afntsmdc", "afn_A1", "afn_A2", "afn_A3", "afn_A4"]
NAMES = {"afntsmdc": "A0 reproduced", "afn_A1": "A1 +ESO", "afn_A2": "A2 +PS", "afn_A3": "A3 full",
         "afn_A4": "A4 ESO, no switching", "ckc_pi": "CKC+PI"}
TRAJS = ["circle", "figure8"]


def stats(runs: pd.DataFrame) -> pd.DataFrame:
    g = runs.groupby(["plant", "trajectory", "condition", "controller"])
    return g[["pos_late_max_cm", "chattering"]].agg(["mean", "std"])


def get(st: pd.DataFrame, plant: str, traj: str, cond: str, cid: str, col: str = "pos_late_max_cm"):
    r = st.loc[(plant, traj, cond, cid)]
    return float(r[(col, "mean")]), float(r[(col, "std")])


def not_worse(a: tuple, b: tuple) -> tuple[bool, float]:
    """A not significantly worse than B under the 2x seed-std rule."""
    tol = 2.0 * max(a[1], b[1])
    return a[0] - b[0] <= tol, tol


def shock_xe(plant: str, cid: str) -> float:
    s2 = pd.read_csv(f"results/reproduction/{plant}/S2/{cid}.csv.gz")
    s3 = pd.read_csv(f"results/reproduction/{plant}/S3/{cid}.csv.gz")
    w = (s3["t"] >= 12.0) & (s3["t"] <= 20.0)
    return float((s3["x_e"] - s2["x_e"])[w].abs().max() * 100)


def criteria(st: pd.DataFrame, runs: pd.DataFrame) -> pd.DataFrame:
    rows = []

    def add(pid, item, value, threshold, ok, note=""):
        rows.append({"criterion": pid, "item": item, "A3": value, "threshold / reference": threshold,
                     "pass": "PASS" if ok else "FAIL", "note": note})

    for traj in TRAJS:
        for cond in ["dist_const", "dist_yaw", "friction_10", "friction_30"]:
            m, s = get(st, "torque", traj, cond, "afn_A3")
            a0 = get(st, "torque", traj, cond, "afntsmdc")[0]
            add("P1", f"{traj} {cond} late err [cm]", f"{m:.3f} ± {s:.3f}", "≤ 0.5", m <= 0.5, f"A0 {a0:.2f}")
    for traj in TRAJS:
        for cond in ["dist_sin1Hz", "dist_sin0.2Hz", "dist_bandlim"]:
            a3, a0 = get(st, "torque", traj, cond, "afn_A3"), get(st, "torque", traj, cond, "afntsmdc")
            ok, tol = not_worse(a3, a0)
            add("P2", f"{traj} {cond} late err [cm]", f"{a3[0]:.3f} ± {a3[1]:.3f}",
                f"A0 {a0[0]:.3f} ± {a0[1]:.3f} (+tol {tol:.3f})", ok)
    for traj in TRAJS:
        nom = get(st, "torque", traj, "nominal", "afn_A3", "chattering")
        a0 = get(st, "torque", traj, "nominal", "afntsmdc", "chattering")[0]
        add("P3a", f"{traj} nominal chattering", f"{nom[0]:.1f} ± {nom[1]:.1f}", "≤ 100", nom[0] <= 100, f"A0 {a0:.0f}")
        for cond in ["mass_x0.7", "noise_x2"]:
            c = get(st, "torque", traj, cond, "afn_A3", "chattering")
            add("P3b", f"{traj} {cond} chattering", f"{c[0]:.1f} ± {c[1]:.1f}", f"≤ 2 × {nom[0]:.1f}", c[0] <= 2 * nom[0])
    rep = pd.read_csv("results/reproduction/summary.csv")
    st_x = float(rep[(rep.plant == "ideal") & (rep.scenario == "S1") & (rep.controller == "afn_A3")]["x_e_settle"].iloc[0])
    add("P4a", "S1 ideal x_e settling [s]", f"{st_x:.2f}", "≤ 1.36", st_x <= 1.36)
    for traj in TRAJS:
        a3, a0 = get(st, "torque", traj, "nominal", "afn_A3"), get(st, "torque", traj, "nominal", "afntsmdc")
        add("P4b", f"{traj} nominal late err [cm]", f"{a3[0]:.3f} ± {a3[1]:.3f}", f"≤ A0 {a0[0]:.3f}", a3[0] <= a0[0])
    a3c = get(st, "motor12V", "circle", "nominal", "afn_A3")
    add("P5", "motor12V circle nominal late err [cm]", f"{a3c[0]:.3f} ± {a3c[1]:.3f}", "≤ 2.0", a3c[0] <= 2.0)
    a3f, a0f = get(st, "motor12V", "figure8", "nominal", "afn_A3"), get(st, "motor12V", "figure8", "nominal", "afntsmdc")
    add("P5", "motor12V figure8 nominal late err [cm]", f"{a3f[0]:.3f} ± {a3f[1]:.3f}", f"≤ A0 {a0f[0]:.3f}", a3f[0] <= a0f[0])
    for traj, a3x in (("circle", a3c), ("figure8", a3f)):
        ck = get(st, "motor12V", traj, "nominal", "ckc_pi")
        tol = 2 * max(a3x[1], ck[1])
        verdict = ("A3 better" if ck[0] - a3x[0] > tol else "CKC+PI better" if a3x[0] - ck[0] > tol else "no significant difference")
        rows.append({"criterion": "P5 (report)", "item": f"motor12V {traj}: A3 vs CKC+PI [cm]",
                     "A3": f"{a3x[0]:.3f} ± {a3x[1]:.3f}", "threshold / reference": f"CKC+PI {ck[0]:.3f} ± {ck[1]:.3f}",
                     "pass": "—", "note": verdict})
    for plant in ("ideal", "motor12V"):
        a3, a0 = shock_xe(plant, "afn_A3"), shock_xe(plant, "afntsmdc")
        add("P6", f"{plant} S3 shock-induced x_e [cm]", f"{a3:.2f}", f"≤ A0 {a0:.2f}", a3 <= a0)
    gains = load_config("configs/controllers_paper.yaml")["improved"]
    cmin, cmax = np.asarray(gains["c_min"]), np.asarray(gains["c_max"])
    ps = runs[runs.controller.isin(["afn_A2", "afn_A3"])]
    hi = ps[[f"c{i}_maxseen" for i in range(1, 5)]].to_numpy() > cmax + 1e-9
    lo = ps[[f"c{i}_minseen" for i in range(1, 5)]].to_numpy() < cmin - 1e-9
    dr = pd.read_csv("results/drift/drift_rates.csv")
    dr3 = dr[dr.controller == "afn_A3"]
    oob = int(dr3[[f"c{i}_out_of_bounds" for i in range(1, 5)]].to_numpy().sum())
    add("P7", "ĉ within [ĉ_min, ĉ_max] (grid A2/A3 + drift A3)",
        f"{int(hi.sum() + lo.sum())} grid violations / {len(ps)} runs; {oob} drift ticks", "0", hi.sum() + lo.sum() + oob == 0)
    noisy = dr3[dr3.seed != "clean"]
    at_max = ", ".join(f"ĉ{i} {noisy[f'c{i}_at_max_pct'].mean():.0f}%" for i in range(1, 5))
    rows.append({"criterion": "P7 (report)", "item": "time at ĉ_max, 200 s drift (A3, noisy, all plants/scenarios)",
                 "A3": at_max, "threshold / reference": "—", "pass": "—", "note": ""})
    return pd.DataFrame(rows)


def ablation(st: pd.DataFrame) -> None:
    conds = ["nominal", "noise_x0", "noise_x2", "mass_x0.7", "dist_const", "dist_yaw", "dist_sin1Hz",
             "dist_sin0.2Hz", "dist_bandlim", "friction_10", "friction_30", "lag_40ms", "h_terms"]
    setup_style()
    for plant in ("torque", "motor12V"):
        for traj in TRAJS:
            rows = []
            for cond in conds:
                if (plant, traj, cond, "afn_A3") not in st.index:
                    continue
                row = {"condition": cond}
                for cid in ["ckc_pi", *ARMS]:
                    m, s = get(st, plant, traj, cond, cid)
                    c = get(st, plant, traj, cond, cid, "chattering")[0]
                    row[NAMES[cid]] = f"{m:.2f} ± {s:.2f} ({c:.0f})"
                rows.append(row)
            tab = pd.DataFrame(rows)
            (OUT / f"ablation_{plant}_{traj}.md").write_text(
                f"# Ablation, plant {plant}, {traj}: late position error [cm] mean ± std (chattering index)\n\n"
                + markdown_table(tab) + "\n")
            fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
            x = np.arange(len(tab))
            wdt = 0.8 / len(ARMS)
            for k, cid in enumerate(ARMS):
                late = [get(st, plant, traj, c, cid)[0] for c in tab.condition]
                chat = [get(st, plant, traj, c, cid, "chattering")[0] for c in tab.condition]
                col = COLORS.get(cid, "tab:blue")
                axes[0].bar(x + k * wdt, late, wdt, color=col, label=NAMES[cid])
                axes[1].bar(x + k * wdt, chat, wdt, color=col)
            axes[0].set(ylabel="late position error [cm]", yscale="log",
                        title=f"Ablation, plant {plant}, {traj} (mean of 5 seeds)")
            axes[1].set(ylabel="chattering index", yscale="log")
            axes[1].set_xticks(x + 0.4 - wdt / 2, tab.condition, rotation=30, ha="right", fontsize=8)
            axes[0].legend(fontsize=7, ncol=5)
            save_fig(fig, OUT / f"ablation_{plant}_{traj}")


def omega_sweep() -> None:
    r = pd.read_csv("results/phase6/omega_sweep/runs.csv")
    g = r.groupby(["plant", "trajectory", "condition", "controller"])[["pos_late_max_cm", "chattering"]].agg(["mean", "std"])
    g.columns = [f"{a}_{b}" for a, b in g.columns]
    g = g.reset_index().round(3)
    (OUT / "omega_sweep.md").write_text(
        "# ESO bandwidth sweep for A3 (descriptive only; omega_o = 25 was fixed in advance)\n\n"
        + markdown_table(g) + "\n")


def main() -> pd.DataFrame:
    OUT.mkdir(parents=True, exist_ok=True)
    runs = pd.read_csv("results/robustness/runs.csv")
    st = stats(runs)
    crit = criteria(st, runs)
    crit.to_csv(OUT / "criteria.csv", index=False)
    (OUT / "criteria.md").write_text("# Phase 6 pre-registered criteria (A3 = full design)\n\n"
                                     "Late error = max position error over the last 5 s, mean ± std over seeds 0-4.\n\n"
                                     + markdown_table(crit) + "\n")
    ablation(st)
    omega_sweep()
    print(crit.to_string(index=False))
    return crit


if __name__ == "__main__":
    main()
