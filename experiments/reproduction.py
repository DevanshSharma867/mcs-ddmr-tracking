"""Phase 4: AFNTSMDC reproduction. Every controller x scenario x plant from configs/reproduction.yaml.

Writes to results/reproduction/:
  <plant>/<scenario>/<controller>.csv.gz       per-tick logs (incl. s, c1..c4, switching term)
  <plant>/fig5_S1, fig6_S1, fig7_S2, fig8_S2, fig9_S3, fig10_S3   paper figures (paper controllers)
  <plant>/table2.md, table3.md, table4.md (+ .csv)             ours vs the paper's tables
  adaptive_gains_<scenario>   c1..c4 and s1, s2 of the reproduced AFNTSMDC, one line per plant
  adaptive_gains_printed      as-printed law vs sign-corrected (S1): the c ramp
  chattering_vs_lag           chattering index of every controller vs actuator lag (S1-S3)
  summary.csv                 metrics of all runs; variants.md: boundary-layer / switching attribution
Run: python -m experiments.reproduction --config configs/reproduction.yaml
"""

from __future__ import annotations

import argparse
import copy
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ddmr.config import build_controller, deep_merge, load_config, simulate_from_config
from ddmr.metrics import BANDS, input_metrics, markdown_table, settling_time, tracking_metrics
from ddmr.paper_tables import TABLE2, TABLE3, TABLE4
from ddmr.plotting import COLORS, INPUT_LABEL, save_fig, setup_style
from ddmr.sim import save_log

OUT = Path("results/reproduction")
ERRS = ("x_e", "y_e", "phi_e")
SCALE = {"x_e": 100.0, "y_e": 100.0, "phi_e": 1.0}        # paper units: cm, cm, rad
YLAB = {"x_e": "X-axis error (m)", "y_e": "Y-axis error (m)", "phi_e": "Orientation angle error (rad)"}
PAPER_KEY = {"ckc_pi": "ckc", "rsmkc_pi": "rsmkc", "csmdc": "csmdc", "afntsmdc": "afntsmdc"}
SHOCK = (12.0, 12.5)


def color(cid: str) -> str:
    return COLORS.get(cid, COLORS.get(cid.split("_")[0], "k"))


def run_all(exp: dict) -> dict[tuple[str, str, str], pd.DataFrame]:
    """Simulate every (plant, scenario, controller); returns logs keyed by those ids."""
    base_gains = load_config(exp["gains"])
    runs = {}
    for plant, tau in exp["plants"].items():
        for scen, path in exp["scenarios"].items():
            cfg = load_config(path)
            cfg["plant"]["actuator_tau"] = tau
            for cid, spec in exp["controllers"].items():
                gains = deep_merge(base_gains, spec.get("gains", {}))
                df = simulate_from_config(cfg, build_controller(spec["kind"], gains, cfg))
                save_log(df, OUT / plant / scen / f"{cid}.csv.gz")
                runs[plant, scen, cid] = df
    return runs


def window_metrics(df: pd.DataFrame, t0: float, t1: float) -> dict[str, dict[str, float]]:
    """RMS/MAX on [t0, t1]; absolute settling time on that window (t0 if never outside the band)."""
    w = df[(df["t"] >= t0 - 1e-9) & (df["t"] <= t1 + 1e-9)]
    t = w["t"].to_numpy()
    out = {}
    for e in ERRS:
        x = w[e].to_numpy()
        st = settling_time(t, x, BANDS[e])
        out[e] = {"rms": float(np.sqrt(np.mean(x**2))), "max": float(np.max(np.abs(x))),
                  "settle_abs": t0 if st == 0.0 else st}
    return out


def summary(runs: dict, exp: dict, Ts: float, u_max: float) -> pd.DataFrame:
    rows = []
    for (plant, scen, cid), df in runs.items():
        tm, im = tracking_metrics(df), input_metrics(df, Ts, u_max)
        row = {"plant": plant, "scenario": scen, "controller": cid,
               "label": exp["controllers"][cid]["label"], **im}
        for e in ERRS:
            row |= {f"{e}_rms": tm[e]["rms"] * SCALE[e], f"{e}_max": tm[e]["max"] * SCALE[e],
                    f"{e}_settle": tm[e]["settle"]}
        if scen == "S3":
            wm = window_metrics(df, SHOCK[0], 20.0)
            for e in ERRS:
                row |= {f"{e}_rms_12_20": wm[e]["rms"] * SCALE[e], f"{e}_max_12_20": wm[e]["max"] * SCALE[e],
                        f"{e}_settle_from12": wm[e]["settle_abs"] - SHOCK[0],
                        f"{e}_settle_from12.5": wm[e]["settle_abs"] - SHOCK[1]}
        if "c1" in df:
            row |= {f"c{i}_final": df[f"c{i}"].iloc[-1] for i in range(1, 5)}
        late = df[df["t"] >= 15.0]
        row["late_pos_err_max_cm"] = float(np.hypot(late["x_e"], late["y_e"]).max() * 100)
        rows.append(row)
    return pd.DataFrame(rows)


def paper_table(summ: pd.DataFrame, plant: str, scen: str, paper: dict, exp: dict) -> pd.DataFrame:
    rows = []
    for cid in exp["paper_controllers"]:
        r = summ[(summ.plant == plant) & (summ.scenario == scen) & (summ.controller == cid)].iloc[0]
        for e in ERRS:
            p = paper[PAPER_KEY[cid]][e]
            row = {"controller": exp["controllers"][cid]["label"], "error": e,
                   "RMS": r[f"{e}_rms"], "MAX": r[f"{e}_max"], "settle_s": r[f"{e}_settle"]}
            if scen == "S3":
                row |= {"RMS_12_20": r[f"{e}_rms_12_20"], "MAX_12_20": r[f"{e}_max_12_20"],
                        "settle_from12": r[f"{e}_settle_from12"],
                        "settle_from12.5": r[f"{e}_settle_from12.5"]}
            row |= {"paper_RMS": p[0], "paper_MAX": p[1], "paper_settle_s": p[2]}
            rows.append(row)
    return pd.DataFrame(rows)


def fig_tracking(runs: dict, plant: str, scen: str, exp: dict, stem: Path, title: str) -> None:
    """Paper Fig. 5/7 layout: (a) paths, (b) heading with 0-1.5 s inset, (c)-(e) errors."""
    fig, ax = plt.subplot_mosaic([["a", "c"], ["a", "d"], ["b", "e"]], figsize=(10, 8))
    ref = runs[plant, scen, exp["paper_controllers"][0]]
    for cid in exp["paper_controllers"]:
        df, lab, col = runs[plant, scen, cid], exp["controllers"][cid]["label"], color(cid)
        ax["a"].plot(df["x"], df["y"], color=col, lw=1, label=lab)
        ax["b"].plot(df["t"], df["phi"], color=col, lw=1, label=lab)
        for k, e in zip("cde", ERRS):
            ax[k].plot(df["t"], df[e], color=col, lw=1, label=lab)
    ax["a"].plot(ref["x_r"], ref["y_r"], "k--", lw=1, label="Reference")
    ax["b"].plot(ref["t"], ref["phi_r"], "k--", lw=1, label="Reference")
    ax["a"].set(xlabel="X-axis (m)", ylabel="Y-axis (m)", aspect="equal", title="(a)")
    ax["a"].legend(fontsize=7, loc="center")
    ax["b"].set(xlabel="Time (s)", ylabel="Orientation angle (rad)", title="(b)")
    ins = ax["b"].inset_axes([0.08, 0.5, 0.35, 0.4])
    for cid in exp["paper_controllers"]:
        df = runs[plant, scen, cid]
        m = df["t"] <= 1.5
        ins.plot(df["t"][m], df["phi"][m], color=color(cid), lw=1)
    ins.plot(ref["t"][ref["t"] <= 1.5], ref["phi_r"][ref["t"] <= 1.5], "k--", lw=1)
    ins.tick_params(labelsize=6)
    for k, e in zip("cde", ERRS):
        ax[k].set(ylabel=YLAB[e], title=f"({k})")
    ax["e"].set_xlabel("Time (s)")
    ax["c"].legend(fontsize=7)
    fig.suptitle(title, fontsize=10)
    save_fig(fig, stem)


def fig_inputs(runs: dict, plant: str, scen: str, exp: dict, stem: Path, title: str,
               window: tuple[float, float] | None = None) -> None:
    """Paper Fig. 6/8/10: applied wheel inputs, one row per controller."""
    cids = exp["paper_controllers"]
    fig, axes = plt.subplots(len(cids), 1, figsize=(8, 2.0 * len(cids)), sharex=True)
    for ax, cid in zip(axes, cids):
        df = runs[plant, scen, cid]
        if window:
            df = df[(df["t"] >= window[0]) & (df["t"] <= window[1])]
        ax.plot(df["t"], df["ua_R"], color=color(cid), lw=0.6, label="right")
        ax.plot(df["t"], df["ua_L"], color=color(cid), lw=0.6, ls="--", alpha=0.7, label="left")
        ax.set_ylabel(exp["controllers"][cid]["label"].replace(" (", "\n("), fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
    axes[0].set_title(f"{title}: {INPUT_LABEL}", fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, stem)


def fig_shock_errors(runs: dict, plant: str, exp: dict, stem: Path, title: str) -> None:
    """Paper Fig. 9: errors on 11-18 s around the shock (S3)."""
    fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    for cid in exp["paper_controllers"]:
        df = runs[plant, "S3", cid]
        df = df[(df["t"] >= 11) & (df["t"] <= 18)]
        for ax, e in zip(axes, ERRS):
            ax.plot(df["t"], df[e], color=color(cid), lw=1, label=exp["controllers"][cid]["label"])
    for ax, e in zip(axes, ERRS):
        ax.axvspan(*SHOCK, color="0.85", zorder=0)
        ax.set_ylabel(YLAB[e])
    axes[0].legend(fontsize=7)
    axes[0].set_title(title, fontsize=10)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, stem)


def fig_adaptive(runs: dict, exp: dict, scen: str, stem: Path) -> None:
    names = ["c1", "c2", "c3", "c4", "s1", "s2"]
    fig, axes = plt.subplots(len(names), 1, figsize=(7, 10), sharex=True)
    for i, plant in enumerate(exp["plants"]):
        df = runs[plant, scen, "afntsmdc"]
        for ax, n in zip(axes, names):
            ax.plot(df["t"], df[n], lw=0.8, color=plt.cm.viridis(i / max(1, len(exp["plants"]) - 1)),
                    label=plant)
    for ax, n in zip(axes, names):
        ax.set_ylabel(n.replace("c", "ĉ") if n.startswith("c") else n)
    for ax in axes[4:]:
        ax.axhspan(-0.05, 0.05, color="0.9", zorder=0)     # dead zone eps = 0.05
    axes[0].legend(fontsize=7, ncol=4)
    axes[0].set_title(f"{scen}: AFNTSMDC (sign-corrected) adaptive gains and sliding variables "
                      "(grey: dead zone)", fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, stem)


def fig_printed(runs: dict, stem: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    for cid, lab in [("afntsmdc_printed", "as printed (-B⁻¹M0LĈ)"), ("afntsmdc", "sign-corrected (+B⁻¹M0LĈ)")]:
        df = runs["ideal", "S1", cid]
        for i in range(1, 5):
            axes[0].plot(df["t"], df[f"c{i}"], color=color(cid), lw=1, ls=["-", "--", ":", "-."][i - 1],
                         label=f"ĉ{i} {lab}")
        axes[1].plot(df["t"], np.hypot(df["x_e"], df["y_e"]), color=color(cid), lw=1, label=lab)
        axes[2].plot(df["t"], df["s1"], color=color(cid), lw=0.8, label=f"s1 {lab}")
        axes[2].plot(df["t"], df["s2"], color=color(cid), lw=0.8, ls="--", label=f"s2 {lab}")
    axes[0].set_ylabel("adaptive gains ĉi")
    axes[1].set(ylabel="position error (m)", yscale="log")
    axes[2].set_ylabel("sliding variables")
    for ax in axes:
        ax.legend(fontsize=6, ncol=2)
    axes[0].set_title("S1, ideal plant: AFNTSMDC as printed vs sign-corrected", fontsize=10)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, stem)


def fig_chattering(summ: pd.DataFrame, exp: dict, stem: Path) -> None:
    taus = {p: (0.0 if t is None else t * 1e3) for p, t in exp["plants"].items()}
    fig, axes = plt.subplots(1, len(exp["scenarios"]), figsize=(13, 4), sharey=True)
    for ax, scen in zip(axes, exp["scenarios"]):
        for cid, spec in exp["controllers"].items():
            if cid == "afntsmdc_printed":
                continue                                   # diverges; not comparable
            d = summ[(summ.scenario == scen) & (summ.controller == cid)]
            ax.plot([taus[p] for p in d.plant], d.chattering, "o-", color=color(cid), lw=1,
                    ms=3, label=spec["label"])
        ax.set(title=scen, xlabel="actuator lag τ_a (ms; 0 = ideal)", yscale="symlog")
    axes[0].set_ylabel("chattering index (input TV per s)")
    axes[-1].legend(fontsize=6, loc="center left", bbox_to_anchor=(1, 0.5))
    save_fig(fig, stem)


def variants_table(summ: pd.DataFrame) -> pd.DataFrame:
    ids = ["csmdc", "csmdc_bl004", "csmdc_bl015", "afntsmdc", "afntsmdc_bl015", "afntsmdc_general",
           "afntsmdc_noswitch", "afntsmdc_printed"]
    cols = ["plant", "scenario", "label", "x_e_rms", "x_e_settle", "y_e_rms", "y_e_settle",
            "phi_e_rms", "phi_e_settle", "late_pos_err_max_cm", "chattering", "effort"]
    d = summ[summ.controller.isin(ids)].copy()
    d["order"] = d.controller.map({c: i for i, c in enumerate(ids)})
    return d.sort_values(["scenario", "plant", "order"])[cols]


def main(config: str = "configs/reproduction.yaml") -> pd.DataFrame:
    exp = load_config(config)
    t0 = time.time()
    runs = run_all(exp)
    print(f"{len(runs)} runs in {time.time() - t0:.0f} s")
    base = load_config(exp["scenarios"]["S1"])
    summ = summary(runs, exp, base["sim"]["Ts"], base["plant"]["u_max"])
    summ.to_csv(OUT / "summary.csv", index=False)

    setup_style()
    paper = {"S1": ("table2", TABLE2), "S2": ("table3", TABLE3), "S3": ("table4", TABLE4)}
    for plant in exp["plants"]:
        d = OUT / plant
        fig_tracking(runs, plant, "S1", exp, d / "fig5_S1", f"S1 initial offset, plant {plant} (cf. Fig. 5)")
        fig_inputs(runs, plant, "S1", exp, d / "fig6_S1", f"S1, plant {plant} (cf. Fig. 6)")
        fig_tracking(runs, plant, "S2", exp, d / "fig7_S2", f"S2 payload, plant {plant} (cf. Fig. 7)")
        fig_inputs(runs, plant, "S2", exp, d / "fig8_S2", f"S2, plant {plant} (cf. Fig. 8)")
        fig_shock_errors(runs, plant, exp, d / "fig9_S3", f"S3 payload + shock, plant {plant} (cf. Fig. 9)")
        fig_inputs(runs, plant, "S3", exp, d / "fig10_S3", f"S3, plant {plant} (cf. Fig. 10)", (11, 18))
        for scen, (name, ptab) in paper.items():
            tab = paper_table(summ, plant, scen, ptab, exp)
            tab.to_csv(d / f"{name}.csv", index=False)
            (d / f"{name}.md").write_text(
                f"# {name} ({scen}), plant {plant}: ours vs paper\n\n"
                "x_e, y_e in cm; phi_e in rad; settle in s (band 1 cm / 0.02 rad, NaN = not settled).\n"
                + ("S3: RMS/MAX/settle over 0-20 s; *_12_20 over 12-20 s; settle_from12 / "
                   "settle_from12.5 measured from shock start / end (negative = settled before 12.5 s).\n"
                   if scen == "S3" else "") + "\n" + markdown_table(tab) + "\n")
    for scen in exp["scenarios"]:
        fig_adaptive(runs, exp, scen, OUT / f"adaptive_gains_{scen}")
    fig_printed(runs, OUT / "adaptive_gains_printed")
    fig_chattering(summ, exp, OUT / "chattering_vs_lag")
    (OUT / "variants.md").write_text(
        "# Boundary-layer and switching-term attribution\n\n"
        "x_e, y_e RMS in cm, phi_e RMS in rad, settle in s (NaN = not settled), late_pos_err = max "
        "position error for t >= 15 s [cm], chattering = input TV per s, effort = sum ||u||^2 Ts.\n\n"
        + markdown_table(variants_table(summ)) + "\n")
    return summ


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="configs/reproduction.yaml")
    main(ap.parse_args().config)
