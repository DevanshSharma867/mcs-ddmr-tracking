"""Phase 7: curated report figures F1-F7 and the key-number table, from existing results.

Reads results/{reproduction,robustness,drift,phase6}/ (run those experiments first; run_all does) and
re-runs two small diagnostic sets itself: CSMDC at Ts = 1 ms and the CSMDC boundary-layer sweep.
Writes to the tracked results/report/: F1..F7 (png + pdf) and key_numbers.csv
(name, value, unit, source). Every number quoted in docs/report/ comes from key_numbers.csv.
Run: python -m experiments.report_figures
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from ddmr.config import build_controller, deep_merge, load_config, simulate_from_config
from ddmr.metrics import input_metrics, tracking_metrics
from ddmr.paper_tables import FIG6_STEADY_APPROX, TABLE2, TABLE4
from ddmr.plotting import COLORS, INPUT_LABEL, save_fig, setup_style

OUT = Path("results/report")
REP = Path("results/reproduction")
PAPER_CTRL = {"ckc_pi": ("CKC+PI", "ckc"), "rsmkc_pi": ("RSMKC+PI", "rsmkc"), "csmdc": ("CSMDC", "csmdc"),
              "afntsmdc": ("AFNTSMDC", "afntsmdc")}
ARMS = {"afntsmdc": "A0 reproduced", "afn_A1": "A1 +ESO", "afn_A2": "A2 +PS", "afn_A3": "A3 full",
        "afn_A4": "A4 ESO, no sw."}
KEY: list[dict] = []


def key(name: str, value: float, unit: str, source: str) -> float:
    KEY.append({"name": name, "value": value, "unit": unit, "source": source})
    return value


def log(plant: str, scen: str, cid: str) -> pd.DataFrame:
    return pd.read_csv(REP / plant / scen / f"{cid}.csv.gz")


def color(cid: str) -> str:
    return COLORS.get(cid, COLORS.get(cid.split("_")[0], "k"))


# ----------------------------------------------------------------------------- F1
def f1(summ: pd.DataFrame) -> None:
    src = "results/reproduction/summary.csv (plant ideal, S1)"
    s = summ[(summ.plant == "ideal") & (summ.scenario == "S1")].set_index("controller")
    for cid, (lab, pk) in PAPER_CTRL.items():
        for e in ("x_e", "y_e", "phi_e"):
            sc = 1.0 if e == "phi_e" else 1.0
            key(f"S1_{cid}_{e}_rms", float(s.loc[cid, f"{e}_rms"]) * (1 if e == "phi_e" else 1), "cm" if e != "phi_e" else "rad", src)
            key(f"S1_{cid}_{e}_settle", float(s.loc[cid, f"{e}_settle"]), "s", src)
            key(f"paper_T2_{pk}_{e}_rms", TABLE2[pk][e][0], "cm" if e != "phi_e" else "rad", "src/ddmr/paper_tables.py TABLE2")
            key(f"paper_T2_{pk}_{e}_settle", TABLE2[pk][e][2], "s", "src/ddmr/paper_tables.py TABLE2")
    fig, ax = plt.subplot_mosaic([["x", "bs"], ["y", "bs"], ["p", "br"]], figsize=(11, 7))
    for cid, (lab, _) in PAPER_CTRL.items():
        df = log("ideal", "S1", cid)
        for k, e in zip("xyp", ("x_e", "y_e", "phi_e")):
            ax[k].plot(df["t"], df[e], color=color(cid), lw=1, label=lab)
    for k, yl in zip("xyp", ("x_e (m)", "y_e (m)", "φ_e (rad)")):
        ax[k].set_ylabel(yl)
    ax["p"].set_xlabel("Time (s)")
    ax["x"].legend(fontsize=7)
    ax["x"].set_title("(a) our S1 tracking errors, ideal plant")
    x = np.arange(len(PAPER_CTRL))
    for k, col, title in (("bs", "x_e_settle", "(b) x_e settling time [s]"), ("br", "x_e_rms", "(c) x_e RMS [cm]")):
        ours = [s.loc[c, col] for c in PAPER_CTRL]
        paper = [TABLE2[pk]["x_e"][2 if col.endswith("settle") else 0] for _, pk in PAPER_CTRL.values()]
        ax[k].bar(x - 0.2, ours, 0.4, label="ours (sim, ideal plant)", color="tab:blue")
        ax[k].bar(x + 0.2, paper, 0.4, label="paper Table 2 (hardware)", color="0.6")
        ax[k].set_xticks(x, [v[0] for v in PAPER_CTRL.values()], fontsize=8)
        ax[k].set_title(title)
    ax["bs"].legend(fontsize=7)
    fig.suptitle("F1: reproduction of S1 vs the paper's headline numbers (paper values: Table 2, cited)", fontsize=10)
    save_fig(fig, OUT / "F1_reproduction")


# ----------------------------------------------------------------------------- F2
def f2(summ: pd.DataFrame) -> None:
    pr, co = log("ideal", "S1", "afntsmdc_printed"), log("ideal", "S1", "afntsmdc")
    src = "results/reproduction/ideal/S1/{afntsmdc_printed,afntsmdc}.csv.gz"
    late = lambda d: float(np.hypot(d.x_e, d.y_e)[d.t >= 15].max() * 100)  # noqa: E731
    key("printed_S1_late_pos_err", late(pr), "cm", src)
    key("corrected_S1_late_pos_err", late(co), "cm", src)
    key("printed_S1_c4_final", float(pr.c4.iloc[-1]), "-", src)
    key("corrected_S1_c1_final", float(co.c1.iloc[-1]), "-", src)
    rho = 0.8
    xe, ye = co.x_e.to_numpy(), co.y_e.to_numpy()
    a = np.abs(xe) + rho
    h21 = -rho * np.sign(xe) * np.arctan(ye) / a**2
    h22 = rho / (a * (1 + ye**2))
    hg22 = h21 * ye - h22 * xe - 1.0
    key("corrected_S1_HG22_max_after_1s", float(hg22[co.t >= 1].max()), "-", src + " (computed)")
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for d, lab, col in ((pr, "as printed: −B⁻¹M0·L·Ĉ", "tab:pink"), (co, "sign-corrected: +B⁻¹M0·L·Ĉ", "tab:blue")):
        for i, ls in zip(range(1, 5), ("-", "--", ":", "-.")):
            axes[0].plot(d.t, d[f"c{i}"], color=col, ls=ls, lw=1, label=f"ĉ{i} ({lab.split(':')[0]})")
        axes[1].semilogy(d.t, np.hypot(d.x_e, d.y_e), color=col, lw=1, label=lab)
    axes[2].plot(co.t, -np.ones_like(xe), color="k", lw=1, label="(HG)₁₁ = −1 (all states)")
    axes[2].plot(co.t, hg22, color="tab:blue", lw=1, label="(HG)₂₂ along the corrected run")
    axes[2].set(ylim=(-1.6, 0.2), ylabel="HG diagonal")
    axes[0].set_ylabel("adaptive gains ĉᵢ")
    axes[1].set_ylabel("position error (m)")
    for ax in axes:
        ax.legend(fontsize=6, ncol=2)
    axes[0].set_title("F2: printed vs sign-corrected switching term (S1, ideal plant). HG ≈ −I ⇒ printed −HG·L·Ĉ ≈ +sgn(s)·Ĉ",
                      fontsize=9)
    axes[-1].set_xlabel("Time (s)")
    save_fig(fig, OUT / "F2_sign_error")


# ----------------------------------------------------------------------------- F3
def f3(summ: pd.DataFrame) -> None:
    cfg = load_config("configs/robot_paper.yaml")
    gains = load_config("configs/controllers_paper.yaml")
    rows = []
    for rho_bl in (None, 0.02, 0.04, 0.07, 0.105, 0.15, 0.3):
        g = deep_merge(gains, {"csmdc": {"diagnostic_boundary_layer": rho_bl}}) if rho_bl else gains
        df = simulate_from_config(cfg, build_controller("csmdc_bl" if rho_bl else "csmdc", g, cfg))
        m = tracking_metrics(df)
        rows.append({"rho": rho_bl, "kts_rho": np.nan if rho_bl is None else 21 * 0.01 / rho_bl,
                     "chattering": input_metrics(df, 0.01, 8.0)["chattering"], "y_e_settle": m["y_e"]["settle"],
                     "x_e_final_cm": float(df.x_e.iloc[-1] * 100)})
    sweep = pd.DataFrame(rows)
    sweep.to_csv(OUT / "F3_rho_sweep.csv", index=False)
    srcs = "results/report/F3_rho_sweep.csv"
    for r in rows:
        tag = "sgn" if r["rho"] is None else f"{r['rho']}"
        key(f"csmdc_rho_{tag}_chattering", r["chattering"], "1/s", srcs)
        key(f"csmdc_rho_{tag}_y_e_settle", r["y_e_settle"], "s", srcs)
        key(f"csmdc_rho_{tag}_x_e_final", r["x_e_final_cm"], "cm", srcs)
    cfg1 = load_config("configs/robot_paper.yaml")
    cfg1["sim"]["Ts"] = 0.001
    d1 = simulate_from_config(cfg1, build_controller("csmdc", gains, cfg1))
    m1 = tracking_metrics(d1)
    for e in ("x_e", "y_e", "phi_e"):
        key(f"csmdc_Ts1ms_{e}_settle", m1[e]["settle"], "s", "experiments/report_figures.py (CSMDC, Ts = 1 ms, S1)")
    cs = log("ideal", "S1", "csmdc")
    late = cs[cs.t >= 10]
    key("csmdc_S1_s1_late_mean", float(late.s1.mean()), "-", "results/reproduction/ideal/S1/csmdc.csv.gz")
    key("csmdc_S1_s2_late_mean", float(late.s2.mean()), "-", "results/reproduction/ideal/S1/csmdc.csv.gz")
    key("csmdc_S1_s1_late_ptp", float(np.ptp(late.s1)), "-", "results/reproduction/ideal/S1/csmdc.csv.gz")
    key("csmdc_S1_x_e_final", float(cs.x_e.iloc[-1] * 100), "cm", "results/reproduction/ideal/S1/csmdc.csv.gz")
    af = log("ideal", "S1", "afntsmdc")
    fig, ax = plt.subplot_mosaic([["s", "s"], ["sw", "pl"]], figsize=(11, 7))
    w = (cs.t >= 10) & (cs.t <= 10.3)
    ax["s"].step(cs.t[w], cs.s1[w], where="post", color=color("csmdc"), lw=1, label="CSMDC s₁ (pure sgn)")
    ax["s"].step(af.t[w], af.s1[w], where="post", color=color("afntsmdc"), lw=1, label="AFNTSMDC s₁ (ϱ = 0.04)")
    ax["s"].axhline(0, color="k", lw=0.5)
    ax["s"].set(xlabel="Time (s)", ylabel="s₁", title="(a) sampled sliding variable: period-2 cycle of width ≈ K·Ts, off-centre mean")
    ax["s"].legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    sw = sweep.dropna(subset=["kts_rho"])
    ax["sw"].loglog(sw.kts_rho, sw.chattering, "o-", color=color("csmdc"))
    ax["sw"].axhline(sweep.chattering.iloc[0], color="k", ls="--", lw=0.8, label="pure sgn")
    ax["sw"].axvline(2, color="tab:red", lw=0.8, label="K·Ts/ϱ = 2 (stability)")
    ax["sw"].axvline(1, color="tab:green", lw=0.8, label="K·Ts/ϱ = 1 (monotone)")
    ax["sw"].set(xlabel="k_c1·Ts/ϱ", ylabel="chattering index", title="(b) CSMDC boundary-layer sweep (S1)")
    ax["sw"].legend(fontsize=7)
    s1 = summ[summ.scenario == "S1"]
    plants = ["ideal", "lag10ms", "lag20ms", "lag40ms", "motor12V", "motor18V"]
    for cid, lab in (("csmdc", "CSMDC"), ("afntsmdc", "AFNTSMDC ϱ = 0.04"), ("afntsmdc_bl015", "AFNTSMDC ϱ = 0.15")):
        v = s1[s1.controller == cid].set_index("plant").loc[plants, "chattering"]
        ax["pl"].semilogy(range(len(plants)), v.to_numpy() + 1e-1, "o-", label=lab, color=color(cid))
        for p in plants:
            key(f"chattering_S1_{cid}_{p}", float(v.loc[p]), "1/s", "results/reproduction/summary.csv")
    ax["pl"].set_xticks(range(len(plants)), plants, rotation=25, fontsize=8)
    ax["pl"].set(ylabel="chattering index", title="(c) chattering vs plant (S1)")
    ax["pl"].legend(fontsize=7)
    fig.suptitle("F3: discrete-time quasi-sliding at Ts = 0.01 s", fontsize=10)
    save_fig(fig, OUT / "F3_chattering")


# ----------------------------------------------------------------------------- F4
def f4(summ: pd.DataFrame) -> None:
    plants = ["ideal", "motor12V", "motor18V"]
    vals = {}
    for p in plants:
        for cid in PAPER_CTRL:
            s2, s3 = log(p, "S2", cid), log(p, "S3", cid)
            wnd = (s3.t >= 12) & (s3.t <= 20)
            vals[p, cid] = key(f"shock_xe_{p}_{cid}", float((s3.x_e - s2.x_e)[wnd].abs().max() * 100), "cm",
                               f"results/reproduction/{p}/S2,S3/{cid}.csv.gz (max |x_e,S3 - x_e,S2|, 12-20 s)")
    for cid, (_, pk) in PAPER_CTRL.items():
        key(f"paper_T4_{pk}_x_e_max", TABLE4[pk]["x_e"][1], "cm", "src/ddmr/paper_tables.py TABLE4")
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(PAPER_CTRL))
    for i, p in enumerate(plants):
        a.bar(x + (i - 1.5) * 0.2, [vals[p, c] for c in PAPER_CTRL], 0.2, label=f"ours: {p}")
    a.bar(x + 1.5 * 0.2, [TABLE4[pk]["x_e"][1] for _, pk in PAPER_CTRL.values()], 0.2, color="0.6",
          label="paper Table 4 x_e MAX (hardware)")
    a.set(yscale="log", ylabel="shock-induced x_e [cm]", title="(a) S3 shock effect")
    a.set_xticks(x, [v[0] for v in PAPER_CTRL.values()])
    a.legend(fontsize=7)
    rows = []
    for p in plants:
        d = log(p, "S3", "afntsmdc")
        pre = d[(d.t >= 11) & (d.t < 12)]
        rows.append((p, key(f"steady_uR_{p}", float(pre.ua_R.mean()), "model units / V", f"results/reproduction/{p}/S3/afntsmdc.csv.gz"),
                     key(f"steady_uL_{p}", float(pre.ua_L.mean()), "model units / V", f"results/reproduction/{p}/S3/afntsmdc.csv.gz")))
    key("paper_fig6_steady_right_approx", FIG6_STEADY_APPROX["right"], "V", "src/ddmr/paper_tables.py FIG6_STEADY_APPROX (read from figure)")
    key("paper_fig6_steady_left_approx", FIG6_STEADY_APPROX["left"], "V", "src/ddmr/paper_tables.py FIG6_STEADY_APPROX (read from figure)")
    labels = [r[0] for r in rows] + ["paper Fig. 6 (≈, |·|)"]
    b.bar(np.arange(4) - 0.2, [abs(r[1]) for r in rows] + [abs(FIG6_STEADY_APPROX["right"])], 0.4, label="right / outer wheel")
    b.bar(np.arange(4) + 0.2, [abs(r[2]) for r in rows] + [abs(FIG6_STEADY_APPROX["left"])], 0.4, label="left / inner wheel")
    b.set_xticks(range(4), labels, fontsize=8)
    b.set(ylabel=f"|steady input| ({INPUT_LABEL})", title="(b) steady wheel input before the shock, AFNTSMDC")
    b.legend(fontsize=7)
    for cid in PAPER_CTRL:
        for p in ("ideal", "motor12V", "motor18V"):
            r = summ[(summ.plant == p) & (summ.scenario == "S1") & (summ.controller == cid)].iloc[0]
            key(f"S1_{p}_{cid}_x_e_settle", float(r.x_e_settle), "s", "results/reproduction/summary.csv")
            key(f"S1_{p}_{cid}_x_e_rms", float(r.x_e_rms), "cm", "results/reproduction/summary.csv")
    for p in ("ideal", "motor12V"):
        r2 = summ[(summ.plant == p) & (summ.scenario == "S2") & (summ.controller == "afntsmdc")].iloc[0]
        key(f"S2_{p}_afntsmdc_x_e_rms", float(r2.x_e_rms), "cm", "results/reproduction/summary.csv")
    fig.suptitle("F4: torque-input plant (a) vs voltage-input DC-motor plants (c) vs the paper", fontsize=10)
    save_fig(fig, OUT / "F4_plants")


# ----------------------------------------------------------------------------- F5
def f5() -> None:
    runs = pd.read_csv("results/robustness/runs.csv")
    ctrls = {"ckc_pi": "CKC+PI", "rsmkc_pi": "RSMKC+PI", "csmdc": "CSMDC", "afntsmdc": "AFNTSMDC",
             "afntsmdc_bl015": "AFNTSMDC ϱ=0.15"}
    d = runs[(runs.plant == "torque") & (runs.trajectory == "circle") & runs.controller.isin(ctrls)]
    piv = d.groupby(["condition", "controller"], sort=False)["pos_late_max_cm"].mean().unstack()[list(ctrls)]
    order = list(load_config("configs/robustness.yaml")["conditions"])
    piv = piv.reindex([c for c in order if c in piv.index])
    for cond in piv.index:
        for cid in ctrls:
            key(f"grid_torque_circle_{cond}_{cid}_late", float(piv.loc[cond, cid]), "cm", "results/robustness/runs.csv")
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 6.5), gridspec_kw={"width_ratios": [1.2, 1]})
    im = a.imshow(np.log10(piv.to_numpy()), aspect="auto", cmap="viridis")
    for (i, j), v in np.ndenumerate(piv.to_numpy()):
        a.text(j, i, f"{v:.2g}", ha="center", va="center", fontsize=6, color="w")
    a.set_xticks(range(len(ctrls)), list(ctrls.values()), rotation=25, ha="right", fontsize=7)
    a.set_yticks(range(len(piv)), piv.index, fontsize=7)
    a.grid(False)
    fig.colorbar(im, ax=a, label="log10 late position error [cm]")
    a.set_title("(a) robustness grid, torque plant, circle\nlate position error (last 5 s), mean of 5 seeds", fontsize=9)
    rates = pd.read_csv("results/drift/drift_rates.csv")
    for i, plant in enumerate(["ideal", "motor12V", "motor18V"]):
        logs = [pd.read_csv(f"results/drift/afntsmdc_S1_{plant}_{s}.csv.gz") for s in range(5)]
        t = logs[0].t
        for c, ls in (("c1", "-"), ("c4", "--")):
            b.plot(t, np.mean([l[c] for l in logs], axis=0), color=plt.cm.tab10(i), ls=ls, lw=1, label=f"{plant} ĉ{c[1]}")
        r = rates[(rates.controller == "afntsmdc") & (rates.scenario == "S1") & (rates.plant == plant) & (rates.seed != "clean")]
        for c in ("c1", "c2", "c3", "c4"):
            key(f"drift_A0_S1_{plant}_{c}_rate", float(r[f"{c}_rate"].astype(float).mean()), "1/s", "results/drift/drift_rates.csv")
            key(f"drift_A0_S1_{plant}_{c}_1h", float(r[f"{c}_1h"].astype(float).mean()), "-", "results/drift/drift_rates.csv")
    b.set(xlabel="Time (s)", ylabel="adaptive gain", title="(b) A0 adaptive gains over 200 s, S1, moderate noise\n(mean of 5 seeds)")
    b.legend(fontsize=7, ncol=2)
    fig.suptitle("F5: robustness weaknesses of the reproduced AFNTSMDC", fontsize=10)
    save_fig(fig, OUT / "F5_robustness")


# ----------------------------------------------------------------------------- F6
def f6() -> None:
    runs = pd.read_csv("results/robustness/runs.csv")
    conds = ["nominal", "noise_x2", "mass_x0.7", "dist_const", "dist_yaw", "dist_sin1Hz", "dist_sin0.2Hz",
             "dist_bandlim", "friction_10", "friction_30", "lag_40ms"]
    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(3, 1, height_ratios=[1, 1, 1.4])
    a, b, c = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
    d = runs[(runs.plant == "torque") & (runs.trajectory == "circle")]
    g = d.groupby(["condition", "controller"])[["pos_late_max_cm", "chattering"]].mean()
    x = np.arange(len(conds))
    wdt = 0.8 / len(ARMS)
    for k, (cid, lab) in enumerate(ARMS.items()):
        late = [g.loc[(cn, cid), "pos_late_max_cm"] for cn in conds]
        chat = [g.loc[(cn, cid), "chattering"] for cn in conds]
        a.bar(x + k * wdt, late, wdt, label=lab, color=COLORS.get(cid, "tab:blue"))
        b.bar(x + k * wdt, chat, wdt, color=COLORS.get(cid, "tab:blue"))
        for cn, lv, cv in zip(conds, late, chat):
            key(f"ablation_torque_circle_{cn}_{cid}_late", float(lv), "cm", "results/robustness/runs.csv")
            key(f"ablation_torque_circle_{cn}_{cid}_chattering", float(cv), "1/s", "results/robustness/runs.csv")
    a.set(yscale="log", ylabel="late position error [cm]", title="(a) ablation, torque plant, circle (mean of 5 seeds)")
    b.set(yscale="log", ylabel="chattering index")
    for ax in (a, b):
        ax.set_xticks(x + 0.4 - wdt / 2, conds, rotation=20, ha="right", fontsize=8)
    a.legend(fontsize=7, ncol=5)
    crit = pd.read_csv("results/phase6/criteria.csv")
    c.axis("off")
    tab = c.table(cellText=crit[["criterion", "item", "A3", "threshold / reference", "pass"]].to_numpy(),
                  colLabels=["", "item", "A3", "threshold / reference", "result"], loc="upper center",
                  cellLoc="left", colWidths=[0.08, 0.40, 0.22, 0.22, 0.06])
    tab.auto_set_font_size(False)
    tab.set_fontsize(5.5)
    tab.scale(1, 0.9)
    c.text(0.5, 1.02, "(c) pre-registered criteria P1–P7 (results/phase6/criteria.md)", ha="center",
           transform=c.transAxes, fontsize=9)
    n_pass = int((crit["pass"] == "PASS").sum())
    key("prereg_criteria_pass", n_pass, "count", "results/phase6/criteria.csv")
    key("prereg_criteria_total", int(crit["pass"].isin(["PASS", "FAIL"]).sum()), "count", "results/phase6/criteria.csv")
    for _, r in crit.iterrows():
        if r["pass"] in ("PASS", "FAIL"):
            key(f"crit_{r['criterion']}_{r['item']}", str(r["A3"]), "", "results/phase6/criteria.csv")
    fig.suptitle("F6: pre-registered improvement — ablation A0–A4 and criteria", fontsize=10)
    save_fig(fig, OUT / "F6_ablation")


# ----------------------------------------------------------------------------- F7
def f7() -> None:
    lag = pd.read_csv("results/phase6/diagnostics_lag.csv")
    m18 = pd.read_csv("results/phase6/diagnostics_motor18V.csv")
    for _, r in lag.iterrows():
        key(f"diag_lag_{r['setting']}_{r['controller']}", str(r["late err [cm] mean ± std"]), "cm",
            "results/phase6/diagnostics_lag.csv")
    for _, r in m18[m18.scenario == "S1"].iterrows():
        key(f"diag_motor18V_S1_{r['label']}_late", float(r["late_pos_err_max_cm"]), "cm", "results/phase6/diagnostics_motor18V.csv")
        key(f"diag_motor18V_S1_{r['label']}_saturated", float(r["saturated_pct"]), "%", "results/phase6/diagnostics_motor18V.csv")
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.5))
    g = lag[lag.setting.str.startswith("grid")]
    mean = g["late err [cm] mean ± std"].str.split(" ± ").str[0].astype(float)
    labels = g.setting.str.replace("grid ", "") + "\n" + g.controller.str.replace(" (diagnostic)", "")
    a.bar(range(len(g)), mean, color=["tab:green" if "aware" not in c else "tab:cyan" for c in g.controller])
    a.set_xticks(range(len(g)), labels, fontsize=6, rotation=20, ha="right")
    a.set(yscale="log", ylabel="late position error [cm]", title="(a) A3 vs A3 with lag-aware ESO (exploratory diagnostic)")
    s1 = m18[m18.scenario == "S1"]
    a2 = b.bar(range(len(s1)), s1.late_pos_err_max_cm, color=["tab:red" if p > 50 else "tab:blue" for p in s1.saturated_pct])
    for rect, p in zip(a2, s1.saturated_pct):
        b.text(rect.get_x() + rect.get_width() / 2, rect.get_height() * 1.1, f"{p:.0f}% sat", ha="center", fontsize=6)
    b.set_xticks(range(len(s1)), s1.label, fontsize=6, rotation=25, ha="right")
    b.set(yscale="log", ylabel="late position error [cm]", title="(b) motor18V, S1: all controllers (red: saturated >50%)")
    fig.suptitle("F7: limitations of the improved design (exploratory diagnostics)", fontsize=10)
    save_fig(fig, OUT / "F7_limitations")


def main() -> pd.DataFrame:
    OUT.mkdir(parents=True, exist_ok=True)
    setup_style()
    KEY.clear()
    summ = pd.read_csv(REP / "summary.csv")
    f1(summ)
    f2(summ)
    f3(summ)
    f4(summ)
    f5()
    f6()
    f7()
    k = pd.DataFrame(KEY)
    k.to_csv(OUT / "key_numbers.csv", index=False)
    print(f"{len(k)} key numbers -> {OUT / 'key_numbers.csv'}")
    return k


if __name__ == "__main__":
    main()
