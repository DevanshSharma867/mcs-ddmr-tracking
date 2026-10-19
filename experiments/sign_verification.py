"""Adversarial verification of the AFNTSMDC switching-term sign (paper eq. 32).

1. Symbolic (sympy), from the paper's own definitions (14), (18)-(19), (22), (24):
   - p_e_dot = F + G z holds with the printed G (pins the sign of G);
   - H equals the Jacobian of xi (22) w.r.t. p_e (printed (24));
   - HG at the origin, and the switching contribution -HG L C_hat to s_dot;
   - the same under the reversed convention e = p - p_r;
   - det(HG M0^-1 B) vs printed (52).
2. Simulation: the printed law under every alternative convention/reading (S1 and S2, ideal plant,
   paper gains, no noise, plus S1 with 20 ms actuator lag).
Writes results/paper/sign_verification_symbolic.txt and results/paper/sign_verification_sim.{csv,md}.
Run: python -m experiments.sign_verification
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
import sympy as sp

from ddmr.config import build_controller, load_config, simulate_from_config
from ddmr.controllers.afntsm import AFNTSMDC
from ddmr.controllers.switching import abs_pow, sat, sig
from ddmr.controllers.xi_terms import xi_terms
from ddmr.errors import tracking_error, wrap_angle
from ddmr.metrics import markdown_table, tracking_metrics

OUT = Path("results/paper")


# ----------------------------------------------------------------------------- symbolic
def symbolic() -> str:
    t = sp.symbols("t", real=True)
    v, w, vr, wr, rho, r, b, m0, I0 = sp.symbols("v omega v_r omega_r rho r b m_0 I_0", real=True)
    x, y, ph = (sp.Function(n)(t) for n in ("x", "y", "phi"))
    xr, yr, phr = (sp.Function(n)(t) for n in ("x_r", "y_r", "phi_r"))
    kin = {x.diff(t): v * sp.cos(ph), y.diff(t): v * sp.sin(ph), ph.diff(t): w,
           xr.diff(t): vr * sp.cos(phr), yr.diff(t): vr * sp.sin(phr), phr.diff(t): wr}
    Rt = sp.Matrix([[sp.cos(ph), sp.sin(ph), 0], [-sp.sin(ph), sp.cos(ph), 0], [0, 0, 1]])
    pe = Rt * sp.Matrix([xr - x, yr - y, phr - ph])                                     # eq. (14)
    xe, ye, phe = pe
    F = sp.Matrix([vr * sp.cos(phe), vr * sp.sin(phe), wr])                              # eq. (19)
    G = sp.Matrix([[-1, ye], [0, -xe], [0, -1]])
    z = sp.Matrix([v, w])
    residual = sp.simplify(sp.expand_trig(pe.diff(t).subs(kin) - (F + G * z)))
    lines = [f"(18)-(19) residual p_e_dot - (F + G z) with printed G, derived from (14) and (4): {list(residual)}"]

    Xe, Ye, Phe = sp.symbols("x_e y_e phi_e", real=True)
    out = []
    for name, region in (("x_e > 0", sp.Symbol("x_e", positive=True)), ("x_e < 0", sp.Symbol("x_e", negative=True))):
        xi2 = Phe + rho / (sp.Abs(region) + rho) * sp.atan(Ye)
        jac = sp.Matrix([[1, 0, 0], [sp.diff(xi2, region), sp.diff(xi2, Ye), sp.diff(xi2, Phe)]])
        sg = sp.sign(region)
        H_printed = sp.Matrix([[1, 0, 0], [-rho * (sp.Abs(region) + rho) ** -2 * sg * sp.atan(Ye),
                                           rho / ((sp.Abs(region) + rho) * (1 + Ye**2)), 1]])
        out.append(f"H (24) == Jacobian of xi (22) for {name}: {sp.simplify(jac - H_printed) == sp.zeros(2, 3)}")
    lines += out

    sgx = sp.Symbol("sgn_xe")
    H = sp.Matrix([[1, 0, 0], [-rho * (sp.Abs(Xe) + rho) ** -2 * sgx * sp.atan(Ye),
                               rho / ((sp.Abs(Xe) + rho) * (1 + Ye**2)), 1]])
    Gs = sp.Matrix([[-1, Ye], [0, -Xe], [0, -1]])
    HG = sp.simplify(H * Gs)
    HG0 = HG.subs({Xe: 0, Ye: 0, sgx: 0})
    lines.append(f"HG (general) = {HG.tolist()}")
    lines.append(f"HG at origin (x_e = y_e = phi_e = 0) = {HG0.tolist()}  ->  -I: {HG0 == -sp.eye(2)}")
    L1, L2, C1, C2 = sp.symbols("L1 L2 Chat1 Chat2")
    term = sp.simplify(-HG0 * sp.Matrix([L1 * C1, L2 * C2]))
    lines.append(f"switching contribution to s_dot at origin, -HG L C_hat = {list(term)}  "
                 "(L_i = sgn(s_i), C_hat_i >= 0  =>  s_i * term_i = +|s_i| C_hat_i >= 0: drives s AWAY from 0)")
    Grev = -Gs                                                              # e = p - p_r  =>  e_dot = -F - G z
    lines.append(f"reversed convention e = p - p_r: G_rev = -G, H G_rev at origin = {(H * Grev).subs({Xe: 0, Ye: 0, sgx: 0}).tolist()} "
                 "(= +I: printed (32) would then be stabilising)")
    M0inv = sp.diag(1 / m0, 1 / I0)
    B = sp.Matrix([[1, 1], [b, -b]]) / r
    det = sp.simplify(sp.det(H * Gs * M0inv * B))
    lines.append(f"det(HG M0^-1 B) = {sp.factor(det)}   (printed (52) has prefactor -2b/(r m0 I0); derived has 1/r^2)")
    return "\n".join(lines)


# ----------------------------------------------------------------------------- simulation
class Reading(AFNTSMDC):
    """The printed AFNTSMDC law (31)-(39) evaluated under an alternative convention / reading."""

    def __init__(self, reading: str, **kw) -> None:
        self.reading = reading
        super().__init__(**kw)

    def compute(self, t, state, ref):
        v, w = state[3:5]
        z = state[3:5]
        p_e = tracking_error(state, ref.pose)
        if self.reading == "reversed_error_printed_formulas":      # e = p - p_r plugged into printed F, G, H
            xt = xi_terms(np.array([-p_e[0], -p_e[1], wrap_angle(-p_e[2])]), z, ref, self.rho, self.params)
            xi, xi_dot, drift, D = xt.xi, xt.xi_dot, xt.drift, xt.D
        else:
            xt = xi_terms(p_e, z, ref, self.rho, self.params)
            xi, xi_dot, drift, D = xt.xi, xt.xi_dot, xt.drift, xt.D
            if self.reading == "reversed_error_consistent":         # e = p - p_r with consistently re-derived F, G
                xi, xi_dot, drift, D = -xi, -xi_dot, -drift, -D
            elif self.reading == "swap_xi_rows":
                P = np.array([[0.0, 1.0], [1.0, 0.0]])
                xi, xi_dot, drift, D = P @ xi, P @ xi_dot, P @ drift, P @ D
            elif self.reading == "G_sign_flipped_in_law":           # controller uses -G in (31)-(32)
                xd, yd, _ = xt.pe_dot
                G_dot = np.array([[0.0, yd], [0.0, -xd], [0.0, 0.0]])
                drift, D = drift - 2.0 * xt.H @ G_dot @ z, -D
        s = xi_dot + self.alpha * xi + self.beta * sig(xi, self.gamma)
        tau_eq = -np.linalg.solve(D, drift + self.alpha * xi_dot
                                  + self.beta * self.gamma * abs_pow(xi, self.gamma - 1.0) * xi_dot)
        L = sat(s, self.boundary_layer)
        if self.reading == "L_negated":
            L = -L
        LC = L * self.C_hat(v, w)
        tau_sw = -np.linalg.solve(D, LC) if self.reading == "switching_inside_inverse" else -self.B_inv_M0 @ LC
        tau = tau_eq - np.linalg.solve(D, self.K1 * s + self.K2 * sig(s, self.mu)) + tau_sw
        if self.reading == "wheel_order_swapped_in_controller":
            tau = tau[::-1]
        self._diag = {"s1": s[0], "s2": s[1], "c1": self.c[0], "c4": self.c[3]}
        self.adapt(s, v, w)
        return tau


READINGS = {
    "printed (as published)": "printed",
    "error e = p - p_r, F/G re-derived consistently": "reversed_error_consistent",
    "error e = p - p_r, printed F/G kept": "reversed_error_printed_formulas",
    "xi rows swapped": "swap_xi_rows",
    "wheel order swapped in controller only": "wheel_order_swapped_in_controller",
    "G sign flipped in law (31)-(32)": "G_sign_flipped_in_law",
    "L = -diag(sgn s) (implicit sign)": "L_negated",
    "switching term inside (HGM0^-1B)^-1": "switching_inside_inverse",
}


def simulate_readings() -> pd.DataFrame:
    gains = load_config("configs/controllers_paper.yaml")["afntsmdc"]
    gains = {k: (tuple(v) if isinstance(v, list) else v) for k, v in gains.items() if k != "switching"}
    cases = [("S1", "configs/robot_paper.yaml", None), ("S2", "configs/s2_payload.yaml", None),
             ("S1 + 20 ms lag", "configs/robot_paper.yaml", 0.020)]
    rows = []
    for case, path, lag in cases:
        cfg = load_config(path)
        cfg["plant"]["actuator_tau"] = lag
        ref_ctrl = build_controller("afntsmdc", load_config("configs/controllers_paper.yaml"), cfg)
        for label, reading in {**READINGS, "sign-corrected (our reproduction)": None}.items():
            ctrl = ref_ctrl if reading is None else Reading(reading, params=ref_ctrl.params, Ts=cfg["sim"]["Ts"], **gains)
            try:
                df = simulate_from_config(cfg, ctrl)
            except FloatingPointError:
                rows.append({"case": case, "reading": label, "diverged": True})
                continue
            late = df[df["t"] >= 15.0]
            m = tracking_metrics(df)
            rows.append({"case": case, "reading": label, "diverged": False,
                         "late_pos_err_cm": float(np.hypot(late["x_e"], late["y_e"]).max() * 100),
                         "x_e_settle_s": m["x_e"]["settle"],
                         "c4_final_over_c4_0": float(df["c4"].iloc[-1] / df["c4"].iloc[0]),
                         "stable": bool(np.hypot(late["x_e"], late["y_e"]).max() < 0.02)})
    return pd.DataFrame(rows)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    sym = symbolic()
    (OUT / "sign_verification_symbolic.txt").write_text(sym + "\n")
    print(sym)
    sims = simulate_readings()
    sims.to_csv(OUT / "sign_verification_sim.csv", index=False)
    (OUT / "sign_verification_sim.md").write_text(
        "# Printed AFNTSMDC law under alternative readings (ideal plant unless stated, paper gains, no noise)\n\n"
        "stable = late (t >= 15 s) position error < 2 cm.\n\n" + markdown_table(sims) + "\n")
    print(sims.to_string(index=False))


if __name__ == "__main__":
    main()
