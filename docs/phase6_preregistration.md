# Phase 6 pre-registration: improved AFNTSMDC (written and committed before any Phase 6 run)

Date: 2026-10-19. This file is committed **before** any Phase 6 implementation run. Nothing here may
be changed after results exist. Deviations are reported as deviations, and failures as failures.

## Design under test (all parameters fixed here)
Base: the reproduced AFNTSMDC (sign-corrected switching), paper gains α = diag(7, 7),
β = diag(7, 6), γ = 1.67, ρ = 0.8, K1 = diag(9, 4), K2 = diag(15, 5), μ = 0.8, ζ = (0.3, 0.4, 0.4, 0.5),
dead zone ε = 0.05, Ts = 0.01 s.

**Components**
- **ESO**, one per channel j ∈ {v, ω}:
  - The measured value is y = z (v, ω). The model input is w = M0⁻¹·B·sat(τ) (saturated command, nominal model).
  - Update: ẑ_{k+1} = ẑ_k + Ts·(w_k + f̂_k + β1·(y_k − ẑ_k)), f̂_{k+1} = f̂_k + Ts·β2·(y_k − ẑ_k).
  - β1 = 2ω_o, β2 = ω_o², with **ω_o = 25 rad/s** (discrete poles 0.75, double). Initial ẑ = y_0, f̂ = 0.
  - Compensation: τ_ESO = −B⁻¹·M0·f̂.
- **Projection + bounded Ĉ + sized boundary layer (PS):**
  - ĉ₁, ĉ₂ ∈ [0.5, 2.5] and ĉ₃, ĉ₄ ∈ [0, 1]. The adaptive law (37)–(38) is applied with projection
    (the update is stopped at a bound), plus the dead zone (53)–(54).
  - ĉ(0) = (0.5, 0.5, 0, 0).
  - Ĉ_j = min(ĉ_j + ĉ_{j+2}·|z_j|, Ĉ_max) with **Ĉ_max = 3** (m/s², rad/s²).
  - Switching term in the general form, τ_sw = −D⁻¹·L·Ĉ with D = HGM0⁻¹B.
  - L = diag(sat(s_j/ϱ_j)), with **ϱ_j = κ_p·Ĉ_max·Ts / (1 − κ_p·K1_j·Ts)** and κ_p = 1.5, giving
    **ϱ = (0.0520, 0.0479)**.
  - Equivalent condition: κ_p·(K1_j + Ĉ_max/ϱ_j)·Ts ≤ 1 (monotone discrete convergence inside the layer).

**Arms**

| Arm | ESO | PS | Switching term |
|---|---|---|---|
| A0 | – | – (paper ĉ(0) = (14, 14, 8, 6), ϱ = 0.04, sign-corrected +B⁻¹M0LĈ; the existing `afntsmdc`) | yes |
| A1 | ✓ | – | yes (as A0) |
| A2 | – | ✓ | yes (PS form) |
| A3 (full) | ✓ | ✓ | yes (PS form) |
| A4 (Occam) | ✓ | – | **none** |

If A4 ≈ A3, that is reported plainly as a finding.

**Experiments** (unchanged configs; arms added as controller ids only):
- Phase 5 robustness grid (`configs/robustness.yaml`): 4 new arms × 360 = 1,440 runs, seeds 0–4.
- Reproduction S1–S3 on all 6 plants (`configs/reproduction.yaml`): 4 × 18 = 72 runs.
- 200 s drift test (`configs/drift.yaml`): A1, A3 and A4, 108 runs.
- ω_o sweep {12.5, 25, 50} rad/s for A3 only: nominal, noise ×2 and dist_bandlim, both plants, both
  trajectories, 5 seeds = 180 runs. **Descriptive only; ω_o is not re-picked from it.**

## Metrics and rules
- **Late error** = max position error over the last 5 s of a 20 s run, mean over seeds 0–4
  (`pos_late_max_cm`). **Chattering** = `chattering` (input TV/s), mean over seeds.
- **Significance rule** for comparisons between arms: a difference counts only if
  |mean_X − mean_Y| > 2·max(std_X, std_Y) over seeds. Deterministic runs (reproduction scenarios,
  no noise) have one value and are compared directly.
- Absolute thresholds are pass if the mean is ≤ the threshold.

## Success criteria (A3 unless stated; plant (a) = `torque`; both trajectories unless stated)
Reference values for A0 are taken from the Phase 5 grid (`results/robustness/runs.csv`, commit 692b419)
and from the Phase 5 shock analysis.

- **P1 Offsets removed.** The late error under `dist_const`, `dist_yaw`, `friction_10` and
  `friction_30` is **≤ 0.5 cm each**, on circle and figure-8.
  (A0 circle: 1.29 / 1.31 / 1.18 / 1.77 cm; figure-8: 1.31 / 1.23 / 1.13 / 1.35 cm.)
- **P2 Disturbance rejection kept.** For `dist_sin1Hz`, `dist_sin0.2Hz` and `dist_bandlim`, A3 is
  **not significantly worse than A0** under the 2× seed-std rule, i.e.
  mean_A3 − mean_A0 ≤ 2·max(std_A3, std_A0).
  (A0 circle: 0.78 / 1.01 / 0.82 cm; figure-8: 0.69 / 0.86 / 0.66 cm.)
- **P3 Chattering.**
  - (a) The chattering index at `nominal` is **≤ 100**, on circle and figure-8 (A0: 837 / 781).
  - (b) At `mass_x0.7` and at `noise_x2` it is **≤ 2× A3's own nominal value**, on circle and figure-8.
- **P4 No nominal regression.**
  - (a) Reproduction S1, ideal plant: x_e settling time **≤ 1.36 s** (2× paper Table 2, as in Phase 4).
  - (b) Grid `nominal` late error **≤ A0's**: circle 0.80 cm, figure-8 0.76 cm.
- **P5 Plant (c12V)** (`motor12V`, `nominal`):
  - circle late error **≤ 2.0 cm**, and
  - figure-8 late error **≤ A0's 6.06 cm**.
  - **Also reported (no threshold):** A3 vs CKC+PI on (c12V): circle 1.33 cm, figure-8 13.73 cm,
    with the significance rule applied.
- **P6 S3 shock.** The shock-induced x_e is max over t ∈ [12, 20] s of |x_e,S3(t) − x_e,S2(t)| (same plant
  and controller). It must be **≤ A0's** on the ideal plant (A0: 16.25 cm) and on motor12V (A0: 6.09 cm).
- **P7 Bounded adaptation.**
  - ĉ never leaves [ĉ_min, ĉ_max]. This is checked by a unit test and over all A2/A3 grid and drift runs.
  - **Reported, no threshold:** the fraction of time each ĉ_i sits at ĉ_max in the 200 s drift test.

## Ablation interpretation (stated in advance)
- A1 vs A0: the effect of the ESO.
- A2 vs A0: PS without the ESO. This arm is **expected to degrade** under large structured
  disturbances, because Ĉ_max = 3 cannot cover them.
- A3 vs A1: the effect of PS, given the ESO.
- A4 vs A3: whether the switching term is still worth having once the ESO is present.
