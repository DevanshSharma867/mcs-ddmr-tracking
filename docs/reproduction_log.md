# Reproduction log

Item IDs (M1…M15, D1) refer to `docs/paper_notes.md` §8. Decided 2026-10-19.

## Assumptions
<!-- One line per ASSUMED value: what, value, why. -->

| ID | ASSUMED | Value / rule | Why |
|---|---|---|---|
| M1 | Units of the control input | The controller output is the generalized input τ of (8)/(11), in "model units" (the paper's "V"). No motor model. Plots are labelled "control input (model units; paper: V)". | The paper gives no motor model (torque constant, gearing, driver mode). If τ is treated as the paper's input, the paper's gains can be used unchanged. |
| M2 | Input saturation | ±8 model units per wheel. It can be switched off in the config (`saturation: off`). | Fig. 6/8/10 inputs appear to clip near −8 V. The paper does not state a limit. |
| M3 | Shock disturbance | `6·|sin(2πf(t−12))|`, f = 1 Hz, model units, added to **each** wheel's input channel for 12 ≤ t ≤ 12.5 s, else 0 (eq. 61). This is exactly one half-sine hump and identical to the printed `6|sin(2πt)|`. **Wheel signs [+1, +1]** (configurable via `wheel_signs`). | Eq. (61) gives d1 = d2 with amplitude 6 V "acting on the wheels". In our units this is an input disturbance. Same positive sign: (61) states d1 = d2, and Fig. 9 shows a forward push (x_e dips negative, φ_e nearly unaffected). Whether the paper's d1, d2 were per-wheel or in the (v, ω) channels of (9) is not stated, so the per-wheel reading is ASSUMED. |
| M7 | Sign of disturbance injection | The plant input to each wheel is τ_i + d_i. **Conflict noted:** (9) has `M ż = Bτ + d`, but (11) defines δ = ΔM ż + d and enters it as `−δ`. Taken literally, these disagree on the sign of d. | Adding the disturbance to the input is the physically consistent choice given M3. The controller only uses \|δ\| bounds, so the sign does not affect the design. |
| M4 | Measurement noise | None in the reproduction (Phases 3–4). Noise is added only in Phase 5. | The paper gives no noise levels (feedback was encoder odometry). Keeping the reproduction deterministic isolates the controller behaviour. |
| Init | Initial pose sign | p(0) = (−0.1, −0.1, 0), i.e. the robot starts below and to the left of the reference start. **Confirmed, not assumed.** | Fig. 5a paths start at about (−0.1, −0.1). Fig. 5c–e start at x_e = y_e = +0.1. |
| Sat | Saturation vs disturbance order | The controller command is saturated first, and then the input disturbance is added. A shock can therefore push the wheel input beyond ±8. | The disturbance acts on the wheel physically, downstream of the actuator limit. |
| M5 | Initial velocity | z(0) = [v, ω] = [0, 0] (robot at rest). | Not stated. A robot at rest at the start of an experiment is the natural reading. |
| M6 | Settling time and window | Settling time is the **last** time the error is outside its band: \|x_e\| ≤ 1 cm, \|y_e\| ≤ 1 cm, \|φ_e\| ≤ 0.02 rad, each error separately. RMS and MAX are taken over the full 0–20 s. | The paper does not define the tolerance band or the RMS window. |
| M8 | Plant model | The reproduction plant is (8) as printed: `M ż = Bτ + d`, M = diag(m, I), h ignored (`use_h_terms: false`). The full reduced model is kept as a config option for Phase 5 (unmodelled dynamics): `m v̇ = … − m h ω²`, `I ω̇ = … + m h v ω`, with h = 0.11 m. | The paper designs for and presents (8). The h-terms appear when (1)–(2) are reduced but are dropped in the paper. Their signs follow the paper's convention: the mass centre is h *behind* the axle (Fig. 1). This equals Fierro–Lewis with d = −h, so it is not a conflict (paper_notes §1.5). |
| M9 | Boundary layer scope | AFNTSMDC: the discontinuous switching term L = diag(sgn(s1), sgn(s2)) in (32)–(33) uses sat(s_i) with ϱ = 0.04 (eq. 55). sig(s)^μ keeps its exact form, since it is already continuous for μ > 0. CSMDC and RSMKC use pure sgn. | Remark 3 applies the boundary layer only to the AFNTSMDC reaching law (32), where the only discontinuity is L. Nothing is said for the baselines. |
| M10 | Payload scenario (S2) | Same initial offset as S1, (−0.1, −0.1, 0). True mass m = 4.5 + 3.6 = 8.1 kg. True inertia I = 0.56 · 8.1/4.5 = 1.008 kg·m². The controller keeps the nominal m0 = 4.5, I0 = 0.56. | The paper says only "an initial posture offset". The printed ΔI = 0.01 kg·m² is not used: scaling the inertia with the mass is the user's decision, giving a more meaningful inertia mismatch. |
| M10 | Shock scenario (S3) | S3 = payload (S2 plant) + shock (M3). Table 4 metrics are computed over both 0–20 s and 12–20 s, and we report which window matches the paper better. | The RMS values in Tables 3 and 4 are almost identical, so S3 was very likely run with the payload. The paper does not state the window. |
| M11 | Heading error wrapping | φ_e is wrapped to (−π, π]. | The paper does not discuss wrapping. Wrapping is needed for long runs, where the heading grows past ±π. |
| M12 | PI inner loop (CKC, RSMKC). **Revised 2026-10-19 (Phase 3).** | The PI acts on z_e = z_d − z (eq. 59) with k_P = 30, k_I = 250, using rectangular integration at Ts. Its output u = [u_v, u_ω] is a **generalized force/torque in the (v, ω) channels of (8)**, and the wheel inputs are **τ = B⁻¹u** with the nominal r and b (no m or I). Clamping anti-windup: a tick's integrator update is discarded if either wheel would exceed u_max. Integrators start at 0. *Superseded:* the original literal mapping "u1 → right, u2 → left". | The paper does not state the units of u or how u maps to the wheels. Fig. 3 feeds u into "robot dynamics (11)", whose input is Bτ. Scratch test (CKC+PI, S1, paper gains, Ts = 0.01): see the table below. The literal mapping is unstable in our model units: the ω-error drives only the left wheel, which lowers ω, so that loop has the wrong sign. On hardware, back-EMF adds damping that our M1 plant lacks. The B⁻¹ form keeps the PI baseline free of m and I. |
| CSMDC-ρ | ρ in CSMDC's error function ξ | ρ = 0.8 (same as AFNTSMDC). | (60) says "ξ is with (22)", but the paper gives ρ only for AFNTSMDC (Sect. 3.3). Using the same ξ keeps the comparison like-for-like. |
| M13 | Initial adaptive gain ĉ4(0) | ĉ4(0) = 6. | The paper prints "c2(0) = 6" a second time, which is clearly a typo for the 4th gain. |
| M14 | Numerical guards | sgn(0) = 0. Before any fractional power is taken, \|x\| is clipped to at least 1e-6 (in `sig^a` and `⌊ξ⌋^(γ−1)`). The guards are documented in code docstrings. | The paper describes no numerical treatment. The guards avoid 0^negative and NaN at x_e = 0 or s = 0. |
| M15 | Derivatives in τ_eq | Ḣ, Ḟ and Ġ are computed exactly from the model state (measured p and z) and the analytic reference (v̇_r, ω̇_r). | This is available in simulation. On hardware they would come from estimated velocities. |
| Ref | Reference circle | Centre (0, 0.6) m, radius 0.6 m, start (0, 0), φ_r(0) = 0, counter-clockwise, ω_r = 0.7 rad/s, v_r = 0.42 m/s, 20 s. | Radius and speed are stated (Sect. 4.3). Centre, start point and direction are read from Fig. 5a/b. |
| Ts | Sampling and integration | Controller Ts = 0.01 s with zero-order hold (the paper's value). The plant is integrated with fixed-step RK4 at dt = 1e-3 s. | Ts = 0.01 s is stated for all controllers (Sect. 4.2). The finer plant step keeps the integration error negligible. |
| D1 | Controller set | Kinematic-only ideal reference (perfect velocity tracking), CKC+PI, RSMKC+PI, CSMDC, AFNTSMDC, plus our improvement later. The generic PID/SMC inner-loop baselines from the original CLAUDE.md are dropped. | This matches the paper's comparison (Sect. 4.1), so the figures and tables can be compared directly. |

## Discrepancies vs. paper
<!-- What differs from the paper's figures/tables, by how much, and suspected cause. -->

**M12 scratch test** (CKC+PI on S1, paper gains, Ts = 0.01 s, u_max = 8; 2026-10-19):

| PI output u → wheels | late max position error (t ≥ 10 s) | max \|u_cmd\| | % ticks saturated |
|---|---|---|---|
| literal: u1 → right, u2 → left | 1.37 m (diverges with saturation off) | 6.4e6 | 100 % |
| per-wheel speed PI (z_d → wheel speeds) | 3.6 cm, limit cycle | 656 | 100 % |
| **[force, torque], τ = B⁻¹u (adopted)** | 0.9 mm | 3.2 | 0 % |
| desired acceleration, τ = B⁻¹M0·u | 0.9 mm | 3.4 | 0 % |

Known in advance (structural):
- The paper's results are hardware measurements (odometry feedback, voltage inputs, friction, sensor quantisation). Our simulation is idealised, so agreement can only be qualitative (trends, rankings, orders of magnitude).
- The paper's own text and tables disagree on some settling times (paper_notes §6). We compare against the **tables**.

### Phase 3: baselines on S1 vs paper Fig. 5 / Fig. 6 / Table 2 (qualitative)
Source: `results/baselines/` (`python -m experiments.baselines`). Paper values: Table 2.

| Controller | Ours: RMS / MAX / settle (x_e cm; y_e cm; φ_e rad) | Paper Table 2 |
|---|---|---|
| CKC+PI | x_e 1.75 / 12.80 / 2.24 s; y_e 1.66 / 10.00 / 2.04 s; φ_e 0.012 / 0.037 / 2.82 s | x_e 3.30 / 11.60 / 6.07; y_e 2.44 / 10.02 / 7.52; φ_e 0.015 / 0.032 / 5.93 |
| RSMKC+PI | x_e 2.95 / 13.34 / 4.39 s; y_e 2.02 / 10.00 / 7.15 s; φ_e 0.014 / 0.036 / 5.73 s | x_e 3.29 / 12.12 / 4.81; y_e 2.53 / 10.01 / 7.59; φ_e 0.023 / 0.064 / 7.39 |
| CSMDC | x_e 1.20 / 10.40 / 0.64 s; y_e 2.80 / 10.00 / **not settled**; φ_e 0.023 / 0.065 / 16.52 s | x_e 3.30 / 17.52 / 2.38; y_e 3.75 / 10.68 / 9.36; φ_e 0.047 / 0.215 / 3.98 |

**Agreements**
- **RSMKC+PI: close match.** x_e falls almost linearly from 0.1 m to 0 in ≈ 4.4 s (paper: same linear ramp, 4.81 s). This is the sliding phase at the rate k_r3 = 0.02 m/s (0.1/0.02 = 5 s). There is a y_e undershoot of −3.4 cm near 3.8 s (paper ≈ −5 cm near 3.5 s) and a positive φ_e bump of 0.036 rad near 3.8 s (paper ≈ 0.05 rad). The y_e settling time (7.15 s vs 7.59 s) and the x_e and φ_e magnitudes are similar.
- **CKC+PI:** the MAX values match (x_e 12.8 vs 11.6 cm, φ_e 0.037 vs 0.032 rad). Both show the early x_e overshoot above 0.1 m and a single y_e undershoot.
- **CSMDC:** the fastest x_e convergence of the three baselines, in both ours and the paper's (0.64 s vs 2.38 s). The full ranking differs: the paper has CSMDC < RSMKC < CKC, ours has CSMDC < CKC < RSMKC, because our CKC+PI is much faster than the paper's (see below). It also has the slowest y_e, with a slow monotone decay and no undershoot (as in Fig. 5d).
- All paths start at (−0.1, −0.1) and converge onto the circle (Fig. 5a).

**Discrepancies**
- **CKC+PI is about 2.5–3× faster than the paper's** (settling 2.0–2.8 s vs 5.9–7.5 s), with a smaller y_e undershoot (−0.9 cm vs ≈ −5 cm). It is almost indistinguishable from the ideal kinematic reference. Suspected cause: our PI inner loop is idealised (no friction, back-EMF, motor lag or noise), so z ≈ z_d. The paper's hardware loop evidently tracks z_d much more slowly, which makes its CKC behave like a lower-gain kinematic loop.
- **CSMDC has a steady offset: x_e ≈ −0.73 cm, and y_e wanders at 1.4–2.8 cm without settling.** φ_e settles only at 16.5 s. Cause, verified: pure sgn sampled at Ts = 0.01 s. The discrete map s ↦ s − K_c·Ts·sgn(s) has a continuum of period-2 cycles, so s ends up in a ±K_c·Ts/2 band (≈ ±0.1) around an arbitrary, biased mean (late mean s = (−0.022, 0.028)). On the sliding surface that gives ξ₁ = s̄₁/λ₁ = −0.022/3 = −0.73 cm, matching the observed x_e offset. With Ts = 1 ms the same controller settles (x_e 0.76 s, y_e 5.33 s, φ_e 3.69 s; final \|x_e\|, \|y_e\| < 0.2 cm). **This fails the fixed Phase 3 acceptance test for CSMDC y_e and φ_e (2× paper settling time).** The thresholds are unchanged; those two cases are `xfail(strict=True)` (decision A, 2026-10-19). See "CSMDC finding" below. The paper's own CSMDC y_e (Fig. 5d) also decays slowly and is still ≈ 0.5–1 cm late in the run.
- **CSMDC early spike missing:** the paper shows x_e jumping to ≈ 0.175 m and φ_e to ≈ 0.22 rad at ≈ 0.4 s. Ours peaks at 0.104 m and dips to −0.065 rad instead. Suspected cause: hardware start-up effects (actuator lag or static friction from rest) that our ideal plant lacks.
- **Input magnitudes and shape (Fig. 6):** our steady inputs average ≈ 0 because the plant has no friction or back-EMF. The paper sits at ≈ −1.5 V (right) and −0.5 V (left); the constant offset suggests friction (→ Phase 5 candidate) and a motor polarity convention. In the paper, CKC/RSMKC show isolated spikes to −8 V and CSMDC has the fewest spikes. In ours it is the reverse: CKC is smooth, RSMKC chatters at ±0.3 (sgn in the outer loop), and CSMDC chatters at ±1 (right) and ±3 (left) every tick. The likely reason is that motor electrical and mechanical dynamics in the real hardware filter the switching. Our inputs never reach ±8, so saturation is 0 % for all controllers.
- φ_e in the paper carries measurement noise. Ours is noise-free (M4).

### CSMDC finding: discrete-time quasi-sliding at Ts = 0.01 s (a property of the baseline, kept as specified)
- **Mechanism.** The paper's CSMDC (60) uses a pure sgn, evaluated once per sample and held for Ts. With exact equivalent control, the sampled sliding variable evolves roughly as s_{k+1} ≈ s_k − K_c·Ts·sgn(s_k). This map has no fixed point at 0. Instead it has a continuum of period-2 cycles {a, a − K_c·Ts} for any 0 < a < K_c·Ts. So s chatters in a band of width ≈ K_c·Ts (here 0.21 and 0.16) around an **off-centre mean** fixed by the transient, not at 0. A nonzero mean s̄ on s = λ_c ξ + ξ̇ means ξ → s̄/λ_c, i.e. a **steady tracking offset**.
- **Evidence (S1, paper gains).**
  - Late mean s = (−0.022, 0.028); peak-to-peak band (0.21, 0.33) ≈ K_c·Ts. That predicts x_e = s̄₁/λ₁ = −0.022/3 = −0.73 cm, and the observed x_e is −0.73 cm.
  - y_e wanders at 1.4–2.8 cm and never settles. φ_e settles only at 16.5 s.
  - **With Ts = 1 ms** (same gains): band ≈ (0.021, 0.017), mean ≈ (−0.005, −0.005). It settles with x_e at 0.76 s, y_e at 5.33 s and φ_e at 3.69 s, and the final \|x_e\| and \|y_e\| are < 0.2 cm.
- **Boundary layer (diagnostic "CSMDC+BL", not the paper baseline).** Replacing sgn(s) with sat(s/ϱ) makes the switching linear inside the layer, with gain K_c/ϱ. The sampled version is stable only if **K_c·Ts/ϱ < 2**. Sweep at Ts = 0.01 s (S1):

  | ϱ | K_c·Ts/ϱ (s₁, s₂) | y_e settle | final x_e / y_e | chattering [units/s] |
  |---|---|---|---|---|
  | none (paper CSMDC) | — | not settled | −0.73 / 1.44 cm | 792 |
  | **0.04 (AFNTSMDC's ϱ; the diagnostic in `results/baselines/`)** | 5.25, 4.0 | not settled | −0.73 / 2.29 cm | 787 |
  | 0.105 | 2.0, 1.52 | 4.86 s | 0.05 / −0.09 cm | 804 (s₁ still chatters) |
  | 0.15 | 1.4, 1.07 | 5.02 s | 0.000 / 0.002 cm | 0.5 |
  | 0.30 | 0.7, 0.53 | 5.02 s | 0.000 / 0.002 cm | 0.3 |

  With the paper's ϱ = 0.04, the layer is too thin for Ts = 0.01 s. CSMDC+BL behaves almost like pure sgn (φ_e happens to settle at 4.1 s, but y_e still does not).
- **Implication for Phase 4 (a prediction to check, not a result).** AFNTSMDC applies the same ϱ = 0.04 to its switching term L·Ĉ, with ĉ₁(0) = 14 (and growing), so ĉ·Ts/ϱ ≈ 3.5 > 2. Its sig(s)^μ term (μ = 0.8) also has unbounded slope at s = 0. Expect similar sampled chattering in our idealised plant at Ts = 0.01 s. The paper's hardware (motor electrical and mechanical lag) would filter it; see the actuator-lag Phase 5 candidate.

## Decisions
- **2026-10-19:** Architecture follows the paper. AFNTSMDC and CSMDC are combined controllers (pose + velocity errors → wheel inputs). CKC and RSMKC are kinematic outer loops with a PI inner loop. CLAUDE.md was updated to match.
