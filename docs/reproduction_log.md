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
| Lag | Actuator lag (plant variant b, Phase 4) | A per-wheel first-order low-pass `ȧ = (sat(u) − a)/τ_a` between the saturated command and the wheel. The disturbance is added after it. τ_a ∈ {10, 20, 40} ms, **fixed before any comparison**, off by default (`plant.actuator_tau: null`). | A phenomenological stand-in for unmodelled actuation: motor electrical lag, PWM/driver filtering, gearbox compliance, compute/update delays. Not derived from motor mechanical time constants, because the plant already models speed dynamics through m and I, so that would double-count. A sensitivity check, **not a fit** to the paper's curves. |
| S2/S3 | Scenario configs | `configs/s2_payload.yaml`: `robot.actual = {m: 8.1, I: 1.008}` (M10). `configs/s3_payload_shock.yaml`: S2 plus the eq. (61) shock (M3). Both are deep-merged over `robot_paper.yaml` via `base:`. | As decided in M3 and M10. |
| Adapt | Discretisation of the adaptive law | ĉ is updated by forward Euler at Ts, **after** τ is computed (τ_k uses ĉ_k). The dead zone (53)–(54) is applied per channel. | The paper gives the continuous law only. |
| SW | **DEVIATION: sign of AFNTSMDC's adaptive switching term** | The reproduced controller uses **+B⁻¹M0·L·Ĉ** instead of the printed −B⁻¹M0·L·Ĉ in (32) (`switching: corrected`). The as-printed law is kept as a literal-reproduction run. | The printed sign is destabilising. See "AFNTSMDC switching sign (deviation)" below. Decided 2026-10-19 (option A). |
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

### AFNTSMDC switching sign (deviation, Phase 4)
**Derivation.** Substituting (32) into (30) gives (42):
`ṡ = −K1 s − K2 sig(s)^μ − HG(LĈ − M0⁻¹δ)`. Near the origin (x_e, y_e → 0),
H → [[1, 0, 0], [0, ρ/ρ = 1, 1]] and G → [[−1, 0], [0, 0], [0, −1]], so **HG → −I**. The switching
term then becomes `−HG·L·Ĉ ≈ +L·Ĉ = +sgn(s)·Ĉ`, which drives s *away* from 0 with magnitude Ĉ ≈ 14.
The adaptive law raises ĉ whenever |s| > ε, so the push grows: the printed law is destabilising.

The paper's proof step `−sᵀHG(LĈ − M0⁻¹δ) ≤ −⌊s⌋ᵀ⌊HG⌋(Ĉ − ⌊M0⁻¹δ⌋)` (Sect. 3.2) is only valid if
HG ≈ +diag(positive). With the paper's own definitions (19) and (24), HG ≈ −I, so the sign is
reversed (proof weakness 1 in paper_notes §4.1 is therefore a sign problem, not only an
off-diagonal one).

Implementation verified first: a finite-difference ṡ of the coded law equals eq. (42) exactly,
on two trajectories (`tests/test_afntsm.py::test_sliding_dynamics_eq42`).

**Corrections.**
- **Sign flip (adopted as the reproduced controller):** τ_sw = +B⁻¹M0·L·Ĉ, so ṡ gets `+HG·L·Ĉ ≈ −L·Ĉ`.
  This is the minimal change. It makes the paper's own Lyapunov step valid where HG ≈ −I (i.e.
  near the origin / on the sliding regime).
- **General (erratum-level) fix:** τ_sw = −(HGM0⁻¹B)⁻¹·L·Ĉ, so ṡ gets exactly `−L·Ĉ` for **any** HG.
  The proof step then holds without assuming anything about HG. It is included as variant
  `afntsmdc_general`. In all our scenarios it is within ≈ 2 % of the sign flip (`variants.md`), because
  the closed loop stays in the region where HG ≈ −I.
- *Hypothesis only (not verifiable):* the authors' hardware implementation likely used a working sign.

**Evidence (S1, paper gains, Ts = 0.01 s; scratch run 2026-10-19, confirmed by `results/reproduction/`):**

| Variant | Plant | Settle x_e / y_e / φ_e (s) | Late \|x_e\|, \|y_e\| (t ≥ 15 s) | ĉ at 20 s | Chattering |
|---|---|---|---|---|---|
| As printed (−B⁻¹M0LĈ) | ideal | never / never / never | 118 cm, 141 cm | 24, 61, 26, 113 (ramping) | 18 |
| As printed | lag 20 ms | never | 171 cm, 218 cm | 31, 53, 47, 111 | 20 |
| Sign-corrected (+B⁻¹M0LĈ) | ideal | 0.40 / 5.47 / 2.92 | 0.47 cm, 0.11 cm | 14.7, 14.7, 8.4, 6.5 | 839 |
| Sign-corrected | lag 20 ms | 0.38 / 5.19 / 3.69 | 0.53 cm, 0.13 cm | 14.7, 14.8, 8.4, 6.7 | 317 |
| General −D⁻¹·L·Ĉ | ideal | 0.41 / 5.51 / 2.89 | 0.48 cm, 0.10 cm | ≈ sign-corrected | 850 |
| No switching term | ideal | 0.35 / 5.47 / 3.82 | ≈ 0 | barely moves | 0 |

Plot: `results/reproduction/adaptive_gains_printed.png`. The printed law's ĉ₂ and ĉ₄ ramp from t = 0
(ĉ₄ jumps to ≈ 95 at ≈ 17 s), and the position error grows to > 1 m. Test:
`test_as_printed_law_diverges` (criterion fixed in advance: late position error > 10 cm and
ĉ₄(20) > 2·ĉ₄(0)).

**Acceptance of the reproduced (sign-corrected) controller**, with thresholds fixed in advance at
2× Table 2: **all pass, on both plants.**
- Ideal plant:
  - x_e: MAX 10.19 cm (limit 26.0), settle 0.40 s (limit 1.36)
  - y_e: MAX 10.00 cm (limit 20.4), settle 5.47 s (limit 17.7)
  - φ_e: MAX 0.065 rad (limit 0.344), settle 2.92 s (limit 8.92)
- τ_a = 20 ms:
  - x_e: MAX 10.60 cm, settle 0.38 s
  - y_e: MAX 10.00 cm, settle 5.16 s
  - φ_e: MAX 0.072 rad, settle 3.46 s

## Reproduction verdict (Phase 4)
Source: `python -m experiments.reproduction` → `results/reproduction/` (120 runs: 10 controllers ×
S1–S3 × {ideal, lag 10/20/40 ms}). "AFNTSMDC" below means the sign-corrected law at the paper's
gains (ϱ = 0.04) unless stated. Paper results are hardware measurements, ours are idealised
simulation (no noise, friction, back-EMF), so quantitative agreement was never expected.

| Paper item | Verdict | What matches / what does not | Likely causes |
|---|---|---|---|
| **Fig. 5 (S1 tracking)** | **Qualitative match** for AFNTSMDC and RSMKC+PI. **Partial** overall. | AFNTSMDC has the fastest x_e of all four (ours 0.40 s; paper 0.68 s), a slow monotone y_e decay with no undershoot, and a negative φ_e dip of ≈ −0.065 rad early (paper ≈ −0.07 rad after its initial spike). RSMKC+PI shows the linear x_e ramp to 0 in ≈ 4.4 s (paper 4.8 s), plus the y_e undershoot and φ_e bump. All paths start at (−0.1, −0.1) and converge. **Not matched:** CKC+PI settles 2.5–3× faster than the paper's. CSMDC keeps a −0.73 cm x_e offset and y_e never settles. The paper's early x_e/φ_e spikes for CSMDC (0.175 m / 0.22 rad) and AFNTSMDC (≈ 0.13 m / 0.17 rad) are absent. AFNTSMDC keeps a +0.47 cm x_e and +0.008 rad φ_e residual. | Idealised inner loop and plant. Discrete quasi-sliding at Ts = 0.01 s (CSMDC, AFNTSMDC). The start-up spikes are plausibly hardware effects (static friction, actuation from rest) that we do not model. |
| **Table 2 (S1)** | **Qualitative match on the paper's headline; not quantitative.** | AFNTSMDC has the best x_e RMS (0.82 cm vs paper 1.92) and the fastest x_e settling (0.40 vs 0.68 s), as in the paper. CKC has the best y_e and φ_e RMS among the four in both (ours 1.66 cm / 0.012 rad; paper 2.44 cm / 0.015 rad), so the paper's statement that AFNTSMDC does not lead on y_e is reproduced. x_e settling ranks AFNTSMDC < CSMDC first in both, but the order of CKC and RSMKC is swapped. Our absolute RMS values are 1.1–2.4× smaller than the paper's (e.g. RSMKC x_e 2.95 vs 3.29 cm; AFNTSMDC x_e 0.82 vs 1.92 cm). | No noise or quantisation in our simulation. Our CKC+PI is too fast. |
| **Fig. 6 (S1 inputs)** | **Not matched.** | Ours: CKC+PI is smooth. RSMKC+PI chatters at about ±0.3. CSMDC and AFNTSMDC chatter at every tick (chattering index ≈ 790 / 840 on the ideal plant). Steady inputs average ≈ 0. Paper: steady ≈ −1.5 V (right) and −0.5 V (left); CKC/RSMKC show spikes to −8 V; AFNTSMDC is described as "persistently smooth" with small residual chattering. | No friction or back-EMF (the steady offset). Discrete switching with ϱ = 0.04 too thin for Ts (see below). Hardware actuation filters the switching. |
| **Fig. 7 / Table 3 (S2 payload)** | **Qualitative match for AFNTSMDC's lead; payload degradation not reproduced.** | AFNTSMDC has the best x_e RMS (0.76 cm; paper 2.81) and the fastest x_e settling (0.31 s; paper 1.12). CKC+PI degrades slightly (x_e RMS 1.75 → 2.00 cm), as in the paper. **But** AFNTSMDC does **not** degrade from S1 to S2 in ours (x_e RMS 0.82 → 0.76 cm), whereas the paper reports 1.92 → 2.81 cm. CSMDC x_e no longer settles (paper 1.81 s). | The model-based equivalent control plus switching absorbs a 1.8× mass / inertia mismatch easily in an ideal plant. CSMDC: quasi-sliding offset. |
| **Fig. 8 (S2 inputs)** | **Not matched** | Same as Fig. 6. | Same as Fig. 6. |
| **Fig. 9 / Table 4 (S3 shock)** | **Qualitative match on the ranking only; magnitudes not matched (10–60× too large).** | x_e dips negative during the shock (a forward push), as in Fig. 9a, and AFNTSMDC has the smallest x_e MAX (16.0 cm vs CKC 94, RSMKC 104, CSMDC 163 cm). **But** the paper's MAX values are 1.6–2.8 cm. In ours RSMKC+PI and CSMDC are knocked ≈ 1–1.6 m off the path and do not re-settle y_e by 20 s. | **Shock scale (M1/M3):** 6 model units on both wheels gives v̇ = 2·6/(m r) ≈ 35 m/s² on the 8.1 kg plant for 0.5 s. On hardware, 6 V is attenuated by the motor constant and back-EMF. The 35 m/s² also exceeds CSMDC's switching gain (k_c1 = 21, in ṡ units), so its matching condition fails. That is a hypothesis, consistent with CSMDC losing sliding. |
| **Table 4 window** | **Not determinable from our runs.** | For AFNTSMDC x_e: settle measured from 12.0 s = 0.80 s, from 12.5 s = 0.30 s, paper 0.52 s. y_e: 3.74 / 3.24 vs 2.09 s. φ_e: 1.98 / 1.48 vs 1.12 s. Measuring from 12.5 s is closer for all three, but neither is within 40 %. RMS: our 12–20 s x_e RMS (2.79 cm) happens to equal the paper's 2.805 cm, but our full-run RMS is distorted by the oversized shock, so this is coincidental. | The paper's own numbers (Table 4 RMS ≈ Table 3 RMS; Table 4 MAX ≪ the initial 10 cm offset) still suggest RMS over the full run and MAX/settle over the post-shock window. |
| **Fig. 10 (S3 inputs)** | **Not matched** | Same as Fig. 6. | Same as Fig. 6. |

**Specific questions**
1. **Predicted AFNTSMDC chattering at Ts = 0.01 s, ϱ = 0.04: confirmed on the ideal plant** (chattering
   index 839 in S1, 753 in S2, 749 in S3, vs 0.4 with no switching term). s chatters in a ≈ ±0.2
   band (`adaptive_gains_S1.png`). **Actuator lag reduces it but does not remove it:** S1 index
   839 → 416 → 326 → 240 for τ_a = 0, 10, 20, 40 ms (CSMDC: 792 → 406 → 346 → 217; `chattering_vs_lag.png`).
   With a boundary layer that is stable in discrete time (ϱ = 0.15) it is 9.2 (ideal). But at τ_a = 40 ms
   AFNTSMDC with ϱ = 0.15 chatters again (211), so lag can also destabilise the boundary-layer loop.
2. **ĉ₁ to ĉ₄ ramp linearly for the whole run, in every scenario and plant** (`adaptive_gains_S*.png`):
   - S1 ideal: ĉ₁ 14 → 14.67, ĉ₄ 6 → 6.49 in 20 s.
   - S1, τ_a = 40 ms: ĉ₁ → 14.85, ĉ₄ → 6.97.
   - The ramp does not level off: the chattering band of s (≈ ±0.2) is wider than the dead zone ε = 0.05,
     so Remark 2's dead zone never engages.
   - The drift is slow at the paper's ζ over 20 s, but it is unbounded in principle.
   - As printed, ĉ ramps fast: ĉ₄ reaches 113 by 20 s.
   - This motivates leakage or projection (CLAUDE.md improvement candidate).
3. **AFNTSMDC's advantage over CSMDC once CSMDC has a working boundary layer** (ϱ = 0.15 for both;
   ideal plant; `variants.md`):
   - **x_e advantage survives:**
     - S1: settle 0.33 vs 0.81 s (≈ 2.5× faster), RMS 0.67 vs 1.03 cm (≈ 35 % lower).
     - S2: 0.33 vs 0.83 s, 0.74 vs 1.11 cm.
   - **y_e / φ_e advantage does not survive:** CSMDC+BL settles slightly faster.
     - S1: y_e 5.02 vs 5.33 s, φ_e 3.37 vs 3.67 s.
     - S2: 5.08 vs 5.36 s, 3.42 vs 3.70 s.
   - Against the paper-spec CSMDC (pure sgn), AFNTSMDC also wins on y_e and φ_e. That part of the
     advantage is the boundary layer / quasi-sliding, not the terminal surface.
   - **S3: a large advantage survives regardless of the boundary layer.** x_e MAX 16.2 cm (AFNTSMDC ϱ = 0.15)
     vs 166 cm (CSMDC+BL ϱ = 0.15). AFNTSMDC with *no switching term at all* still reaches 26.7 cm, so most
     of that comes from the continuous reaching law (K1 s + K2 sig(s)^μ, which grows with s) and the
     surface, not from the boundary layer.
   - At the paper's ϱ = 0.04, CSMDC+BL ≈ CSMDC (layer too thin, K_c·Ts/ϱ > 2).
4. **Does the adaptive switching term earn its keep?** (sign-corrected vs no switching term, ideal plant)
   - **S1: no.** Mixed performance: x_e RMS 0.82 vs 0.75 cm (worse), φ_e settle 2.92 vs 3.82 s (better),
     y_e settle equal. Chattering 839 vs 0.4.
   - **S2: marginally.** x_e RMS 0.76 vs 0.85 cm, y_e settle 5.15 vs 5.57 s, φ_e 3.39 vs 3.92 s
     (≈ 10 % better). Chattering 753 vs 0.4.
   - **S3: yes.** x_e MAX 16.0 vs 26.7 cm, RMS 1.92 vs 3.22 cm, late position error 1.4 vs 2.9 cm.
   - With ϱ = 0.15 the switching term keeps essentially all of the S3 benefit (MAX 16.2 cm, RMS 1.93 cm)
     at chattering 2.4 instead of 749.
   - Conclusion: the term is worth having under large disturbances, but the paper's ϱ = 0.04 makes it
     expensive at Ts = 0.01 s.
5. **Actuator-lag hypothesis from Phase 3: not supported for CKC+PI speed.** CKC+PI x_e settles at
   2.24 / 2.23 / 2.21 / 2.46 s for τ_a = 0 / 10 / 20 / 40 ms (paper 6.07 s). Lag up to 40 ms does not
   explain the 2.5–3× speed gap. It only partly explains the chattering gap (see 1).

**Bug found and fixed during Phase 4 (no results affected):** the bare YAML key `true:` (plant
parameters) parses as the boolean `True`, so `robot.true` was silently ignored. It was found before
any S2/S3 run (Phases 2–3 only used S1 with plant = nominal). It was renamed to `robot.actual`, with a
guard and a regression test.

### Plant (c): voltage-input DC motor with back-EMF (Phase 5) — set-up and prediction, written BEFORE any run
**Which motor.** The paper names no motor model. It says only "two dc motors (Maxon) which are
integrated with encoders" and "Maxon ESCON-36/2" drivers (Sect. 4.2, p. 1446). The Fig. 4 caption says
"DC motors & encoders". The authors' earlier robot paper [12] (Comput. Electr. Eng. 96, 2021) is closed
access and its abstract names no hardware. So we use a **representative** small Maxon brushed motor,
not the authors' motor:
- **Motor:** maxon RE 25 Ø25 mm, graphite brushes, 20 W. Two windings give the k_t/R sweep:
  - **12 V, part 339150:** R = 0.517 Ω, k_t = 11.5 mNm/A, speed constant 828 rpm/V (k_e = 0.01153 V·s/rad),
    rotor inertia 14.5 g·cm², L = 0.0573 mH → k_t/R = 0.0222 N·m/V.
  - **18 V, part 118751:** R = 1.33 Ω, k_t = 16.3 mNm/A, 586 rpm/V (k_e = 0.01630 V·s/rad),
    9.49 g·cm², L = 0.115 mH → k_t/R = 0.0123 N·m/V.
  - Source: maxon product pages 339150 and 118751 (accessed 2026-10-19).
- **Gearhead:** maxon GP 32 A planetary, **33:1 (529/16 absolute)**, listed for the RE 25 in the maxon
  GP 32 A catalogue page. Gear efficiency and gearhead inertia are not modelled (η = 1, ASSUMED).
- **Excluded before running:** the 24 V winding (118752: 2.32 Ω, 23.4 mNm/A). With the ±8 V limit (M2)
  it would need ≈ 10 V to hold the outer-wheel speed on the circle, so it is infeasible for any
  controller (a feasibility check, not a fit).
- The ratio 33:1 was chosen from the catalogue list so that the steady voltage on the reference
  circle (3.8 V / 5.4 V for 12 V / 18 V, outer wheel 5.0 V / 7.1 V) fits under 8 V. It was not
  chosen to match any paper curve.

**Model (per wheel, `plant.motor`, ASSUMED structure).**
- Wheel speed ω_w,R/L = (v ± bω)/r. Wheel torque
  `τ_w = n·(k_t/R)·(u_v − k_e·n·ω_w) − n²·J_m·ω̇_w`.
- `u_v` = saturated command (±8 V) + input disturbance (V).
- Equivalent body dynamics:
  `(m + 2n²J_m/r²) v̇ = n(k_t/R)(u_R+u_L)/r − 2n²k_t k_e v/(R r²)`, and
  `(I + 2n²J_m b²/r²) ω̇ = b n(k_t/R)(u_R−u_L)/r − 2n²k_t k_e b² ω/(R r²)`.
- **Rotor inertia** reflected through the gear is included as real inertia the controller does not know
  about (+1.80 kg and +0.062 kg·m² for 12 V; +1.18 kg and +0.041 kg·m² for 18 V). Controllers keep m0
  and I0.
- **Neglected:** inductance (L/R = 0.11 / 0.09 ms ≪ dt), friction, and gear efficiency/backlash.
- The controllers are unchanged: their output (designed as torque) is applied as volts, exactly as on
  the paper's hardware. The motor gain and back-EMF damping become part of δ.

**Two hypotheses for what the paper's "V" means (the paper does not say).**
1. *Voltage (PWM duty) mode* of the ESCON 36/2. Then plant (c) is the right structure.
2. *Current (torque) mode* of the ESCON. Its analog set value is in volts but commands motor current,
   i.e. torque. Then our original torque-input plant (with an unknown V→N·m scale) is the right
   structure, and plant (c) is wrong.

We keep both plants. The verdict is framed as "which plant is more consistent with the paper", not as proof.

**Prediction for the S3 shock on plant (c)** (back-of-envelope, S3 plant m = 8.1 kg plus rotor inertia,
shock 6·|sin| V on both wheels for 0.5 s). Computed 2026-10-19 before any plant-(c) simulation:
- **Open loop (no controller reaction):**
  - The v-channel is first order, with time constant m_eff/c = 31 ms (12 V) and 38 ms (18 V), which is
    ≪ 0.5 s, so the response is quasi-static.
  - 12 V: peak push 210 N, Δv_peak ≈ 0.65 m/s, extra displacement ≈ **21 cm**.
  - 18 V: 116 N, Δv_peak ≈ 0.46 m/s, ≈ **15 cm**.
  - For comparison, the torque plant gave Δv ≈ 11 m/s open loop (a_peak = 35 m/s²). That is about 17×
    (12 V) to 25× (18 V) more velocity disturbance than plant (c).
- **Closed loop:**
  - The controllers can only cancel the shock within their voltage headroom. On the circle the
    steady outer-wheel voltage is 5.0 V (12 V) or 7.1 V (18 V), so the headroom to the +8 V limit is
    only ≈ 3 V or ≈ 1 V. (The shock pushes forward, so the controller must *reduce* voltage, and the
    relevant headroom is toward −8 V. That is ≥ 11 V, enough to cancel 6 V if the loop is fast enough.)
  - So we expect the closed-loop x_e MAX to be set by loop bandwidth, not saturation. Our rough
    expectation is **a few cm (≈ 2–8 cm) for AFNTSMDC on 12 V, and somewhat larger on 18 V**, which is
    the paper's order of magnitude (1.6–2.8 cm).
  - This is a rough prediction. Nothing will be tuned to match it.

### Plant (c) results vs prediction (Phase 5, 2026-10-19)
Source: `results/reproduction/{motor12V,motor18V}/` and `summary.csv`. 180 runs in total, now with
plants motor12V and motor18V added. The paper's gains are unchanged, and nothing was tuned.

**S3 shock: prediction vs result.** The shock-induced error is measured as max over t ∈ [12, 20] s of
|e_S3(t) − e_S2(t)|, i.e. the same plant and controller with and without the shock. This removes the
pre-existing tracking error.

| Plant | CKC+PI | RSMKC+PI | CSMDC | AFNTSMDC | Predicted (open loop / AFNTSMDC) | Paper MAX (Table 4) |
|---|---|---|---|---|---|---|
| ideal (a) | 94.5 cm | 104.2 cm | 164.4 cm | 16.3 cm | open-loop Δv ≈ 11 m/s | 2.8 / 2.7 / 2.0 / 1.6 cm |
| motor12V (c) | 16.4 cm | 17.8 cm | 19.4 cm | **6.1 cm** | 21 cm / ≈ 2–8 cm | same |
| motor18V (c) | 12.3 cm | 13.3 cm | 13.2 cm | **9.3 cm** | 15 cm / "somewhat larger" | same |

- **The prediction holds.**
  - The baselines barely react to the 0.5 s shock, so their x_e increments sit near the open-loop
    bound (12–19 cm vs 15–21 cm).
  - AFNTSMDC's increment, 6.1 cm, lies inside the predicted 2–8 cm for 12 V, and 18 V is larger (9.3 cm).
  - Δv during the shock: AFNTSMDC 0.33 / 0.34 m/s vs open-loop 0.65 / 0.46 m/s.
- **Scale:** plant (c) brings the shock effect from 10–60× the paper's down to **4–7×**. AFNTSMDC
  still has the smallest effect of the four, as in the paper. Not at the paper's scale.

**S1/S2 tracking on plant (c) is much worse than the paper's.**
- With the paper's gains, no controller except CKC+PI settles within 20 s.
- AFNTSMDC: x_e RMS 7.4 cm and y_e RMS 9.7 cm (12 V); 12.4 / 11.8 cm (18 V). Paper: 1.9 / 3.2 cm.
- CSMDC: ≈ 46–55 cm. RSMKC+PI: ≈ 25–29 cm.
- **Cause (from the model):** back-EMF is a velocity-proportional term c·v/m0 ≈ 318 × 0.42/4.5 ≈ 30 m/s²
  (12 V), which the model-based laws (designed for M0ż = Bτ) treat as δ. It is far above the paper's
  assumed bound at its initial gains (ĉ₁ + ĉ₃|v| ≈ 17 m/s²). ĉ₁ adapts upward but only slowly
  (ĉ₁ 14 → 17.5 in 20 s).
- Steady wheel inputs are now nonzero: AFNTSMDC holds ≈ 4.3 / 2.0 V (outer/inner) before the shock.

**Paper vs our two plants.** Paper value first; ideal-plant (a) value second; plant (c) 12 V / 18 V
value third.

| Criterion | Paper | Torque plant (a) | Plant (c) | More consistent |
|---|---|---|---|---|
| AFNTSMDC S1 tracking | settles x_e 0.68 s, RMS 1.9 cm | settles 0.40 s, RMS 0.8 cm | never settles, RMS 7.4 / 12.4 cm | **(a)** |
| CKC+PI x_e settling (S1) | 6.07 s | 2.24 s (2.7× too fast) | 17.2 / 19.2 s (2.8–3.2× too slow) | **neither**; the paper lies between |
| S2 payload degrades AFNTSMDC | x_e RMS 1.92 → 2.81 cm | 0.82 → 0.76 cm (no) | 7.43 → 7.44 cm (no) | **neither** |
| S3 shock effect (AFNTSMDC / baselines) | 1.6 / 2.0–2.8 cm | 16 / 94–164 cm | 6–9 / 12–19 cm | **(c)** (4–7× vs 10–60×) |
| Steady wheel inputs (Fig. 6) | ≈ −1.5 / −0.5 V, outer > inner | ≈ 0 | ≈ 4.3 / 2.0 V, outer > inner | **(c)** structurally; magnitude ≈ 3×, opposite sign (likely a motor polarity convention) |
| Input chattering, AFNTSMDC (S1) | "persistently smooth", small residual | index 839 | 2.4 / 0.3 | **(c)** (back-EMF damping filters the switching) |

**Reading.** Neither plant reproduces the paper.
- Plant (c), our datasheet motor in voltage mode, gets the *disturbance scale*, *steady inputs* and
  *smoothness* closer to the paper. But it makes the paper's own gains track far worse than the paper
  reports.
- The torque plant (a) gets the *tracking* headline but not the disturbance scale or input shape.
- That is consistent with the paper's hardware sitting between the two: much weaker effective
  back-EMF relative to the commanded effort than our RE 25 / 33:1 combination. Two explanations fit:
  - the ESCON was run in **current (torque) mode** with some mechanical damping (hypothesis 2 above), or
  - the motor, gear or wheel differ from our representative choice.
- The paper gives no information that would let us decide, and we did not tune either plant.

## Decisions
- **2026-10-19:** Phase 4 option A: the reproduced AFNTSMDC uses the sign-corrected switching term. The as-printed law is kept as a literal run (expected to diverge), and the general −D⁻¹LĈ fix is documented as an erratum-level correction (not reserved for Phase 6).
- **2026-10-19:** Architecture follows the paper. AFNTSMDC and CSMDC are combined controllers (pose + velocity errors → wheel inputs). CKC and RSMKC are kinematic outer loops with a PI inner loop. CLAUDE.md was updated to match.
