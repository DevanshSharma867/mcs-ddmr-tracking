# Reproduction log

Item IDs (M1…M15, D1) refer to `docs/paper_notes.md` §8. Decided 2026-10-19.

## Assumptions
<!-- One line per ASSUMED value: what, value, why. -->

| ID | ASSUMED | Value / rule | Why |
|---|---|---|---|
| M1 | Units of the control input | The controller output is the generalized input τ of (8)/(11), in "model units" (the paper's "V"). No motor model. Plots are labelled "control input (model units; paper: V)". | The paper gives no motor model (torque constant, gearing, driver mode). If τ is treated as the paper's input, the paper's gains can be used unchanged. |
| M2 | Input saturation | ±8 model units per wheel. It can be switched off in the config (`saturation: off`). | Fig. 6/8/10 inputs appear to clip near −8 V. The paper does not state a limit. |
| M3 | Shock disturbance | `6·|sin(2πt)|` model units added to **each** wheel's input channel for 12 ≤ t ≤ 12.5 s, else 0 (eq. 61). | Eq. (61) gives d1 = d2 with amplitude 6 V "acting on the wheels". In our units this is an input disturbance. |
| M7 | Sign of disturbance injection | The plant input to each wheel is τ_i + d_i. **Conflict noted:** (9) has `M ż = Bτ + d`, but (11) defines δ = ΔM ż + d and enters it as `−δ`. Taken literally, these disagree on the sign of d. | Adding the disturbance to the input is the physically consistent choice given M3. The controller only uses \|δ\| bounds, so the sign does not affect the design. |
| M4 | Measurement noise | None in the reproduction (Phases 3–4). Noise is added only in Phase 5. | The paper gives no noise levels (feedback was encoder odometry). Keeping the reproduction deterministic isolates the controller behaviour. |
| M5 | Initial velocity | z(0) = [v, ω] = [0, 0] (robot at rest). | Not stated. A robot at rest at the start of an experiment is the natural reading. |
| M6 | Settling time and window | Settling time is the **last** time the error is outside its band: \|x_e\| ≤ 1 cm, \|y_e\| ≤ 1 cm, \|φ_e\| ≤ 0.02 rad, each error separately. RMS and MAX are taken over the full 0–20 s. | The paper does not define the tolerance band or the RMS window. |
| M8 | Plant model | The reproduction plant is (8) as printed: `M ż = Bτ + d`, M = diag(m, I), h ignored. The full reduced model, with the derived terms `[m h ω², −m h v ω]` (h = 0.11 m), is kept as a config option for Phase 5 (unmodelled dynamics). | The paper designs for and presents (8). The h-terms appear when (1)–(2) are reduced but are dropped in the paper (paper_notes §1.5). |
| M9 | Boundary layer scope | AFNTSMDC: the discontinuous switching term L = diag(sgn(s1), sgn(s2)) in (32)–(33) uses sat(s_i) with ϱ = 0.04 (eq. 55). sig(s)^μ keeps its exact form, since it is already continuous for μ > 0. CSMDC and RSMKC use pure sgn. | Remark 3 applies the boundary layer only to the AFNTSMDC reaching law (32), where the only discontinuity is L. Nothing is said for the baselines. |
| M10 | Payload scenario (S2) | Same initial offset as S1, (−0.1, −0.1, 0). True mass m = 4.5 + 3.6 = 8.1 kg. True inertia I = 0.56 · 8.1/4.5 = 1.008 kg·m². The controller keeps the nominal m0 = 4.5, I0 = 0.56. | The paper says only "an initial posture offset". The printed ΔI = 0.01 kg·m² is not used: scaling the inertia with the mass is the user's decision, giving a more meaningful inertia mismatch. |
| M10 | Shock scenario (S3) | S3 = payload (S2 plant) + shock (M3). Table 4 metrics are computed over both 0–20 s and 12–20 s, and we report which window matches the paper better. | The RMS values in Tables 3 and 4 are almost identical, so S3 was very likely run with the payload. The paper does not state the window. |
| M11 | Heading error wrapping | φ_e is wrapped to (−π, π]. | The paper does not discuss wrapping. Wrapping is needed for long runs, where the heading grows past ±π. |
| M12 | PI inner loop (CKC, RSMKC) | The PI output u = [u1, u2] is the wheel input in model units: **u1 → right (τ1), u2 → left (τ2)**. The PI acts on z_e = z_d − z (eq. 59) with k_P = 30, k_I = 250. Clamping anti-windup at the saturation limit. Integrators start at 0. | The paper does not state the units of u, how u maps to the wheels, or any anti-windup. Fig. 3 feeds u straight into the robot dynamics. |
| M13 | Initial adaptive gain ĉ4(0) | ĉ4(0) = 6. | The paper prints "c2(0) = 6" a second time, which is clearly a typo for the 4th gain. |
| M14 | Numerical guards | sgn(0) = 0. Before any fractional power is taken, \|x\| is clipped to at least 1e-6 (in `sig^a` and `⌊ξ⌋^(γ−1)`). The guards are documented in code docstrings. | The paper describes no numerical treatment. The guards avoid 0^negative and NaN at x_e = 0 or s = 0. |
| M15 | Derivatives in τ_eq | Ḣ, Ḟ and Ġ are computed exactly from the model state (measured p and z) and the analytic reference (v̇_r, ω̇_r). | This is available in simulation. On hardware they would come from estimated velocities. |
| Ref | Reference circle | Centre (0, 0.6) m, radius 0.6 m, start (0, 0), φ_r(0) = 0, counter-clockwise, ω_r = 0.7 rad/s, v_r = 0.42 m/s, 20 s. | Radius and speed are stated (Sect. 4.3). Centre, start point and direction are read from Fig. 5a/b. |
| Ts | Sampling and integration | Controller Ts = 0.01 s with zero-order hold (the paper's value). The plant is integrated with fixed-step RK4 at dt = 1e-3 s. | Ts = 0.01 s is stated for all controllers (Sect. 4.2). The finer plant step keeps the integration error negligible. |
| D1 | Controller set | Kinematic-only ideal reference (perfect velocity tracking), CKC+PI, RSMKC+PI, CSMDC, AFNTSMDC, plus our improvement later. The generic PID/SMC inner-loop baselines from the original CLAUDE.md are dropped. | This matches the paper's comparison (Sect. 4.1), so the figures and tables can be compared directly. |

## Discrepancies vs. paper
<!-- What differs from the paper's figures/tables, by how much, and suspected cause. -->

Known in advance (structural):
- The paper's results are hardware measurements (odometry feedback, voltage inputs, friction, sensor quantisation). Our simulation is idealised, so agreement can only be qualitative (trends, rankings, orders of magnitude).
- The paper's own text and tables disagree on some settling times (paper_notes §6). We compare against the **tables**.

## Decisions
- **2026-10-19:** Architecture follows the paper. AFNTSMDC and CSMDC are combined controllers (pose + velocity errors → wheel inputs). CKC and RSMKC are kinematic outer loops with a PI inner loop. CLAUDE.md was updated to match.
