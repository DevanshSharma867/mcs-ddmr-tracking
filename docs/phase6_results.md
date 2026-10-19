# Phase 6 results: improved AFNTSMDC (ESO + projection + sized boundary layer)

Pre-registration: `docs/phase6_preregistration.md` (commit 15a2d05, made before any run).
Regenerate everything with:
- `python -m experiments.robustness`
- `python -m experiments.robustness --config configs/phase6_omega_sweep.yaml`
- `python -m experiments.drift`
- `python -m experiments.reproduction`
- `python -m experiments.phase6_eval`

The criteria table is generated in `results/phase6/criteria.md`, and the ablation tables and figures in
`results/phase6/ablation_*`.

Runs:
- grid: 3,240 (the 1,800 Phase 5 runs re-run identically, plus 1,440 new), 0 diverged
- reproduction: 252
- drift: 144 × 200 s
- ω_o sweep: 180

The A0 values in the new grid equal the Phase 5 values exactly (e.g. nominal circle 0.800 cm), so the
comparison is on identical conditions.

## Pre-registered criteria: all seven pass

| | Criterion | Result (A3 = full design) | Verdict |
|---|---|---|---|
| P1 | Offsets ≤ 0.5 cm (const, yaw, friction 10/30 %; circle and figure-8) | 0.14–0.16 cm (A0: 1.13–1.77 cm) | **PASS** (8/8) |
| P2 | Disturbance rejection not significantly worse than A0 | sine 1 Hz / 0.2 Hz / band-limited: 0.22–0.38 cm vs A0 0.67–1.01 cm (better, not merely "not worse") | **PASS** (6/6) |
| P3a | Chattering at nominal ≤ 100 | 32.2 (circle), 31.5 (figure-8); A0 837 / 781 | **PASS** |
| P3b | mass ×0.7 and noise ×2 ≤ 2× own nominal | 35.8 / 56.8 (circle), 35.0 / 55.4 (figure-8), vs limit 64.4 / 63.0 | **PASS** (4/4) |
| P4a | S1 ideal x_e settling ≤ 1.36 s | 0.35 s | **PASS** |
| P4b | Grid nominal late error ≤ A0 | 0.156 vs 0.800 cm (circle), 0.144 vs 0.762 cm (figure-8) | **PASS** |
| P5 | motor12V: circle ≤ 2.0 cm; figure-8 ≤ A0 | 0.124 cm; 0.343 vs 6.06 cm | **PASS** |
| P5 (report) | A3 vs CKC+PI on motor12V | circle 0.124 ± 0.023 vs 1.331 ± 0.071 cm; figure-8 0.343 ± 0.031 vs 13.73 ± 0.12 cm. **A3 significantly better on both** | — |
| P6 | S3 shock-induced x_e ≤ A0 | ideal 8.73 vs 16.25 cm; motor12V 5.68 vs 6.09 cm | **PASS** |
| P7 | ĉ stays in bounds | 0 violations in 720 A2/A3 grid runs and in all A3 drift ticks | **PASS** |
| P7 (report) | Time at ĉ_max (200 s drift, A3, noisy) | ĉ₁ 32 %, ĉ₂ 35 %, ĉ₃ 32 %, ĉ₄ 63 % | — |

As anticipated, the projection bounds ĉ but does not stop it drifting *within* the bounds: ĉ sits at
its upper bound for a third to two thirds of the time.

## Ablation (late error [cm], mean of 5 seeds; `results/phase6/ablation_*.md`)
- **A1 (+ESO only).**
  - Removes the *disturbance-induced* part of the offsets on plant (a): constant push 1.29 → 0.81 cm,
    friction 30 % 1.77 → 0.78 cm.
  - Removes the back-EMF error on motor12V: circle 11.7 → 0.18 cm, figure-8 6.06 → 0.20 cm.
  - But the quasi-sliding residual (≈ 0.8 cm) and the chattering (≈ 840) remain on plant (a).
  - On motor12V, chattering *rises* from 31 to 632: once the ESO cancels the back-EMF, s reaches the
    thin ϱ = 0.04 layer and quasi-slides.
- **A2 (projection + sized layer only).**
  - Fixes chattering (≈ 30) and the residual (nominal 0.15 cm).
  - It is best of all arms under 40 ms lag (0.18 cm).
  - As pre-stated, it **fails under structured disturbance**: friction 30 % 4.07 cm (worse than A0),
    constant push 1.23 cm, motor12V circle 16.9 cm.
- **A3 (full).** The components are complementary: the ESO handles structured disturbance and PS handles
  the discrete switching. A3 is the best or tied-best arm everywhere except actuator lag (see below).
- **A4 (ESO, no switching term), the Occam check.**
  - **On plant (a), A4 ≈ A3**: no significant difference in any condition except 40 ms lag, where A4 is
    better (2.31 vs 8.34 cm). A4 also has ≈ 30 % less chattering.
  - On motor12V figure-8, **A3 is significantly better in every condition** (e.g. nominal 0.34 vs
    0.52 cm; friction 30 % 0.42 vs 0.68 cm).
  - **Finding: once the ESO is present, the adaptive switching term adds little on the torque plant.**
    It helps only where the ESO residual is large (input-gain mismatch on the motor plant, fast
    trajectory changes).

## Unregistered findings: regressions not covered by P1–P7 (reported, not fixed)
1. **Actuator lag of 40 ms breaks the ESO arms.**
   - Late error at lag_40ms: A3 8.34 / 8.46 cm (circle / figure-8), A1 9.03 / 8.50, A4 2.31 / 2.50,
     against A0 0.92 / 0.93, A2 0.18 / 0.17, CKC+PI 0.26 / 0.28.
   - At lag_20ms all arms are fine (A3 0.18 cm).
   - The reproduction run confirms it: S1 with 40 ms lag gives A3 8.36 cm, while A0 gives 0.70.
   - Likely cause: the ESO's model assumes the input acts immediately (w = M0⁻¹B·u). An unmodelled
     40 ms lag adds phase delay inside the observer–compensation loop at ω_o = 25 rad/s, which is the
     classic sensitivity of ADRC to input delay. This is not yet verified by a dedicated test.
2. **The motor18V plant (input gain ≈ 0.32× the model, near voltage saturation) makes every ESO arm
   lose the path.**
   - S1 late error: A1 312 cm, A3 333 cm, A4 335 cm, against A0 16.9 cm and A2 23 cm.
   - Diagnosis from the logs: the commands saturate on 98 % of ticks. The ESO correctly estimates
     f̂ ≈ −70 to −79 m/s² of back-EMF, and cancelling it needs more voltage than the 8 V limit
     allows (the reference alone needs 7.1 V on the outer wheel).
   - With the input pinned at the limit, the loop loses authority. The robot ends up driving
     backwards at the voltage limit (v = −0.62 m/s), with f̂ stuck at +84.7.
   - motor18V was not part of the grid or the criteria. It shows that the full design has **no
     saturation-aware behaviour**: feeding the saturated command into the ESO prevents observer
     windup, but not loss of control authority.

These two failure modes bound the claim. The improvement holds on the pre-registered grid
(torque plant, motor12V, lag ≤ 20 ms), but **not under large input delay or under saturation with a
large input-gain deficit**. Fixing them, e.g. a delay-aware or lower-bandwidth ESO, or a
saturation-aware compensation limit, would be a new design iteration needing its own
pre-registration.

## ESO bandwidth sweep (descriptive only; ω_o = 25 was fixed beforehand)
`results/phase6/omega_sweep.md`. A3 with ω_o ∈ {12.5, 25, 50}:
- **Chattering is essentially independent of ω_o** (51–58 at 2× noise). This confirms the pre-run
  analysis that the switching layer, not the ESO, is the dominant noise path.
- **Band-limited disturbance rejection improves with ω_o:**
  - torque circle: 0.43 / 0.38 / 0.29 cm
  - motor12V figure-8: 0.69 / 0.42 / 0.28 cm
- **Nominal and noise ×2** results change little on plant (a).
- The lag finding above suggests that a higher ω_o would make the lag sensitivity worse. This was
  not tested.

## Stability: what is claimed
- **Proven (continuous time, under A1–A4 of the plan):**
  - ĉ is bounded.
  - The ESO error is ultimately bounded.
  - s reaches and stays in an O(ϱ·|f̃|/Ĉ) neighbourhood.
  - p_e is ultimately bounded.
  - The switching term enters ṡ as exactly −L·Ĉ, so the paper's Lyapunov step is valid for any HG.
- **Proven for the sampled s-recursion only:** monotone (no sign-flip) convergence inside the layer
  for plant/model gain ratios up to κ_p = 1.5.
- **Not proven:**
  - finite-time convergence
  - discrete-time stability of the full sampled loop
  - robustness to input delay or to large input-gain deficits under saturation. The two regressions
    above are counter-examples to any such claim.

## Credit and contribution
**Prior work used (bibliographic details verified 2026-10-19):**
- J. Han, "From PID to active disturbance rejection control," *IEEE Trans. Ind. Electron.*, vol. 56,
  no. 3, pp. 900–906, 2009. doi:10.1109/TIE.2008.2011621. *(ESO / ADRC)*
- Z. Gao, "Scaling and bandwidth-parameterization based controller tuning," *Proc. American Control
  Conference*, vol. 6, 2003, doi:10.1109/ACC.2003.1242516. Pages 4989–4996 come from a secondary
  reference list and were not confirmed on the proceedings page. *(bandwidth parameterization
  β1 = 2ω_o, β2 = ω_o²)*
- B.-Z. Guo and Z.-L. Zhao, "On the convergence of an extended state observer for nonlinear systems
  with uncertainty," *Systems & Control Letters*, vol. 60, no. 6, pp. 420–430, 2011. *(ESO error bound)*
- W. Gao, Y. Wang and A. Homaifa, "Discrete-time variable structure control systems," *IEEE Trans.
  Ind. Electron.*, vol. 42, no. 2, pp. 117–122, 1995. doi:10.1109/41.370376. *(discrete / quasi-sliding)*
- M. Cui, W. Liu, H. Liu, H. Jiang and Z. Wang, "Extended state observer-based adaptive sliding mode
  control of differential-driving mobile robot with uncertainties," *Nonlinear Dynamics*, vol. 83,
  no. 1–2, pp. 667–683, 2016. doi:10.1007/s11071-015-2355-z. *(ESO + adaptive SMC for a DDMR already
  exists; the primary paper cites it as [17])*
- P. A. Ioannou and J. Sun, *Robust Adaptive Control*, Prentice Hall, 1996 *(projection)*. J.-J. E. Slotine
  and W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991 *(boundary layer)*. These are textbooks;
  no volume or page numbers.

**Our contribution (honest).** Not new theory:
1. **Diagnosis of the reproduced AFNTSMDC.**
   - the sign error in (32)
   - sampled chattering at Ts = 0.01 s with ϱ = 0.04
   - adaptive-gain drift
   - no integral action against structured disturbance
2. **A Ts-aware sizing rule** linking Ĉ_max, ϱ, K1 and Ts through the monotone (≤ 1) discrete condition
   with a plant-gain margin, applied to this controller together with the general −D⁻¹LĈ form.
3. **A pre-registered ablation** on a fixed grid, showing which component fixes which weakness,
   including the Occam result (the switching term is largely redundant once the ESO is in) and two
   failure modes of the improved design.
