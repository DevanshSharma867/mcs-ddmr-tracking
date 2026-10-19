# Slide outline (14 slides)

Figures are in `results/report/`. Numbers come from `results/report/key_numbers.csv`. Claim IDs (Cn) are
in `docs/paper/claims.md`.

**1. Title.** "Reproducing, diagnosing and improving AFNTSMDC for a differential-drive robot"
*Figure: none (the robot schematic from our own plot, F1a inset optional).*
- Term project: reproduce a state-of-the-art controller, find its weaknesses, improve it, with evidence.
- Paper: Xie et al., *Nonlinear Dynamics* 2022. Its results are hardware; ours are a deterministic simulation.
- Story: reproduction → sign issue → chattering → robustness → pre-registered fix → limits.

**2. Problem and model.**
*Figure: F1a (error traces) as a teaser.*
- Nonholonomic constraint (no side-slip), unicycle kinematics, and reduced dynamics M·ż = B·τ + d.
- Tracking error in the robot frame, pₑ = R(φ)ᵀ(p_r − p), with ṗₑ = F + G·z.
- Goal: drive pₑ → 0 despite mass mismatch, disturbances and unknown actuation.

**3. The paper's method.**
*Figure: equations slide (ξ, s, τ_eq, τ_r, adaptive law).*
- ξ couples yₑ into the heading error, so a 2-D surface controls a 3-D error.
- Fast nonsingular terminal surface s = ξ̇ + αξ + β·sig(ξ)^γ, with 1 < γ < 2, so there is no singularity.
- τ = τ_eq + τ_r. An adaptive bound Ĉ on the switching term, with a boundary layer ϱ = 0.04.

**4. Reproduction set-up.**
*Figure: table of assumptions (Appendix A).*
- RK4 at 1 ms, controllers at the paper's Ts = 0.01 s with ZOH, fixed seeds, every figure regenerable.
- The paper's gains are untouched. Every missing value is ASSUMED and logged (input units, saturation,
  PI mapping, circle geometry).
- Four controllers: CKC+PI, RSMKC+PI, CSMDC, AFNTSMDC.

**5. Reproduction verdict.**
*Figure: F1.*
- Qualitative match: AFNTSMDC has the fastest xₑ (0.40 s vs paper 0.68 s) and the lowest xₑ RMS
  (0.82 vs 1.92 cm).
- CKC is best on yₑ and φₑ in both the paper and our runs. RSMKC's linear xₑ ramp is reproduced.
- Not quantitative: errors are 1.1–2.8× smaller, our CKC+PI is 2.7× faster, and the paper's start-up
  spikes are absent.

**6. Sign inconsistency in the printed law.**
*Figure: F2.*
- As printed, the law diverges (141 cm late error; ĉ₄ goes 6 → 113).
- HG = −I at the origin, so the printed term adds +sgn(s)·Ĉ to ṡ, pushing s away from zero.
- The paper's Lyapunov step implicitly assumes HG ≈ +I.

**7. Ruling out a misreading.**
*Figure: convention table (`docs/paper/sign_error_verification.md` §3).*
- A blind reviewer given only the PDF, sympy, and seven alternative readings simulated.
- Only readings that contradict a printed equation are stable, and all equal the sign-corrected law.
- Framing: the printed equations are mutually inconsistent. Either a sign slip, or a derivation under the
  p − p_r convention.

**8. Discrete-time chattering.**
*Figure: F3.*
- At Ts = 0.01 s, a sampled sgn quasi-slides in a band ≈ K·Ts around an off-centre mean, giving a steady
  offset (CSMDC −0.73 cm).
- A boundary layer only works if K·Ts/ϱ < 2. The paper's ϱ = 0.04 gives ≈ 3.5, so it chatters (index 839).
- At Ts = 1 ms the same CSMDC settles, so sampling is the cause.

**9. Two plant models.**
*Figure: F4.*
- Plant (c) is a voltage-input DC motor with back-EMF (maxon RE 25 datasheet values).
- The S3 shock moves from 16 cm (torque plant) to 6–9 cm (motor), as predicted before the run. The
  paper's value is 1.6 cm.
- Neither plant reproduces the paper. The paper's hardware sits between them.

**10. Robustness weaknesses.**
*Figure: F5.*
- No integral action: constant push or friction leaves 1.3–1.8 cm (CKC+PI 0.24 cm).
- Strength: periodic and broadband rejection (0.8–1.0 cm vs 2.9–7.4 cm for the PI baselines).
- The adaptive gains never level off (0.02–0.06 1/s; two mechanisms).

**11. Pre-registered improvement.**
*Figure: block diagram (AFNTSMDC + ESO + projection + sized layer).*
- The ESO cancels the lumped disturbance (integral-like action). Projection bounds ĉ.
- The boundary layer is sized from Ts: κ_p·(K₁ + Ĉ_max/ϱ)·Ts ≤ 1, so convergence is monotone with no
  sampled chattering.
- Design, parameters and criteria were committed before any run (3e595fd → ca41546).

**12. Results and ablation.**
*Figure: F6.*
- All 28 checks of P1–P7 pass: offsets 0.14–0.16 cm, chattering 32 vs 837, motor plant 0.12 cm.
- The ESO fixes the structured offsets; projection + layer fix the chattering. They are complementary.
- Negative result: once the ESO is in, the adaptive switching term is largely redundant (A4 ≈ A3).

**13. Limitations.**
*Figure: F7.*
- 40 ms lag breaks the ESO arms (8.3 cm). A lag-aware ESO restores 0.19 cm, confirming the cause.
- On the 18 V motor the ESO arms saturate and lose the path. The non-ESO controllers stay bounded, so the
  plant is feasible.
- ĉ is bounded but sits at its upper bound 32–63 % of the time. Everything here is simulation.

**14. Conclusion and future work.**
*Figure: summary table (contribution → evidence).*
- Reproduction with an assumption registry. A verified sign inconsistency. A chattering diagnosis.
- A robustness study, and a pre-registered fix with an honest ablation.
- Next: hardware, a lag-aware ESO, saturation-aware compensation, and a confirmatory test of A4.
