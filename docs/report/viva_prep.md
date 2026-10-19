# Viva preparation: 20 likely questions

Numbers come from `results/report/key_numbers.csv`. Claim IDs (Cn) are in `docs/paper/claims.md`.

**1. What is the nonholonomic constraint here, and why does it matter?**
- The wheels cannot slip sideways: ẋ sin φ − ẏ cos φ = 0. This restricts velocities but not reachable
  poses.
- By Brockett's condition, no smooth time-invariant state feedback stabilises a fixed pose.
- Tracking a moving reference with v_r > 0 avoids this, which is why the paper and we study tracking.

**2. Why express the error in the robot frame?**
- pₑ = R(φ)ᵀ(p_r − p) makes the error dynamics ṗₑ = F + G·z depend on the reference only through
  (v_r, ω_r).
- It separates longitudinal (xₑ), lateral (yₑ) and heading (φₑ) errors. This is the Kanayama
  formulation [2].

**3. Why does the paper use ξ = [xₑ, φₑ + ρ/(|xₑ|+ρ)·atan yₑ]?**
- There are only two inputs for three errors. ξ couples yₑ into the heading channel.
- On ξ = 0, φₑ = −atan(yₑ), so the kinematics give ẏₑ = −v_r·yₑ/√(1+yₑ²), and yₑ → 0 if v_r > 0.
- The weight ρ/(|xₑ|+ρ) keeps D = HGM0⁻¹B invertible for 0 < ρ < 1.

**4. Why does terminal SMC claim finite-time convergence?**
- With an exponent below 1 (e.g. ξ̇ = −β·sig(ξ)^a with 0 < a < 1), the Lyapunov derivative satisfies
  V̇ ≤ −cV^(a') with a' < 1. That gives a finite settling time t ≤ V(0)^(1−a')/(c(1−a')).
- Linear sliding (a = 1) only converges exponentially.

**5. Then why do we not claim finite time?**
- The paper's surface uses 1 < γ < 2 to be *nonsingular*. Near ξ = 0 the fractional term is then weaker
  than the linear one, so convergence is exponential there.
- yₑ converges asymptotically: its rate is ∝ |yₑ| → 0.
- With a dead zone and a boundary layer, only practical (ultimately bounded) convergence holds.
- Our improved design claims ultimate boundedness, not finite time (Appendix C).

**6. What is the singularity issue?**
- Terminal surfaces with exponents below 1 have derivatives containing |ξ|^(a−1), which is unbounded at
  ξ = 0 and appears in the control law.
- With γ > 1, the derivative contains |ξ|^(γ−1), which is bounded.
- Separately, the law must invert D = HGM0⁻¹B. Its determinant is ∝ 1 + ρxₑ/((|xₑ|+ρ)(1+yₑ²)), which is
  never zero for 0 < ρ < 1. (The paper prints the prefactor with r instead of r²; immaterial.)

**7. What causes chattering here?**
- In continuous time an ideal sgn would slide. With sampling at Ts = 0.01 s and zero-order hold, s jumps
  by ≈ K·Ts per sample and alternates in a period-2 cycle.
- The band is ≈ 0.21 for CSMDC, centred off zero, which gives a steady offset (CSMDC xₑ = −0.73 cm).
- At Ts = 1 ms the same law settles (C9, C10).

**8. Why does a boundary layer not always fix it?**
- Inside the layer the switching is linear with gain K/ϱ. The sampled loop is stable only if K·Ts/ϱ < 2.
- The paper's ϱ = 0.04 with ĉ ≈ 14 gives ≈ 3.5, still chattering (index 839).
- CSMDC's chattering drops from about 800 to 0.45 only once ϱ = 0.15 (k·Ts/ϱ = 1.4) (C11, C12).

**9. ≤ 1 vs < 2: why the stricter condition?**
- For s_{k+1} = (1 − g·Ts)·s_k, g·Ts < 2 gives stability but allows 1 − g·Ts < 0, i.e. sign flipping
  every sample, which *is* sampled chattering.
- g·Ts ≤ 1 gives monotone convergence with no sign flips.
- The margin κ_p = 1.5 covers plant/model gain ratios up to 1.5. The mass ×0.7 case needs 1.43; it broke
  ϱ = 0.15 in the grid (C23).

**10. How does the adaptive law work, and what goes wrong?**
- ĉ̇ᵢ ∝ |sᵢ| (and |v|, |ω| for ĉ₃, ĉ₄): the bound grows while s ≠ 0 and freezes inside the dead zone
  |s| ≤ 0.05.
- The gains can only grow. The chattering band (≈ ±0.2) is wider than the dead zone, and noise keeps
  |s| > ε.
- So ĉ drifts without bound: 0.02–0.06 1/s, a linear extrapolation of 174–239 after 1 h, from 14 (C19).

**11. Sketch the paper's Lyapunov argument.**
- V = ½sᵀs + ½Σ(ĉᵢ − cᵢ)². Substituting the law gives ṡ = −K₁s − K₂·sig(s)^μ − HG·L·Ĉ + HG·M0⁻¹δ.
- The paper bounds the switching part as if it were −|s|ᵀĈ, then concludes V̇ ≤ −√2·η·V^½, i.e. finite
  time.
- Gaps: that bound needs HG ≈ +I (it is −I); Lemma 1 is circular; and η depends on the state
  (Appendix C).

**12. What exactly is wrong with the printed switching term?**
- Substituting the printed −B⁻¹M0·L·Ĉ into ṡ gives −HG·L·Ĉ. With HG = −I at the origin, that is
  +sgn(s)·Ĉ, which is destabilising.
- The printed law diverges: 141 cm late error, ĉ₄ 6 → 113 (C4, C5).

**13. Why is flipping the sign a valid correction, rather than a hack?**
- With +B⁻¹M0·L·Ĉ, ṡ gets +HG·L·Ĉ ≈ −sgn(s)·Ĉ. That is exactly what the paper's own Lyapunov step
  assumes, so the correction makes the paper's argument valid near the origin.
- The general form −D⁻¹·L·Ĉ gives −L·Ĉ for *any* HG, and behaves within ≈ 2 % of the flip.
- Every stable reading of the paper we tested coincides with this corrected law (C6).

**14. Could you simply have misread the paper?**
- We ruled this out adversarially:
  - a blind reviewer with only the PDF;
  - sympy confirming that G follows from the printed error and that HG = −I;
  - seven alternative readings simulated, where only readings contradicting a printed equation or
    figure are stable.
- We present it as an internal inconsistency: a sign slip, or a derivation under e = p − p_r (C6, C7).

**15. Why two plant models?**
- The paper's input is a *voltage*, but its model is torque-input. Back-EMF and motor gain change the
  behaviour qualitatively.
- The torque plant reproduces the tracking headline. The DC-motor plant reproduces the disturbance scale
  and steady-input structure.
- Neither alone reproduces the paper. Comparing both tells us where the paper's hardware probably sits
  (C14–C16).

**16. Why is the reproduction only qualitative?**
- The paper's results are hardware measurements, with encoder odometry, noise, friction, motor dynamics
  and an unspecified driver mode.
- Many parameters are not given (motor, input units, PI mapping, saturation). We filled them with logged
  assumptions and did not tune to the paper's curves.
- So rankings and trends are comparable; absolute numbers are not.

**17. What does the improvement do, and why does it work?**
- The ESO estimates the lumped disturbance f, and −B⁻¹M0·f̂ cancels it. That gives integral-like action
  against constant, friction and back-EMF disturbances.
- The residual f − f̂ is small (∝ ḟ/ω_o), so a small Ĉ_max suffices. That lets ϱ satisfy the monotone
  discrete condition, which removes chattering.
- Result: offsets 0.14–0.16 cm (from 1.13–1.77 cm), chattering 32 (from 837), motor plant 0.12 cm
  (C20, C21).

**18. What does "pre-registered" mean here, and what is confirmatory?**
- The design, parameters, ablation arms and the success criteria P1–P7 were committed (3e595fd) before the
  implementation (ca41546) and any run. All 28 checks pass.
- Only P1–P7 are confirmatory. The diagnostics, A4 ≈ A3 and the lag/motor18V findings are exploratory.
- Earlier thresholds (Phases 3–5) were committed together with their results, so they are exploratory too.
  Two Phase 3 predictions are "pre-specified".

**19. What does A4 ≈ A3 mean?**
- A4 is the ESO with *no* adaptive switching term. On the torque plant it is statistically
  indistinguishable from the full A3, except under 40 ms lag, where it is better. It also chatters ≈ 30 %
  less.
- A3 helps only on the motor-plant figure-8.
- So once disturbances are estimated and cancelled, the adaptive switching term is largely redundant.
  This is a negative result, reported as such (C22).

**20. What are the limitations of your improvement?**
- **40 ms actuator lag** breaks the ESO arms (8.34 cm). Feeding the ESO the lag model restores 0.19 cm,
  so its instantaneous-actuation assumption is the cause.
- **18 V motor:** the ESO arms saturate (98–100 %) and lose the path, while all non-ESO controllers stay
  bounded (CKC+PI 4.3 cm). Saturation-aware compensation is needed.
- **Other limits:** ĉ sits at its bound 32–63 % of the time; there is no hardware; and ESO + adaptive SMC
  for this robot is prior art (Cui et al. 2016). Our contribution is the diagnosis and the evaluation.
