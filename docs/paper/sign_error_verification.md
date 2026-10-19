# Verification: sign of the adaptive switching term in AFNTSMDC (paper eq. 32)

Paper: H. Xie, J. Zheng, Z. Sun, H. Wang, R. Chai, *Nonlinear Dynamics* 110(2):1437–1453, 2022,
doi:10.1007/s11071-022-07682-2. Equations are cited by the paper's numbers; only short equation forms are
quoted (rule 6).

Regenerate: `python -m experiments.sign_verification` → `results/paper/sign_verification_symbolic.txt`,
`results/paper/sign_verification_sim.{csv,md}`.

**Evidence tier: exploratory.** The issue was found in Phase 4 after observing divergence, and this
verification was done afterwards. The *analytic* part is a derivation from the paper's own definitions
(strength: proven, conditional on reading the printed equations as written).

## Question
With the law exactly as printed, does the term −B⁻¹M0·L·Ĉ in (32) drive the sliding variable s toward
or away from zero near the origin? And could the printed law be correct under some reading of the paper
that we missed?

## 1. Derivation (from the paper's definitions)
- Write (30) as `ṡ = A + HGM0⁻¹B·τ − HGM0⁻¹δ`, where A collects the τ-free terms.
- (31) gives `HGM0⁻¹B·τ_eq = −A`.
- (32) gives `HGM0⁻¹B·τ_r = −K1·s − K2·sig(s)^μ − HG·(M0⁻¹B)(B⁻¹M0)·L·Ĉ = −K1·s − K2·sig(s)^μ − HG·L·Ĉ`.
- Hence `ṡ = −K1·s − K2·sig(s)^μ − HG·L·Ĉ − HG·M0⁻¹δ`. The switching term agrees with the paper's (42).
  For the δ term, (42) has +HG·M0⁻¹δ. This traces to a sign inconsistency between (9) and (11), and it is
  immaterial because δ only enters through absolute-value bounds.
- From (19) and (24):
  `HG = [[−1, y_e], [−h21, h21·y_e − h22·x_e − 1]]`, with h21 = −ρ(|x_e|+ρ)⁻²·sgn(x_e)·atan(y_e) and
  h22 = ρ/((|x_e|+ρ)(1+y_e²)).
  - At the origin, h21 = 0 and h22 = 1, so **HG = −I**.
  - (HG)₁₁ = −1 at every state.
- So near the origin, `−HG·L·Ĉ = +L·Ĉ`, i.e. `s_i·(term)_i = +|s_i|·Ĉ_i ≥ 0` (L = diag(sgn s) by (33), and
  Ĉ ≥ 0 because ĉ_i(0) ≥ 0 and (37)–(38) are non-decreasing). **The printed term drives s away from zero.**
  The adaptive law then increases Ĉ whenever s ≠ 0.
- **The paper's Lyapunov step.** It passes from `−sᵀHG(LĈ − M0⁻¹δ)` to `≤ −⌊s⌋ᵀ⌊HG⌋(Ĉ − ⌊M0⁻¹δ⌋)`. For the
  Ĉ part that requires `sᵀHG·L·Ĉ ≥ ⌊s⌋ᵀ⌊HG⌋Ĉ`, which holds only when HG is diagonal with non-negative
  entries (≈ +I). With HG ≈ −I, the left side is −Σ|s_i|Ĉ_i, the opposite extreme.

**Independent blind check.** A separate reviewer was given only the PDF and the question above. It had no
access to our notes, our code, or our conclusion. It derived the same ṡ and HG = −I at the origin, and
concluded that "the switching term −B⁻¹M0·L·Ĉ drives s **away from zero**", that "the paper's inequality
step … is invalid there", with confidence ≈ 95 %. It also independently noted the δ-sign inconsistency
between (9) and (11) and the 1/r vs 1/r² factor in (52).

## 2. Symbolic check (sympy), `results/paper/sign_verification_symbolic.txt`
- Differentiating (14) with the kinematics (4) and (15): `ṗ_e − (F + G·z) = 0`, **with G exactly as
  printed in (19)**. This pins the sign of G.
- H in (24) equals the Jacobian of ξ (22) with respect to p_e, for both x_e > 0 and x_e < 0.
- **HG at the origin = [[−1, 0], [0, −1]] = −I.**
- The switching contribution to ṡ at the origin is `−HG·L·Ĉ = [Ĉ₁·L₁, Ĉ₂·L₂]`, i.e. away from zero.
- Under the reversed convention e = p − p_r, consistently re-derived, G_rev = −G and H·G_rev = +I at the
  origin. That is the one convention under which printed (32) would be stabilising (see §3).
- `det(HGM0⁻¹B) = −2b(…)/(m0·I0·r²·…)`. Printed (52) has r instead of r². This is a minor typo, separate
  from the sign question, and irrelevant to it (a determinant cannot distinguish −I from +I).

## 3. Conventions that could flip the sign
Simulations: ideal plant, paper gains, no noise. S1 = initial offset, S2 = payload, plus S1 with 20 ms
actuator lag. "Stable" means late (t ≥ 15 s) position error < 2 cm.
Source: `results/paper/sign_verification_sim.md`.

| # | Alternative convention / reading | Pinned down by the paper? | Late error S1 / S2 / S1 + lag [cm] | Printed law stable? |
|---|---|---|---|---|
| 0 | **As printed** | — | 141 / 331 / 265 (ĉ₄ ×19) | **no** |
| 1 | Error e = p − p_r, with F, G re-derived consistently | **Yes, against it.** (14) prints (p_r − p). (19)'s G is exactly the one derived from (14) (§2). Fig. 5c,d start at x_e = y_e = +0.1 for the stated offset (−0.1, −0.1, 0), which is p_r − p. | 0.49 / 0.25 / 0.55 | yes (≡ sign-corrected law) |
| 1b | Error e = p − p_r, printed F, G kept | Not a consistent reading (it contradicts (18)). | 23,948 / 21,880 / 13,623 | no |
| 2 | Rows of ξ swapped | **Yes:** (22) prints ξ = [x_e, φ_e + …]ᵀ explicitly. | 11,201 / 6,221 / 10,389 | no |
| 3 | Wheel order in B (τ₁ = left) | **Yes:** τ₁ and τ₂ are "right and left wheel" (Sect. 2), and B = (1/r)[[1, 1], [b, −b]] matches right = +b. A consistent relabelling of controller and plant changes nothing. The row shows a controller/plant *mismatch*. | 29.8 / 351,982 / 27.0 | no |
| 4 | Sign of δ in (9) vs (11) | The printed equations disagree. **Immaterial:** the law (31)–(39) contains no δ, and δ enters the analysis only through absolute-value bounds. | — (not a law variant) | n/a |
| 5 | Sign of G (ṗ_e = F − G'z with G' = −G used in the law) | **Yes:** (18) prints ṗ_e = F + Gz, and (21), (30) and (31) consistently use +G. | 72,665 / 24,152 / 25,367 | no |
| 6 | An implicit sign in L / sat, i.e. L = −diag(sgn s) | **Yes, against it:** (33) prints L = diag([sgn(s₁), sgn(s₂)]), and (55) sat is sign-preserving. | 0.49 / 0.25 / 0.55 | yes (≡ sign-corrected law) |
| 7 | Switching term inside (HGM0⁻¹B)⁻¹ | **Yes, against it:** (32) typesets −B⁻¹M0·L·Ĉ outside the bracket, after the closing parenthesis. | 0.49 / 0.25 / 0.56 | yes (≈ sign-corrected law) |
| 8 | The Lyapunov step implicitly assumes HG ≈ +I | This is not a convention but the analysis the paper states. It is consistent with reading 1, and inconsistent with (14) and (19) as printed. | — | — |

**Every reading that keeps all the printed equations makes the printed law unstable** (rows 0, 1b, 2, 3,
5). The three readings that make it stable (rows 1, 6, 7) each require departing from at least one printed
equation or figure. All three are numerically identical to our sign-corrected law (late error 0.49 cm in
S1, the same as `afntsmdc`).

## 4. Erratum / follow-up search (2026-10-19)
- **Web searches** found no erratum, corrigendum, correction or published comment:
  - the exact article title with erratum OR correction OR corrigendum
  - author names + AFNTSMDC + DOI, 2023–2024 follow-ups

  The results contain only the original article (Springer, ResearchGate) and later papers that cite it.
  One of them is a 2024 *Proc. IMechE Part I* paper (doi:10.1177/09596518241233319), which cites it; per
  the search snippet it does so approvingly. We did not read its full text.
- **The Springer article page** (link.springer.com/article/10.1007/s11071-022-07682-2) could not be
  fetched automatically (it redirects to an authentication page). **Not verified:** whether the page
  carries a correction notice. This needs a manual check in a browser before any public claim.
- No later paper by the same group discussing or correcting this term was found. The authors' earlier
  related paper (*Comput. Electr. Eng.* 96, 2021) is closed access and was not checked.

## Verdict: **confirmed**, as an internal inconsistency of the printed equations
Under the paper's own printed definitions (14), (18)–(19), (22), (24), (30)–(33), confirmed symbolically
and by an independent blind derivation:
- HG = −I at the origin;
- the printed switching term −B⁻¹M0·L·Ĉ drives s away from zero;
- the printed law diverges in simulation under every reading that keeps the printed equations.

No self-consistent reading of the printed paper (equations and figures together) makes the printed law
stable.

**Most likely explanations** (we cannot distinguish them, and the paper's hardware results suggest the
authors' implementation behaved correctly):
1. **A sign slip in (32)**, e.g. a typesetting error: the stable form needs +B⁻¹M0·L·Ĉ, or equivalently
   −(HGM0⁻¹B)⁻¹·L·Ĉ.
2. **A different error convention in the derivation of (32) and the Lyapunov step.** With e = p − p_r,
   one gets HG ≈ +I, (32) as printed is stabilising, and the paper's inequality step holds near the
   origin. Meanwhile (14) and (19) are printed with p_r − p.

   This reading is supported by the fact that the paper's Lyapunov step implicitly treats HG as positive.

Both explanations lead to the same corrected law, the one used in our reproduction.

**Suggested wording for the paper and report** (rule 5):
> "We find that, under the error convention of Eq. (14) and the matrix G of Eq. (19), the printed sign of
> the adaptive switching term in Eq. (32) makes ṡ contain +sgn(s)·Ĉ near the origin, which is
> destabilising; the law is stable with the opposite sign. This is consistent with a sign slip in Eq. (32)
> or with the switching term having been derived under the opposite error convention (e = p − p_r)."

**Caveats**
- The Springer page has not been checked manually for a correction notice.
- The claim is about the *printed* equations, not the authors' hardware implementation.
- The blind reviewer was a model instance, not an independent human expert.
