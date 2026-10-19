# Paper notes: AFNTSMDC

H. Xie, J. Zheng, Z. Sun, H. Wang, R. Chai, "Finite-time tracking control for nonholonomic wheeled
mobile robot using adaptive fast nonsingular terminal sliding mode", *Nonlinear Dynamics* 110(2),
1437–1453, 2022. doi:10.1007/s11071-022-07682-2. Open access (CC BY 4.0).

Source: `papers/primary.pdf` (17 pages; read in full, equations checked against the rendered pages).
**When this file and the paper disagree, the paper wins.**

Conventions in this file:
- "(n)" = the paper's equation number. "p. N" = journal page number.
- **[INFERRED]** = not stated in the text, read off a figure or derived by us. Must be confirmed or
  logged as an assumption.
- **[TYPO?]** = the printed paper looks internally inconsistent; our reading is given.
- Paper notation: `sgn(ψ)` element-wise sign; `sig(ψ)^a = [|ψ1|^a sgn(ψ1), …]ᵀ`; `diag(ψ)`;
  `⌊Ψ⌋` = **element-wise absolute value** (not floor).

> **Headline fact: the paper's results are hardware experiments, not simulations.** Everything in
> Sect. 4 was measured on a physical robot (NI myRIO, Maxon motors, encoder odometry) and the
> plotted control inputs are **motor voltages (V)**, not torques. Our reproduction is therefore a
> simulation of an experimental paper; agreement can only be qualitative. See MISSING §M1–M4.

---

## 1. Robot model

### 1.1 Geometry and symbols (Sect. 2, Fig. 1, Table 1)
| Symbol | Meaning | Unit |
|---|---|---|
| `p = [x_g, y_g, φ_g]ᵀ` | pose of the robot in the global frame {O_g X_g Y_g} | m, m, rad |
| `z = [v, ω]ᵀ` | forward and angular velocity | m/s, rad/s |
| `O_l` | midpoint of the driving-wheel axle (origin of the body frame) | – |
| `O_m` | mass centre; distance `O_m → O_l` is `h` | m |
| `r` | wheel radius | m |
| `b` | half the distance between driving wheels (Table 1 lists `2b`) | m |
| `m`, `I` | total mass (incl. load), moment of inertia | kg, kg·m² |
| `τ = [τ1, τ2]ᵀ` | **τ1 = right** wheel torque, **τ2 = left** wheel torque | N·m |
| `τ_d = [τ_d1, τ_d2]ᵀ` | lumped friction / external disturbance / unmodelled dynamics | N·m |

Driving wheels are at the front; two caster wheels at the rear (p. 1446).

### 1.2 Full Lagrangian model, (1)–(3)
```
M̄(p) p̈ + V̄(p,ṗ) ṗ + τ_d = B̄(p) τ − Aᵀ(p) λ                         (1)

M̄ = [[m, 0, m̄1], [0, m, −m̄2], [m̄1, −m̄2, I]]      m̄1 = m h sinφ_g, m̄2 = m h cosφ_g
V̄ = [[0, 0, m̄2 φ̇_g], [0, 0, m̄1 φ̇_g], [0, 0, 0]]
B̄ = (1/r) [[cosφ_g, cosφ_g], [sinφ_g, sinφ_g], [b, −b]]
Aᵀ = [−sinφ_g, cosφ_g, 0]ᵀ,   λ = −m (ẋ_g cosφ_g + ẏ_g sinφ_g) φ̇_g                (2)
A(p) ṗ = 0   (no lateral slip)                                                     (3)
```

### 1.3 Kinematics, (4)–(5)
```
ṗ = S(p) z,   S(p) = [[cosφ_g, 0], [sinφ_g, 0], [0, 1]]                          (4),(5)
i.e.  ẋ_g = v cosφ_g,  ẏ_g = v sinφ_g,  φ̇_g = ω
```

### 1.4 Reduced dynamics used for control, (6)–(11)
Substituting `p̈ = S ż + Ṡ z` (6) into (1) and premultiplying by Sᵀ (7):
```
M ż = B τ + d                                                                       (8)
M = diag(m, I),   B = (1/r) [[1, 1], [b, −b]],   d = [d1, d2]ᵀ = −Sᵀ(p) τ_d          (9)
```
i.e. per channel (SI): `m v̇ = (τ1 + τ2)/r + d1`, `I ω̇ = b (τ1 − τ2)/r + d2`.
This is the same structure as the starting form in CLAUDE.md.

Parametric uncertainty (10): `M = M0 + ΔM`, `M0 = diag(m0, I0)`, `ΔM = diag(Δm, ΔI)`.
Model used for design (11):
```
M0 ż = B τ − δ,     δ = [δ1, δ2]ᵀ = ΔM ż + d   ("reformatted"/lumped uncertainty)    (11)
```
Note the sign: in (9) `d` enters with `+`, and δ contains `+d` but enters (11) with `−`.
Taken literally, (8)–(11) are inconsistent in the sign of d (M0ż = Bτ + d − ΔMż would give
δ = ΔMż − d). Irrelevant for the controller (only |δ| is bounded) but matters for how we inject
disturbances in the plant — see M7.

**Assumption 1 (12)–(13)** — lumped uncertainty bound:
```
|m0⁻¹ δ1| < c1 + c3 |v|        (12)
|I0⁻¹ δ2| < c2 + c4 |ω|        (13)
```
`c_i > 0` unknown but bounded. Paper: c1, c3 bound external disturbance and surface friction;
c2, c4 bound modelling imprecision. Units: c1 [m/s²], c3 [1/s], c2 [rad/s²], c4 [1/s].

### 1.5 Remarks on the model **[INFERRED — our derivation]**
- Carrying out SᵀM̄S and Sᵀ(M̄Ṡ + V̄S)z with (2) gives `SᵀM̄S = diag(m, I)` exactly, but the
  velocity-product terms do **not** vanish when h ≠ 0: they are `[m h ω², −m h v ω]ᵀ` (with the
  signs of V̄ as printed). The paper's (8) silently drops them (or lumps them into d). h = 0.110 m
  is listed in Table 1 but is not used anywhere in the controller. → M8.
- `τ_d ∈ R²` in the text, but it is added to 3-vectors in (1) and premultiplied by `Sᵀ` (2×3) in (9).
  Dimensionally τ_d must be in R³ for (9) to make sense. **[TYPO?]**

---

## 2. Tracking error and kinematic (outer-loop) control

### 2.1 Reference, (15)–(17)
```
ṗ_r = S(p_r) z_r,   z_r = [v_r, ω_r]ᵀ                                               (15)
v_r = sqrt(ẋ_r² + ẏ_r²)                                                             (16)
ω_r = φ̇_r = (ẋ_r ÿ_r − ẍ_r ẏ_r) / (ẋ_r² + ẏ_r²)                                    (17)
```

### 2.2 Tracking error in the robot frame, (14) (Kanayama et al. [19])
```
p_e = [x_e, y_e, φ_e]ᵀ = [[cosφ_g, sinφ_g, 0], [−sinφ_g, cosφ_g, 0], [0, 0, 1]] (p_r − p)   (14)
```
Identical to CLAUDE.md's `e1 = x_e, e2 = y_e, e3 = φ_e`. The paper does not mention wrapping φ_e
(its Fig. 5b plots the unwrapped heading growing to ~14 rad, so wrapping is moot there) → M11.

### 2.3 Error dynamics, (18)–(21)
```
ṗ_e = F + G z                                                                        (18)
F = [v_r cosφ_e, v_r sinφ_e, ω_r]ᵀ,   G = [[−1, y_e], [0, −x_e], [0, −1]]             (19)
p̈_e = Ḟ + Ġ z + G ż                                                                (20)
p̈_e = Ḟ + Ġ z + G M0⁻¹ (B τ − δ)                                                   (21)
```

### 2.4 Kinematic controller
The proposed AFNTSMDC **has no separate kinematic outer loop** — it is a single "unified" law
mapping pose + velocity errors directly to wheel torques (Fig. 2, Remark 4). A Kanayama-type
outer loop appears only as the **CKC baseline**, eq. (56)–(57), given in §7.1 below.

> **Architectural mismatch with CLAUDE.md.** CLAUDE.md describes AFNTSM as an *inner loop* tracking
> `[v_c, ω_c]` from a Kanayama outer loop. In the paper, AFNTSMDC (and the CSMDC baseline) replace
> both loops. → decision D1 in §8.

---

## 3. AFNTSMDC design (Sect. 3.1)

### 3.1 Tracking-error function, (22)–(24)
```
ξ = [ξ1, ξ2]ᵀ = [ x_e ,  φ_e + (ρ / (|x_e| + ρ)) · atan(y_e) ]ᵀ,     0 < ρ < 1          (22)
ξ̇ = H ṗ_e                                                                            (23)
H = [[ 1, 0, 0 ],
     [ −ρ (|x_e| + ρ)⁻² sgn(x_e) atan(y_e),  ρ / ((|x_e| + ρ)(1 + y_e²)),  1 ]]          (24)
ξ̈ = Ḣ ṗ_e + H p̈_e                                                                   (25)
ξ̈ = Ḣ ṗ_e + H ( Ḟ + Ġ z + G M0⁻¹ (B τ − δ) )                                        (26)
```
`ρ/(|x_e|+ρ) ∈ (0, 1]` acts as a weight on how fast y_e and φ_e converge.
Idea: ξ → 0 forces x_e = 0 and φ_e = −atan(y_e), and that manifold drives y_e → 0 (see §4).

### 3.2 Sliding surface, (27)–(30)
```
s = ξ̇ + α ξ + β sig(ξ)^γ                                                             (27)
α = diag(α1, α2) > 0,   β = diag(β1, β2) > 0,   1 < γ < 2
ṡ = ξ̈ + α ξ̇ + β γ ⌊ξ⌋^(γ−1) ξ̇                                                     (29)
ṡ = Ḣ ṗ_e + H( Ḟ + Ġ z + G M0⁻¹ (B τ − δ) ) + α ξ̇ + β γ ⌊ξ⌋^(γ−1) ξ̇                (30)
```
Claimed finite-time bound on the surface (28): `t_ξ ≤ max{ η1⁻¹ |ξ1(0)|^½, η2⁻¹ |ξ2(0)|^½ }`
with `η = α⌊ξ⌋ + β⌊ξ⌋^γ` (Appendix B). "Nonsingular": because γ > 1, `|ξ|^(γ−1)` is bounded
at ξ = 0 so (29) has no 0^negative power — no ε-clipping is needed for this term.

### 3.3 Control law, (31)–(39)
Equivalent control (set δ = 0, solve ṡ = 0):
```
τ_eq = −(H G M0⁻¹ B)⁻¹ ( Ḣ ṗ_e + H(Ḟ + Ġ z) + α ξ̇ + β γ ⌊ξ⌋^(γ−1) ξ̇ )              (31)
```
Reaching control:
```
τ_r = −(H G M0⁻¹ B)⁻¹ ( K1 s + K2 sig(s)^μ ) − B⁻¹ M0 L Ĉ                              (32)
L  = diag( sgn(s1), sgn(s2) )                                                          (33)
Ĉ  = [ ĉ1 + ĉ3 |v| ,  ĉ2 + ĉ4 |ω| ]ᵀ                                                   (34)
K1 = diag(k1, k2),   K2 = diag(k3, k4),   k_i > 0,   0 < μ < 1                         (35),(36)
```
Total: `τ = τ_eq + τ_r`  (39). (Fig. 2 labels the block "AFNTSMDC law (40)" — **[TYPO?]**, it is (39).)

Dimensions: H 2×3, G 3×2, M0⁻¹B 2×2 → `HGM0⁻¹B` is 2×2. Signals needed: p_e (from pose),
z (measured v, ω), ṗ_e via (18), Ḣ (needs ẋ_e, ẏ_e), Ḟ (needs v̇_r, ω̇_r, φ̇_e), Ġ (needs ẋ_e, ẏ_e).

Note on the adaptive term: substituting into (30) gives `HGM0⁻¹B · (−B⁻¹M0LĈ) = −HG L Ĉ`, which
is what appears in (42). The compensation is therefore `−HG·L·Ĉ`, not simply `−L·Ĉ`.

### 3.4 Adaptive law, (37)–(38), with dead zone (53)–(54)
```
dĉ1/dt = ζ1 |s1|,        dĉ2/dt = ζ2 |s2|                                              (37)
dĉ3/dt = ζ3 |v| |s1|,    dĉ4/dt = ζ4 |ω| |s2|                                          (38)
ζ_i ≥ 0,   ĉ_i(0) ≥ 0
```
Remark 2 (used in the experiments): dead zone of width ε to stop gain drift from noise-induced
chattering of s:
```
ĉ1, ĉ3 frozen when |s1| ≤ ε;   ĉ2, ĉ4 frozen when |s2| ≤ ε;   ε = 0.05             (53),(54)
```
The estimates are monotonically non-decreasing (no leakage / projection) — a known weakness.

### 3.5 Boundary layer (Remark 3, (55))
`sgn(·)` "in the reaching law (32)" is replaced by
```
sat(ϑ) = sgn(ϑ) if |ϑ| > ϱ;  ϑ/ϱ if |ϑ| ≤ ϱ;       ϱ = diag(0.04, 0.04)               (55)
```
Which `sgn`s exactly (L in (33) only? also inside sig(s)^μ?) is not stated → M9.

### 3.6 Singularity check (Remark 1, (52))
```
|HGM0⁻¹B| = −(2b / (r m0 I0)) · ( ρ x_e / ((|x_e|+ρ)(1+y_e²)) + 1 )                    (52)
```
Since `ρx_e/(|x_e|+ρ) ∈ (−1, 1)` and `1/(1+y_e²) ∈ (0, 1]`, the bracket is never 0 → the inverse
always exists. **[INFERRED]** Our own computation gives the prefactor `−2b/(r² m0 I0)` (B carries
1/r, so its determinant carries 1/r²). Only the scaling differs; the non-singularity argument stands.

### 3.7 Gains used by the paper (Sect. 3.3, p. 1444)
| Parameter | Value | Eq. |
|---|---|---|
| α = diag(α1, α2) | diag(7, 7) | (27) |
| β = diag(β1, β2) | diag(7, 6) | (27) |
| γ | 1.67 | (27) |
| ρ | 0.8 | (22) |
| K1 = diag(k1, k2) | diag(9, 4) | (35) |
| K2 = diag(k3, k4) | diag(15, 5) | (36) |
| μ | 0.8 | (32) |
| ĉ1(0), ĉ2(0), ĉ3(0), ĉ4(0) | 14, 14, 8, **6** | (37),(38) — printed as "c2(0) = 6" twice; the 4th is read as ĉ4(0) **[TYPO?]** |
| ζ1, ζ2, ζ3, ζ4 | 0.3, 0.4, 0.4, 0.5 | (37),(38) |
| ε (dead zone) | 0.05 | (53),(54) |
| ϱ (boundary layer) | diag(0.04, 0.04) | (55) |

Tuning guidance given (Sect. 3.3): larger α, β, γ → faster ξ convergence but more overshoot; smaller ρ
→ faster y_e, φ_e before x_e converges but φ_e oscillation; larger K1, K2 → more robust, less smooth;
μ trades chattering vs robustness; large ζ → saturation and overshoot.

---

## 4. Stability / finite-time proof (Sect. 3.2, App. A–B) — viva version

**Plain-language argument (what the paper claims):**
1. **Adaptive gains stay bounded (Lemma 1).** The ĉ_i only grow while s ≠ 0. Since the true
   uncertainty is bounded (Assumption 1), they eventually become large enough to dominate it,
   which drives s to zero, after which they stop growing. So each ĉ_i ends up below some c_i.
2. **The sliding variable reaches zero in finite time (Lemma 2, (42)–(47)).** With τ = τ_eq + τ_r,
   the sliding dynamics become `ṡ = −K1 s − K2 sig(s)^μ − HG(LĈ − M0⁻¹δ)` (42). Take
   `V = ½sᵀs + ½Σ(ĉ_i − c_i)²` (43). Using the uncertainty bound and ĉ_i ≤ c_i they show
   `V̇ ≤ −√2 η3 V^½` (45). By the standard lemma (App. A: V̇ + ιV^φ ≤ 0 with 0<φ<1 ⇒ V reaches 0
   by `t ≤ V^(1−φ)(0)/(ι(1−φ))`), s → 0 by `t_s ≤ √2 V^½(0)/η3` (47).
3. **On the surface, ξ reaches zero in finite time (App. B, (28)).** s = 0 means
   `ξ̇ = −αξ − β sig(ξ)^γ`, a scalar Lyapunov argument per channel gives a finite settling time t_ξ.
4. **ξ = 0 implies the pose error vanishes ((48)–(51)).** ξ = 0 gives x_e = 0 and φ_e = −atan(y_e).
   Plugging into (18): `ẏ_e = −v_r y_e / √(1+y_e²)` (49), so with `V = ½y_e²` and v_r > 0, y_e → 0,
   and φ_e = −atan(y_e) → 0 with it.
5. Net: x_e converges by `t_s + t_ξ`, then y_e and φ_e follow. The controller never divides by zero
   because `det(HGM0⁻¹B)` never vanishes for 0 < ρ < 1 (Remark 1).
6. Requires **v_r > 0** (persistent forward motion) for step 4.

### 4.1 Proof weaknesses (our reading, not stated in the paper — input for Phase 6)

1. **Non-diagonal HG in the reaching-phase Lyapunov step.** The adaptive compensation enters
   (42) as `−HG·L·Ĉ`. The step from `−sᵀHG(LĈ − M0⁻¹δ)` to `≤ −⌊s⌋ᵀ⌊HG⌋(Ĉ − ⌊M0⁻¹δ⌋)` is valid
   only if HG is diagonal. It is not:
   `HG = [[−1, y_e], [−h21, h21·y_e − h22·x_e − 1]]`, where h21 and h22 are the second-row
   entries of H in (24). Because of the off-diagonal terms, the sign of each s_i's compensation is
   not guaranteed, so the inequality is unjustified.
   *Improvement hook:* write the switching / adaptive term in the s-coordinates, as
   `−(HGM0⁻¹B)⁻¹ L Ĉ`, so that it maps to `−L Ĉ` in ṡ. This is a one-line change to (32).
2. **Lemma 1 is circular.** It assumes s reaches zero in order to bound ĉ_i, then uses that bound
   to prove that s reaches zero. The η3 in (46) depends on |s_i| and |c̄_i|, so (45) does not give
   a constant rate, and "finite time" is not cleanly established. The ĉ_i also never decrease (no
   leakage or projection). Under noise they drift upward, which the dead zone (53)–(54) only
   mitigates.
3. **γ > 1 gives no strict finite time on the surface.** With 1 < γ < 2,
   `ξ̇ = −αξ − β sig(ξ)^γ` converges only exponentially near ξ = 0. The γ-term speeds convergence
   *far* from the origin. Strict finite time needs an exponent below 1, which is the classic
   singularity trade-off of terminal SMC. Appendix B is written with exponent μ and
   `η1 = α1|ξ1| − β1|ξ1|^μ` (wrong sign). The ½-power in (28) does not match (B4).
4. **y_e converges only asymptotically.** `η4 = v_r|y_e|/√(1+y_e²) → 0` as y_e → 0, so (50)–(51)
   do not give a finite-time bound. y_e and φ_e converge asymptotically (exponentially, at rate
   ≈ v_r, near zero). (51) also has a stray minus sign. Convergence also requires v_r > 0.
5. **Practical, not exact, convergence.** With the dead zone (ε = 0.05) and boundary layer
   (ϱ = 0.04) used in the experiments, the closed loop is only ultimately bounded anyway.

Net claim we can defend: finite-time reaching of s (modulo points 1–2), then exponential/asymptotic
convergence of p_e. This is "fast", but not finite-time end to end.

---

## 5. Experimental settings (Sect. 4)

### 5.1 Platform (Sect. 4.2, Table 1, Fig. 4)
| Item | Value |
|---|---|
| m0 (nominal mass) | 4.500 kg |
| I0 (nominal inertia) | 0.560 kg·m² |
| r (wheel radius) | 0.042 m |
| 2b (driving-wheel separation) | 0.372 m → **b = 0.186 m** |
| h (O_m to O_l) | 0.110 m |
| Actuators | 2 Maxon DC motors with encoders, Maxon ESCON-36/2 drivers |
| Controller / DAQ | NI myRIO |
| **Sample time** | **0.01 s for all controllers** |
| Pose feedback | posture estimator (odometry) from motor encoders — no external ground truth |
| Control signal plotted | motor **voltage** [V], right and left |

### 5.2 Reference trajectory (Sect. 4.3, Fig. 5)
- Circle, **radius 0.60 m, angular speed 0.70 rad/s** → v_r = 0.42 m/s, ω_r = 0.70 rad/s (constant).
- **[INFERRED from Fig. 5a/b]** centre (0, 0.6) m, start at (0, 0), φ_r(0) = 0, counter-clockwise;
  φ_r reaches ≈ 14 rad at t = 20 s (= 0.7·20). Fig. 5b's y-axis label says "rad/s" — **[TYPO?]**, it is rad.
- Duration: **20 s** (stated for the disturbance experiment; all plots span 0–20 s).

### 5.3 Initial condition
- "Initial posture offset (−0.10, −0.10, 0)" (p. 1446). **[INFERRED]** this is the robot's pose
  relative to the reference start: p(0) = (−0.1, −0.1, 0), consistent with Fig. 5c–e, which start at
  x_e = 0.1 m, y_e = 0.1 m, φ_e = 0.
- Initial velocities z(0) not stated → M5.

### 5.4 Scenarios
| # | Name | Setting | Figs / Table |
|---|---|---|---|
| S1 | Initial offset | nominal robot, offset (−0.1, −0.1, 0) | Fig. 5, 6, Table 2 |
| S2 | Load uncertainty | 3.60 kg cylinder on the mass centre: **ΔM = diag(3.60, 0.01)** (printed "M = diag([3.60, 0.01])" — **[TYPO?]**, read as ΔM, i.e. Δm = 3.6 kg, ΔI = 0.01 kg·m²); controller keeps nominal M0; offset still applied ("an initial posture offset", value not restated) | Fig. 7, 8, Table 3 |
| S3 | Shock disturbance | `d1 = d2 = 6|sin(2πt)|` for 12 ≤ t ≤ 12.5 s, else 0 (61) — half-sine, 0.5 s, **amplitude 6.0 V**, acting "on the wheels" (enters as d in Fig. 2/3). Data shown for 11–18 s of a 20 s run. | Fig. 9, 10, Table 4 |

**[INFERRED]** S3 was very likely run *with the payload* of S2: Table 4 RMS values are nearly
identical to Table 3 (e.g. x_e RMS 2.805 vs 2.806 cm for AFNTSMDC; 3.227 vs 3.224 for CKC). Also
the RMS in Table 4 seems to be over the whole 20 s run, while MAX and settling time are over the
11–18 s window (MAX in Table 4 is far below the initial 10 cm offset). → M10.

### 5.5 Gains
AFNTSMDC: §3.7. Baselines: §7.

---

## 6. Figures and tables in the results section

All results are hardware measurements. "Need" = what our simulation must produce.

| Item | Shows | Need to regenerate |
|---|---|---|
| Fig. 1 | Robot top-view schematic | (none — schematic) |
| Fig. 2 | Unified AFNTSMDC block diagram | redraw (report) |
| Fig. 3 | Inner–outer cascade (CKC/RSMKC + PI) block diagram | redraw (report) |
| Fig. 4 | Photo of hardware platform | (none) |
| **Fig. 5** (S1) | (a) XY paths vs reference; (b) φ_g(t) vs φ_r(t) with 0–1.5 s inset; (c) x_e(t); (d) y_e(t); (e) φ_e(t); 0–20 s, 4 controllers | S1 sim of CKC+PI, RSMKC+PI, CSMDC, AFNTSMDC; log p, p_r, p_e |
| **Fig. 6** (S1) | (a) right, (b) left motor input [V] vs t | control signal per wheel; needs voltage model (M1) or plot torque instead |
| **Table 2** (S1) | RMS, MAX, settling time of x_e, y_e, φ_e per controller | metrics: RMS & MAX over 0–20 s; settling-time definition (M6) |
| **Fig. 7** (S2) | same layout as Fig. 5 with payload | S2 sim (plant m, I perturbed, controller nominal) |
| **Fig. 8** (S2) | motor inputs [V] with payload | as Fig. 6 |
| **Table 3** (S2) | same metrics as Table 2 | as Table 2 |
| **Fig. 9** (S3) | x_e, y_e, φ_e for 11–18 s, shock at 12–12.5 s | S3 sim; disturbance model (M3) |
| **Fig. 10** (S3) | motor inputs [V], 11–18 s | as Fig. 6 |
| **Table 4** (S3) | same metrics; MAX/settling evidently over the post-disturbance window | as Table 2, windowed |

Headline numbers claimed for AFNTSMDC (for later comparison, qualitative only):
- S1: x_e settling 0.680 s (Table 2) vs CKC 6.070, RSMKC 4.810, CSMDC 2.380; x_e RMS 1.918 cm (best).
  It is **not** best on y_e or φ_e RMS/MAX (CKC is better there).
- S2: x_e settling 1.120 s, x_e RMS 2.806 cm (best); φ_e settling best (3.860 s).
- S3: x_e MAX 1.624 cm (best), x_e settling 0.520 s.

Text–table inconsistencies (record, don't fix):
- S1 text says x_e settling "0.684 s", Table 2 says 0.680 s. "788%, 603%, 249% faster" are
  `(T_other / 0.684 − 1)·100` using the text value (6.070/0.684 − 1 = 7.87, etc.).
- S2 text says "1.128 s", Table 3 says 1.120 s.
- S3 text says AFNTSMDC 0.525 s vs CKC "0.520 s", RSMKC 0.870 s, CSMDC 0.670 s; Table 4 says
  AFNTSMDC 0.520 s and CKC **1.830 s**.
- S2 text: "increased amplitudes in Fig. 8b are 39.54%, 40.83%, 35.41%, 24.05%" — the measure of
  "amplitude" is not defined.

---

## 7. Baselines (Sect. 4.1)

### 7.1 CKC — classic kinematic control [19] (Kanayama) + PI inner loop
```
z_d = [v_d, ω_d]ᵀ = K_E p_e + K_D                                                     (56)
K_E = [[k_e1, 0, 0], [0, v_r k_e2, 0]],   K_D = [v_r cosφ_e,  v_r k_e3 sinφ_e + ω_r]ᵀ   (57)
⇒  v_d = v_r cosφ_e + k_e1 x_e,     ω_d = ω_r + k_e2 v_r y_e + k_e3 v_r sinφ_e
k_e1 = 1.7,  k_e2 = 1.7,  k_e3 = 1.5
```
Matches the CLAUDE.md outer loop with (k1, k2, k3) = (1.7, 1.7, 1.5).

### 7.2 RSMKC — robust sliding-mode kinematic control [15] (Nath et al.) + PI inner loop
```
s_r = [ k_r1 φ_e + v_r atan(y_e) ,  x_e ]ᵀ
z_d = −B_r⁻¹ ( A_r F_r + D_r + K_r sgn(s_r) )                                         (58)
A_r = [[0, v_r/(1+y_e²), k_r1], [1, 0, 0]]
B_r = [[0, −(k_r1 + v_r x_e/(1+y_e²))], [−1, y_e]]
F_r = [v_r cosφ_e, v_r sinφ_e, ω_r]ᵀ,   D_r = [v̇_r atan(y_e), 0]ᵀ,   K_r = diag(k_r2, k_r3)
k_r1 = 0.40,  k_r2 = 0.02,  k_r3 = 0.02
```
(For the constant-speed circle, v̇_r = 0 ⇒ D_r = 0.)

### 7.3 PI velocity inner loop (for CKC and RSMKC), (59)
```
u = k_P z_e(T) + k_I ∫₀ᵀ z_e(t) dt,    z_e = z_d − z,    u = [u1, u2]ᵀ
k_P = 30,  k_I = 250
```
Units of u and how [u1, u2] map to the two motors are not stated (Fig. 3 feeds u straight into
"Robot dynamics (11)") → M12. No anti-windup mentioned.

### 7.4 CSMDC — conventional sliding-mode dynamic control (unified, same structure as AFNTSMDC), (60)
```
s_csm = λ_c ξ + ξ̇                         (ξ from (22))
τ_eq2 = −(HGM0⁻¹B)⁻¹ ( λ_c ξ̇ + Ḣ ṗ_e + H(Ḟ + Ġ V) )      ["ĠV" read as Ġz — [TYPO?]]
τ_r2  = −(HGM0⁻¹B)⁻¹ K_c sgn(s_csm)
τ2    = τ_eq2 + τ_r2
K_c = diag(k_c1, k_c2) = diag(21, 16),    λ_c = diag(λ_c1, λ_c2) = diag(3, 4)
```
Whether CSMDC also used the boundary layer (55) is not stated → M9.

Mapping to the CLAUDE.md comparison list: CKC+PI ≈ "kinematic + PID"; CSMDC ≈ "conventional SMC",
but the paper's CSMDC is **unified** (not an inner loop); RSMKC is not in CLAUDE.md at all.
CLAUDE.md's "kinematic only, perfect velocity tracking" is not in the paper (ours, for reference). → D1.

---

## 8. MISSING / AMBIGUOUS — needs a decision before Phase 2/4

**Decided 2026-10-19:** every item below is resolved as an `ASSUMED` entry in
`docs/reproduction_log.md`. That file is authoritative for what we simulate.

**Actuation and plant**
- **M1. Torque vs voltage.** The law outputs torque τ (N·m) but the hardware input, the plots
  (Fig. 6, 8, 10) and the disturbance amplitude are in **volts**. No motor model (torque constant,
  gear ratio, resistance, back-EMF) or driver mode (current vs voltage control) is given.
- **M2. Saturation.** No torque/voltage limit given. Fig. 6 inputs appear to clip near −8 V; ESCON
  36/2 supports ≤ 36 V supply but the actual limit is unknown. No rate limits mentioned.
- **M3. Disturbance units/injection.** (61) is 6 V on "the wheels". To use in our torque model we need
  a V→N·m (or V→d) conversion, and the sign/channel: d1 = d2 in (61) are labelled as d in (9)
  (i.e. already in [v̇, ω̇]-equation units?) or per-wheel torques τ_d1, τ_d2? Unclear.
- **M4. Measurement / noise.** Pose comes from encoder odometry; noise levels, encoder resolution and
  estimator are not given. Paper attributes residual chattering to measurement noise and "coarse
  sensor resolution". We need to choose a noise model (or none) for the nominal reproduction.
- **M5. Initial velocity** z(0) not stated (assume robot at rest, z(0) = 0?).
- **M7. Sign of d / δ.** (9) vs (11) disagree on the sign of d (see §1.4). Need a convention for
  injecting disturbances into the plant.
- **M8. h and Coriolis terms.** h = 0.11 m listed but unused. Derived reduced model has extra
  `[m h ω², −m h v ω]` terms that (8) omits. Simulate the paper's (8) (h ignored) or the full
  reduced model with h as an unmodelled effect?
- **Friction.** Mentioned as part of τ_d/δ, never quantified.

**Controller details**
- **M9. Boundary layer scope.** Which `sgn` terms use sat(·)/ϱ (L only, or also sig(s)^μ)? Does
  CSMDC use a boundary layer too? RSMKC's sgn(s_r)?
- **M12. PI inner loop.** Units of u, mapping [u1, u2] → wheels (is u in (v, ω) space converted by
  B⁻¹, or per wheel?), anti-windup, initial integrator state.
- **M13. ĉ4(0).** Printed as "c2(0) = 6" (duplicate); we read it as ĉ4(0) = 6.
- **M14. Numerical guards.** sgn(x_e) in H at x_e = 0 and sig(s)^μ with μ < 1 near s = 0 — no
  numerical treatment described (μ < 1 is fine for sig itself, but its derivative is not needed).
- **M15. Derivatives for τ_eq.** Ḣ, Ḟ, Ġ require ẋ_e, ẏ_e, φ̇_e, v̇_r, ω̇_r; in hardware these must
  come from measured/estimated velocities. Our simulation can compute them analytically from state.

**Scenario / evaluation**
- **M6. Settling-time definition.** Tolerance band and whether it is absolute or relative are not
  given (e.g. 2% of initial error? fixed band like 1 cm?). RMS window also not stated.
- **M10. S2/S3 details.** Initial offset value in S2 (assume same as S1). Whether S3 includes the
  S2 payload (tables suggest yes). Window used for Table 4 metrics.
- **M11. Angle wrapping** of φ_e not discussed.
- **Reference geometry** (centre, start point, direction) only inferable from Fig. 5.
- **Controller sample time vs integration step.** Paper: Ts = 0.01 s. CLAUDE.md default Ts = 1e-3.

**Decision items (architecture)**
- **D1. Controller set.** Paper compares CKC+PI, RSMKC+PI, CSMDC (unified), AFNTSMDC (unified).
  CLAUDE.md plans kinematic-only, kinematic+PID, kinematic+SMC-inner-loop, AFNTSMDC, improved.
  Choose: follow the paper's set, CLAUDE.md's set, or the union.
