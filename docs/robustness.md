# Robustness study (Phase 5 step 3)

Source: `python -m experiments.robustness --config configs/robustness.yaml` → `results/robustness/`.
It runs 1,800 runs in ≈ 2 min on 10 cores, and **none diverged**. The definitions (disturbance
equivalence, friction levels, figure-8, noise) were fixed before running; see
`docs/reproduction_log.md`, "Robustness grid".

**Design.**
- One factor at a time around nominal (S1 set-up + moderate noise).
- 5 seeds; the paper circle and a figure-8.
- Plants:
  - (a) the torque-input plant, 19 conditions
  - (c12V) the voltage-input RE 25 12 V motor, 17 conditions (no actuator lag)
- Controllers: CKC+PI, RSMKC+PI, CSMDC, AFNTSMDC (paper ϱ = 0.04) and AFNTSMDC (ϱ = 0.15).
  All use the paper's gains, and AFNTSMDC is sign-corrected.

**How to read it.**
- `pos_rms_*` heatmaps and `summary_*.md`: whole-run position RMS. This is dominated by the 14 cm
  start transient, so it moves little between conditions.
- **Steady-state behaviour is judged by the late error**, i.e. the max position error over the last
  5 s (`runs.csv` column `pos_late_max_cm`), mean of 5 seeds.
- On **(c12V)**, no model-based controller settles even at nominal (Phase 5 step 1). The rankings there
  measure **graceful degradation, not robustness** around a working operating point.

## Late position error [cm] (last 5 s, mean of 5 seeds), selected conditions

Plant (a), paper circle:

| Condition | CKC+PI | RSMKC+PI | CSMDC | AFNTSMDC ϱ=0.04 | AFNTSMDC ϱ=0.15 |
|---|---|---|---|---|---|
| nominal | 0.24 | 0.23 | 2.52 | 0.80 | 0.16 |
| noise ×0 | 0.01 | 0.12 | 2.53 | 0.49 | 0.02 |
| mass ×0.7 | 0.24 | 0.22 | 7.47 | 1.13 | 0.69 |
| dist constant (a0 both wheels) | 0.24 | 0.23 | 1.80 | 1.29 | 0.49 |
| dist yaw (±a0) | 0.24 | 0.21 | 1.35 | 1.31 | 0.35 |
| dist sine 1 Hz | 5.27 | 4.95 | 0.92 | 0.78 | 0.25 |
| dist sine 0.2 Hz (2·a0) | 4.76 | 7.41 | 1.66 | 1.01 | 0.45 |
| dist band-limited | 3.04 | 2.91 | 1.23 | 0.82 | 0.22 |
| friction 30 % | 0.21 | 1.69 | 5.48 | 1.77 | 1.25 |
| lag 40 ms | 0.26 | 0.63 | 1.59 | 0.92 | 0.61 |

Plant (c12V), graceful degradation:

| Condition | CKC+PI | RSMKC+PI | CSMDC | AFNTSMDC ϱ=0.04 | AFNTSMDC ϱ=0.15 |
|---|---|---|---|---|---|
| circle, nominal | 1.33 | 32.5 | 96.3 | 11.7 | 11.4 |
| circle, friction 30 % | 3.72 | 37.7 | 116.0 | 15.6 | 15.6 |
| figure-8, nominal | 13.7 | 40.6 | 57.6 | 6.06 | 6.26 |
| figure-8, friction 30 % | 14.8 | 52.3 | 95.8 | 9.06 | 9.18 |

Chattering index (AFNTSMDC, plant (a), circle), ϱ = 0.04 / ϱ = 0.15:

| Condition | ϱ = 0.04 | ϱ = 0.15 |
|---|---|---|
| noise ×0 | 839 | 9 |
| noise ×0.5 | 840 | 149 |
| moderate noise | 837 | 275 |
| noise ×2 | 806 | 433 |
| mass ×0.7 | 933 | 855 |
| lag 20 ms | 334 | 166 |
| lag 40 ms | 247 | 203 |

## Weaknesses observed (AFNTSMDC, reproduced with the paper's gains)
1. **No integral action, so constant and structured disturbances leave a steady offset.**
   - Plant (a), circle:
     - constant push: late error 1.29 cm, against 0.24 cm for CKC+PI, whose PI integrator removes it
     - yaw push: 1.31 cm
     - 30 % friction: 1.77 cm
   - Plant (c), circle: AFNTSMDC holds 11.7 cm while CKC+PI's integrator gets down to 1.33 cm. The
     back-EMF is a velocity-proportional, *structured* disturbance that the bound-based adaptive
     switching cannot cancel.
   - The same mechanism drives the ĉ₁/ĉ₃ growth in the drift test.
2. **Sampled chattering at the paper's ϱ = 0.04 in every condition** (index 590–930 on plant (a)).
   It also leaves a steady quasi-sliding residual: 0.80 cm late error at nominal, which is *worse than
   both PI baselines in steady state*.
   - ϱ = 0.15 removes the residual (0.16 cm) and most of the chattering without noise.
   - But its linear layer then amplifies measurement noise: chattering 9 → 433 from no noise to
     2× noise.
   - It loses its discrete-time stability margin when the real plant is lighter than the model
     (mass ×0.7: chattering 855). The effective K·Ts/ϱ grows by 1/0.7, consistent with the
     K·Ts/ϱ < 2 condition from Phase 3.
3. **Adaptive gains only grow.**
   - In 20 s: +0.4 to +1.1 on plant (a), +2 to +5.6 on (c).
   - The final values hardly depend on the condition, i.e. the adaptation does not track the actual
     uncertainty level.
   - The growth is chattering- and noise-driven (see the long-horizon drift test).
4. **Strength confirmed: periodic and broadband disturbance rejection.**
   - Under 1 Hz / 0.2 Hz sinusoidal and band-limited input disturbances, AFNTSMDC's late error stays
     at 0.7–1.0 cm (ϱ = 0.15: 0.22–0.48).
   - CKC+PI and RSMKC+PI degrade to 3–7 cm, a 3–7× (ϱ = 0.04) or ≈ 10–20× (ϱ = 0.15) advantage.
   - It is also the least sensitive to mass/inertia mismatch of the model-based laws: CSMDC reaches
     7.5 cm at mass ×0.7.
5. **Plant (c) (graceful degradation).**
   - On the figure-8, AFNTSMDC degrades least (6.1 cm vs CKC+PI 13.7 cm): CKC's integrator cannot
     keep up with the time-varying speed demand.
   - On the circle, CKC+PI's integrator wins (1.3 vs 11.7 cm).
   - Friction hurts all controllers. Mass mismatch is irrelevant, because back-EMF damping dominates
     the inertia.
6. **Measurement noise and actuator lag at the tested levels** change AFNTSMDC's tracking little
   (late error 0.49–0.92 cm). Lag reduces its chattering (837 → 334 → 247 for 0 / 20 / 40 ms).

These point Phase 6 at:
- (i) an estimate-and-cancel path for structured disturbance, e.g. a disturbance observer or ESO,
  which also covers back-EMF and friction
- (ii) leakage or projection for the adaptive gains, kept separate from (i) so it does not bleed off
  legitimate compensation
- (iii) a switching/boundary-layer design tied to Ts and robust to noise and plant-gain variation
