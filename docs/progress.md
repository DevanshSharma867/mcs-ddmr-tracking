# Progress

## Phase 0: Setup — DONE (2026-10-19)
**Done**
- `pyproject.toml` (uv, hatchling, src layout); deps numpy, scipy, matplotlib, pyyaml, pandas; pytest in the `dev` group.
- Repo layout from CLAUDE.md: stub modules with docstrings in `src/ddmr/`, `Controller` interface in `src/ddmr/controllers/base.py`, `configs/`, `experiments/`, `tests/`, `docs/`.
- `docs/paper_notes.md`, `docs/reproduction_log.md` with section headers.
- `.gitignore`: `results/*` ignored except `results/report/` (final report figures are tracked). `CLAUDE.md` stays local-only.

**Verified**
- `uv sync` succeeds; `uv run pytest -q` passes (1 smoke test).

**Open**
- `ref` type in `Controller.compute` is untyped until trajectories are implemented (Phase 2).

## Phase 1: Paper extraction — DONE (2026-10-19)
**Done**
- Read `papers/primary.pdf` end to end (17 pp.); equations checked against rendered pages.
- `docs/paper_notes.md`: model (1)–(13), error/kinematics (14)–(21), AFNTSMDC (22)–(39), adaptive
  law + dead zone + boundary layer (37)–(38), (53)–(55), proof summary + §4.1 proof weaknesses,
  experimental settings, figure/table inventory, baselines (56)–(60), MISSING/AMBIGUOUS list.
- All MISSING items (M1–M15, reference circle, Ts, D1) decided and logged as ASSUMED in
  `docs/reproduction_log.md`.
- Architecture aligned with the paper: controller stubs renamed to `ckc.py`, `rsmkc.py`, `pi.py`,
  `csmdc.py` (+ `afntsm.py`, `improved.py`); CLAUDE.md layout, controller list and Ts default
  (0.01 s) updated (CLAUDE.md is git-ignored, so that change is local only).

**Verified**
- Gains, Table 1 parameters and scenario settings transcribed from the paper; figure-derived
  values tagged [INFERRED], suspected misprints [TYPO?]. `pytest -q` passes.

**Open**
- Paper is a hardware study (voltage inputs, odometry feedback); reproduction can only be qualitative.

## Phase 2: Plant model, trajectories, simulator — DONE (2026-10-19)
**Done**
- `model.py`: `RobotParams`, `Plant` implementing eq. (4) and (8): per-wheel input saturation (then
  additive input disturbance), optional h-terms (off), `InputDisturbance` (constant/step,
  sinusoid, half_sine for eq. 61, `wheel_signs`), `MeasurementNoise`.
- `trajectories.py`: `RefSample`; `Circle` / `paper_circle()`, `FigureEight`, `StraightLine`, all
  with analytic v_r, ω_r (16)–(17), v̇_r, ω̇_r.
- `errors.py`: `wrap_angle` to (−π, π], `tracking_error` (14), `error_rate` (18).
- `sim.py`: fixed-step RK4 (dt = 1e-3) with ZOH at Ts = 0.01; integer tick/step counters
  (`steps_ratio` asserts that Ts/dt and T/Ts are integers to within 1e-9); velocity mode for the
  ideal kinematic reference; per-tick DataFrame log and CSV. `config.py` builds everything from YAML.
- `configs/robot_paper.yaml` (S1), with ASSUMED values tagged. `Controller.compute` now takes a `RefSample`.
- Checked against the paper: h-term signs (a convention: CoM behind the axle; documented in
  paper_notes §1.5), initial offset (−0.1, −0.1) confirmed from Fig. 5a, shock is exactly one
  hump with same-sign wheels (supported by Fig. 9).
- Demo `python -m experiments.phase2_open_loop` (also via `run_all`):
  `results/phase2/open_loop_circle.{png,pdf,csv}`.

**Verified**
- `pytest -q`: 37 passed. This covers the model sanity checks, saturation, shock shape/area, h-terms,
  trajectory derivatives vs finite differences, error wrapping and rotation, `error_rate` vs FD,
  tick timing, ZOH, seeded noise, and exact velocity-mode tracking.
- Open-loop circle: R = v/ω = 0.6 m, max radius error 2.7e-13 m over 20 s.

**Open**
- Band-limited noise disturbance and input rate limits are deferred to Phase 5.
- Controller diagnostics (s, ĉ) logging will be added in Phase 4.
## Phase 3: Baselines (kinematic-only, CKC+PI, RSMKC+PI, CSMDC) — DONE (2026-10-19)
**Done**
- Controllers:
  - `ckc.py`: eq. (56)–(57), velocity output; run alone it is the ideal reference.
  - `rsmkc.py`: eq. (58); B_r singularity raises.
  - `pi.py`: eq. (59), τ = B⁻¹u, clamping anti-windup (M12 revised).
  - `xi_terms.py`: eq. (18)–(26), shared with AFNTSMDC.
  - `csmdc.py`: eq. (60), pure sgn, ρ = 0.8 ASSUMED; optional boundary layer for the diagnostic.
  - `switching.py`: `sat` from eq. (55).
- `metrics.py`: RMS / MAX / settling (M6 bands, optional window), effort, chattering (TV/s),
  % saturated, plus a dependency-free markdown table. `paper_tables.py` holds the Table 2 values.
- `configs/controllers_paper.yaml` holds the paper gains and the diagnostic ϱ. `config.build_controller`.
- `experiments/baselines.py` → `results/baselines/`: paths, errors and inputs figures, per-controller
  CSVs, `metrics.csv` / `metrics.md`. Includes "CSMDC+BL (diagnostic)" (ϱ = 0.04), which is not a
  paper baseline; Phase 4 uses it to separate the boundary-layer effect from the terminal
  surface / adaptation. It is in `run_all`.
- Simulator: divergence now always raises `FloatingPointError` with its time.
- The payload-scenario start offset is confirmed from Fig. 7a: (−0.1, −0.1), same as S1.
- Qualitative comparison with Fig. 5/6 and Table 2, plus the CSMDC quasi-sliding finding
  (mechanism, Ts = 1 ms evidence, ϱ sweep), is in `reproduction_log.md`.

**Verified**
- Unit tests pass for: the CKC law, RSMKC sliding dynamics, ξ̇ and ξ̈ (eq. 23, 26) vs finite
  differences, CSMDC ṡ = −K_c sgn(s) and its boundary-layer linear regime, det(HGM0⁻¹B) vs
  eq. (52) (with the 1/r² prefactor), PI mapping and anti-windup, and metrics on synthetic signals.
- Closed-loop acceptance uses thresholds fixed from Table 2 before running (2× the paper's
  settling time and MAX). Ideal, CKC+PI and RSMKC+PI pass everything, CSMDC x_e passes, and
  there is no NaN/Inf in any run.
- `pytest -q`: **64 passed, 2 xfailed**. The xfails are CSMDC y_e and φ_e, `strict=True`, thresholds
  unchanged (decision A). Cause: discrete quasi-sliding at Ts = 0.01 s.

**Open**
- Our CKC+PI is about 2.5–3× faster than the paper's (idealised inner loop).
- Phase 4 should check the prediction that AFNTSMDC (ϱ = 0.04, ĉ·Ts/ϱ ≈ 3.5) also chatters at Ts = 0.01 s.

## Phase 4: Reproduce AFNTSMDC — DONE (2026-10-19)
**Done**
- `controllers/afntsm.py`: eq. (22)–(39), adaptive law (37)–(38) with dead zone (53)–(54), boundary
  layer (55) on L, guards `sig`/`abs_pow` (M14). It logs s, ĉ1..ĉ4, L, the switching term and
  τ_eq/τ_r every tick (`Controller.diagnostics()` hook, merged into the sim log). Switching variants:
  `printed` | `corrected` (the reproduced controller, DEVIATION) | `general` (−D⁻¹LĈ) | `none`.
- **Finding:** the printed (32) is destabilising (HG ≈ −I ⇒ −HG·L·Ĉ ≈ +sgn(s)·Ĉ). Documented with
  the derivation and evidence; the as-printed law is kept as a literal run that is tested to diverge.
- Plant: optional per-wheel actuator lag (phenomenological, τ_a ∈ {10, 20, 40} ms, off by default).
- Config `base:` deep-merge. `configs/s2_payload.yaml`, `s3_payload_shock.yaml`, `reproduction.yaml`.
  Fixed a latent YAML bug: `true:` key → `actual:` (no results affected).
- `experiments/reproduction.py` (in `run_all`): 120 runs (10 controllers × S1–S3 × 4 plants, ≈ 1 min).
  It produces paper Figs 5–10 and Tables 2–4 per plant (Table 4 over 0–20 s and over 12–20 s, with
  settling measured from 12.0 s and 12.5 s), adaptive-gain plots, chattering vs lag, `variants.md`
  (boundary-layer / switching attribution) and `summary.csv`.
- Reproduction verdict in `reproduction_log.md`.

**Verified**
- Unit tests:
  - eq. (42) closed-loop ṡ for all four switching variants vs finite differences
  - guards
  - adaptive increments and dead zone
  - actuator lag via the existing plant tests
  - scenario configs build the expected plants
- AFNTSMDC acceptance (fixed 2× Table 2 thresholds, incl. MAX): **all pass on the ideal plant and at
  τ_a = 20 ms.** The as-printed law diverges, as expected.
- `pytest -q`: 85 passed, 2 xfailed (the CSMDC strict xfails from Phase 3).

**Open / handed to later phases**
- AFNTSMDC chatters at Ts = 0.01 s with ϱ = 0.04 (index ≈ 840). Lag reduces it but does not remove it.
- ĉ1..ĉ4 drift linearly and never level off: the dead zone ε = 0.05 is narrower than the s chattering
  band. This motivates leakage or projection (Phase 6).
- S3 shock is 10–60× too strong in model units (M1/M3). Baselines lose the path by ≈ 1 m.
- The Phase 3 hypothesis "actuator lag explains CKC+PI being 2.5–3× faster" is **not supported**
  (τ_a ≤ 40 ms changes CKC+PI settling by < 10 %).

## Phase 5: Robustness study — IN PROGRESS (steps 1–2 done 2026-10-19; grid pending approval)
**Done**
- Step 1: `MotorParams` / plant (c), a voltage-input DC motor with back-EMF and reflected rotor
  inertia (maxon RE 25 12 V / 18 V + GP 32 A 33:1 datasheet values; the paper names no motor). The
  S3 prediction was logged before running. The reproduction was re-run with 6 plants (180 runs,
  ≈ 1.5 min).
  - The verdict is in `reproduction_log.md`: neither plant reproduces the paper.
  - Plant (c) matches the disturbance scale, steady inputs and smoothness better.
  - The torque plant matches the tracking headline better.
  - The CKC+PI speed gap is bracketed, not closed.
  - The S2 payload effect does not appear on either plant.
- Step 2: `experiments/drift.py` (in `run_all`), 36 runs × 200 s (≈ 2 min).
  - No adaptive gain levels off.
  - Two mechanisms: chattering-driven (ideal plant) and noise-driven (ĉ₂, ĉ₄ on the motor plants,
    flat without noise).
  - Rates 0.01–0.08 1/s.

**Verified**
- Motor plant tests:
  - The steady speed matches back-EMF balance.
  - The free-decay rates match the analytic values.
  - Combining the motor with actuator lag raises an error.
- `pytest -q`: 88 passed, 2 xfailed.

**Open**
- Robustness grid: plan and run count to be approved. Seeds and conditions will go in config so
  Phase 6 can re-run the identical grid.

Candidates (added during Phase 3):
- Viscous wheel friction as unmodelled dynamics (paper's ~−1.5 V steady input suggests friction).
- First-order actuator lag (time constant config, off by default). Proposed in Phase 3 as the
  likely reason our CKC+PI is 2.5–3× faster than the paper and our CSMDC chatters while the paper's
  hardware CSMDC was smooth. **Implemented and tested in Phase 4: it does not explain the CKC+PI
  speed gap and only partly reduces chattering (2–3.5× at 10–40 ms).**
- Voltage-input DC motor with back-EMF (the paper's input is volts; back-EMF gives velocity damping
  our torque-input plant lacks). This is also the most likely way to put the S3 shock (6 V) on the
  right scale.
## Phase 6: Improvement — TODO
## Phase 7: Report and slides — TODO
