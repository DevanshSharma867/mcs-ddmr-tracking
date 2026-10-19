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
## Phase 3: Baselines (kinematic-only, CKC+PI, RSMKC+PI, CSMDC) — TODO
## Phase 4: Reproduce AFNTSMDC — TODO
## Phase 5: Robustness study — TODO
## Phase 6: Improvement — TODO
## Phase 7: Report and slides — TODO
