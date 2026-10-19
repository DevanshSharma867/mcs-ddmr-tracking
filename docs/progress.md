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

## Phase 2: Plant model, trajectories, simulator — TODO
## Phase 3: Baselines (kinematic-only, CKC+PI, RSMKC+PI, CSMDC) — TODO
## Phase 4: Reproduce AFNTSMDC — TODO
## Phase 5: Robustness study — TODO
## Phase 6: Improvement — TODO
## Phase 7: Report and slides — TODO
