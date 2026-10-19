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

## Phase 1: Paper extraction — TODO
## Phase 2: Plant model, trajectories, simulator — TODO
## Phase 3: Kinematic, PID and SMC baselines — TODO
## Phase 4: Reproduce AFNTSMDC — TODO
## Phase 5: Robustness study — TODO
## Phase 6: Improvement — TODO
## Phase 7: Report and slides — TODO
