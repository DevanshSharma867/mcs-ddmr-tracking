# Grading map

Course criteria (EEE F422 term project):
- **(U)** understanding of the problem
- **(R)** reproduction of a state-of-the-art method
- **(I)** improvement on that method
- **(P)** presentation

| Report section / artefact | U | R | I | P | Evidence |
|---|---|---|---|---|---|
| §1 Problem and model (nonholonomic constraint, dynamics, robot-frame error) | ● | | | | `src/ddmr/model.py`, `errors.py`; tests in `tests/test_model.py`, `test_errors.py` |
| §2 Literature (Kanayama, Utkin, Nath, Fierro–Lewis, Han, Gao, Guo–Zhao, Gao–Wang–Homaifa, Cui) | ● | | | ○ | `docs/paper/references.bib` (verified) |
| §3 The paper's method (AFNTSMDC) | ● | ● | | | `docs/paper_notes.md`, `src/ddmr/controllers/afntsm.py`, `xi_terms.py` |
| §4 Reproduction set-up, assumptions, verdict (F1) | | ● | | | `docs/reproduction_log.md`, `results/reproduction/`, F1 |
| §5 Sign inconsistency in the printed law (F2) | ● | ● | | | `docs/paper/sign_error_verification.md`, `experiments/sign_verification.py` |
| §6 Discrete-time chattering diagnosis (F3) | ● | ● | ○ | | F3, `results/report/F3_rho_sweep.csv` |
| §7 Robustness study: two plants, grid, drift (F4, F5) | ● | ● | ○ | | `results/robustness/`, `results/drift/`, `docs/robustness.md` |
| §8 Pre-registered improvement and ablation (F6) | | | ● | | `docs/phase6_preregistration.md`, `results/phase6/criteria.md`, `src/ddmr/controllers/improved.py` |
| §9 Limitations (F7) | ● | | ● | | `results/phase6/diagnostics*.csv` |
| §10 Conclusion and future work | | | ○ | ● | — |
| Appendices A–C (assumptions, parameters, proof sketch with gaps) | ● | ● | ○ | | `docs/reproduction_log.md`, configs, `docs/phase6_results.md` |
| F1–F7, one-command regeneration (`experiments.run_all`) | | ○ | ○ | ● | `experiments/report_figures.py`, `results/report/` |
| Slides (`slides_outline.md`), viva prep (`viva_prep.md`) | ○ | | | ● | — |
| Claims ledger, evidence tiers, key-number table | | ○ | ○ | ● | `docs/paper/claims.md`, `docs/paper/evidence_tiers.md`, `results/report/key_numbers.csv` |

● = primary contribution to the criterion, ○ = supporting.
