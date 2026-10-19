# Evidence tiers of existing results (rule 4, docs/paper/README.md)

Classified 2026-10-19 **strictly from git evidence**. Commit order (all on 2026-10-19, local only):

| Commit | Time | Content |
|---|---|---|
| f1f585e | 13:06 | phase 0 |
| 4a2f004 | 13:32 | phase 1 |
| 73f99c1 | 13:42 | phase 2 |
| ea256b3 | 14:51 | phase 3 |
| 1eabfbf | 15:25 | phase 4 |
| b37864e | 15:40 | phase 5 step 1 |
| addbcca | 15:41 | phase 5 step 2 |
| 1b10b13 | 15:56 | phase 5 step 3 |
| 3e595fd | 16:07 | phase 6 pre-registration |
| ca41546 | 16:23 | phase 6 implementation and results |
| d1ebcea | 16:29 | phase 6 diagnostics |

From commit 4b42530 onward, ordering is externally timestamped by the GitHub remote.
(The GitHub remote is the private repository DevanshSharma867/mcs-ddmr-tracking; the first push was
on 2026-10-19. GitHub records the *push* time, so commits pushed together share one external
timestamp. For a pre-specified threshold to be externally ordered before its results, push the
threshold commit before running.)

**General caveats**
- `results/` is never committed, so a run's timing is inferred from when the *code that produces it*
  was first committed.
- The commits are local and have not been pushed to a remote, so their timestamps are self-asserted.
  There is no third-party timestamp, which is the usual limitation of an informal pre-registration.
- Many thresholds and predictions in Phases 3–5 *were* written before their runs (session record), but
  they were committed **in the same commit** as their results. Under rule 4 that does not qualify, so they
  are listed as exploratory with the note "fixed before run, order not git-verifiable".

## Confirmatory (formally pre-registered)
| Result | Evidence of order | Notes |
|---|---|---|
| Phase 6 criteria P1–P7 for A3 (all pass) | Pre-registration 3e595fd (16:07) precedes implementation ca41546 (16:23). At 3e595fd, `src/ddmr/controllers/improved.py` is a one-line docstring stub (`git show 3e595fd:src/ddmr/controllers/improved.py`), so the committed code could not have produced the results. | Results: `results/phase6/criteria.md` (`python -m experiments.phase6_eval`). |
| Pre-stated ablation expectation: "A2 is expected to degrade under large structured disturbances" | In 3e595fd (line 84 of `docs/phase6_preregistration.md`). | Outcome: supported. A2: friction 30 % 4.07 cm, motor12V circle 16.9 cm (`results/phase6/ablation_*.md`). A qualitative expectation, not a numeric criterion. |

## Pre-specified (committed before the runs that test them)
| Result | Pre-specification commit | Test commit | Outcome |
|---|---|---|---|
| Prediction that AFNTSMDC at Ts = 0.01 s, ϱ = 0.04 (ĉ·Ts/ϱ ≈ 3.5 > 2) chatters on the idealised plant | ea256b3, `docs/reproduction_log.md` "Implication for Phase 4 (a prediction to check, not a result)" | 1eabfbf (the AFNTSMDC code first appears here; it was a stub before) | **Supported:** chattering index 839 (S1, ideal). Caveat: qualitative, with no numeric threshold. At ea256b3 the sign issue was unknown, and the prediction was tested on the sign-corrected law (the printed law diverges instead). |
| Hypothesis that first-order actuator lag explains (i) our CKC+PI being 2.5–3× faster than the paper and (ii) CSMDC chattering while the hardware was smooth | ea256b3, `docs/progress.md` Phase 5 candidates ("This likely explains…") | 1eabfbf (the actuator-lag plant first appears here) | **(i) Refuted:** CKC+PI x_e settling 2.24 / 2.23 / 2.21 / 2.46 s for τ_a = 0 / 10 / 20 / 40 ms. **(ii) Partly supported:** chattering is reduced 2–3.5×, not removed. Caveat: the τ_a values {10, 20, 40} ms were fixed in 1eabfbf itself. |

No other Phase 3–5 threshold or prediction meets the commit-order requirement (see Exploratory).

## Exploratory
| Result | Phase / commit | Note |
|---|---|---|
| Baseline acceptance vs 2× Table 2 thresholds; CSMDC y_e and φ_e fail (strict xfail) | P3, ea256b3 | Thresholds fixed before the run, but order not git-verifiable (test file and results write-up in the same commit). |
| CSMDC discrete quasi-sliding mechanism, Ts = 1 ms check, ϱ sweep, K·Ts/ϱ < 2 boundary | P3, ea256b3 | Mechanism derived after observing the failure. |
| Qualitative comparison of baselines with the paper's Fig. 5/6 | P3, ea256b3 | — |
| Sign error in eq. (32): the HG ≈ −I derivation | P4, 1eabfbf | The *derivation* is analytic (strength: proven, under the paper's own definitions) and found after observing divergence. The *simulation evidence* is exploratory. |
| As-printed law diverges (test criterion: late error > 10 cm, ĉ₄ > 2·ĉ₄(0)) | P4, 1eabfbf | Criterion fixed before the run, order not git-verifiable. |
| Sign-corrected AFNTSMDC acceptance (2× Table 2, ideal and 20 ms lag) | P4, 1eabfbf | Thresholds fixed before the run, order not git-verifiable. |
| Reproduction verdict per paper figure/table; adaptive-gain ramp; boundary-layer attribution; switching-term value | P4, 1eabfbf | — |
| Plant (c) motor choice, S3 shock prediction and its outcome | P5.1, b37864e | Prediction written to the log before the run, but committed together with the results, so order not git-verifiable. |
| Plant (a) vs (c) consistency table | P5.1, b37864e | — |
| 200 s adaptive-gain drift (rates, two mechanisms) | P5.2, addbcca | Noise levels fixed before the run, same commit. |
| Robustness grid results and "weaknesses observed" | P5.3, 1b10b13 | Disturbance equivalence and friction levels were logged before the run, but committed with the results. |
| Phase 6 ablation comparisons beyond P1–P7, incl. **A4 ≈ A3** | P6, ca41546 | The comparisons were named in 3e595fd, but without a criterion. |
| P5 report: A3 vs CKC+PI on motor12V; P7 report: time at ĉ_max | P6, ca41546 | Pre-registered *to be reported*, with no hypothesis or threshold, so descriptive. |
| ESO bandwidth sweep (ω_o ∈ {12.5, 25, 50}) | P6, ca41546 | Pre-registered as descriptive only. |
| 40 ms lag and motor18V regressions of the ESO arms | P6, ca41546 | Not covered by P1–P7. |
| Lag-aware ESO diagnostic and motor18V all-controller diagnostic | P6, d1ebcea | Explicitly diagnostic. |

## Not tiered: verification (code correctness), re-run by `pytest -q`
These are correctness checks, not empirical claims:
- model sanity
- trajectory derivatives vs finite differences
- error wrapping
- ξ̇, ξ̈ (eq. 23, 26) and det(HGM0⁻¹B) (eq. 52) vs finite differences
- closed-loop ṡ vs eq. (42) for all switching variants
- ESO convergence, projection and the sized ϱ formula
- the motor plant's steady state and decay
- metric definitions

They support the claims above by showing the code implements the stated equations.

## History rewrite (2026-10-19, before the first push)
`papers/*.pdf` was removed from all history with `git filter-repo --path-glob 'papers/*.pdf' --invert-paths`
(copyright release blocker). Author and commit dates and messages are unchanged (verified with an
identical `git log --format='%ad %s'`), so the ordering evidence above is preserved. **All commit hashes
changed.** This file now cites the new hashes. The frozen `docs/phase6_preregistration.md` still cites
pre-rewrite `692b419` (= new `1b10b13`). A pre-rewrite backup bundle was kept locally (not published).

| pre-rewrite | post-rewrite |
|---|---|
| 692b419 | 1b10b13 |
| fb48539 | 1eabfbf |
| 15a2d05 | 3e595fd |
| dcf0462 | 4a2f004 |
| 69174d3 | 57217c4 |
| b7a8086 | 73f99c1 |
| c6acda3 | addbcca |
| a3f4d5d | b37864e |
| f313678 | c5e9128 |
| 7cc0531 | ca41546 |
| 12c19de | d1ebcea |
| 131cf10 | ea256b3 |
| 0a86dcd | f1f585e |
