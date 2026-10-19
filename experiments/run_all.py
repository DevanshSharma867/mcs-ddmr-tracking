"""Regenerate every result and figure from scratch: python -m experiments.run_all

Order matters: report_figures reads the outputs of all earlier experiments.
"""

from experiments import (baselines, drift, phase2_open_loop, phase6_diagnostics, phase6_eval,
                         report_figures, reproduction, robustness, sign_verification)


def main() -> None:
    phase2_open_loop.main()
    baselines.main()
    reproduction.main()
    drift.main()
    robustness.main()
    robustness.main("configs/phase6_omega_sweep.yaml")
    phase6_eval.main()
    phase6_diagnostics.main()
    sign_verification.main()
    report_figures.main()


if __name__ == "__main__":
    main()
