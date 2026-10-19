"""Regenerate every figure: python -m experiments.run_all"""

from experiments import (baselines, drift, phase2_open_loop, phase6_diagnostics, phase6_eval,
                         reproduction, robustness)


def main() -> None:
    phase2_open_loop.main()
    baselines.main()
    reproduction.main()
    drift.main()
    robustness.main()
    robustness.main("configs/phase6_omega_sweep.yaml")
    phase6_eval.main()
    phase6_diagnostics.main()


if __name__ == "__main__":
    main()
