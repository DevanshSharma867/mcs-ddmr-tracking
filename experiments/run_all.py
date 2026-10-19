"""Regenerate every figure: python -m experiments.run_all"""

from experiments import baselines, phase2_open_loop, reproduction


def main() -> None:
    phase2_open_loop.main()
    baselines.main()
    reproduction.main()


if __name__ == "__main__":
    main()
