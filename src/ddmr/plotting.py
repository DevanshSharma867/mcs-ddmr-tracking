"""Publication-style figures with consistent per-controller colors, saved as png and pdf."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {
    "reference": "black",
    "kinematic": "0.5",
    "ckc": "tab:red",
    "rsmkc": "tab:purple",
    "csmdc": "tab:orange",
    "csmdc_bl": "tab:brown",
    "afntsmdc": "tab:blue",
    "improved": "tab:green",
}

INPUT_LABEL = "control input (model units; paper: V)"


def setup_style() -> None:
    plt.rcParams.update({
        "figure.dpi": 120, "savefig.dpi": 200, "font.size": 10, "axes.grid": True,
        "grid.alpha": 0.3, "legend.frameon": False, "figure.constrained_layout.use": True,
    })


def save_fig(fig: plt.Figure, stem: str | Path) -> None:
    """Save fig as <stem>.png and <stem>.pdf, creating parent directories."""
    stem = Path(stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(stem.with_suffix(f".{ext}"))
    plt.close(fig)
