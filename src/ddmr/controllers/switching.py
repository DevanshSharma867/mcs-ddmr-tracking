"""Switching functions shared by the sliding-mode controllers."""

from __future__ import annotations

import numpy as np


def sat(x: np.ndarray, width: float) -> np.ndarray:
    """Boundary-layer saturation, paper eq. (55): sgn(x) if |x| > width, else x / width."""
    return np.clip(np.asarray(x, dtype=float) / width, -1.0, 1.0)
