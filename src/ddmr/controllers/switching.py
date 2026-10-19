"""Switching functions shared by the sliding-mode controllers."""

from __future__ import annotations

import numpy as np


def sat(x: np.ndarray, width: float) -> np.ndarray:
    """Boundary-layer saturation, paper eq. (55): sgn(x) if |x| > width, else x / width."""
    return np.clip(np.asarray(x, dtype=float) / width, -1.0, 1.0)


EPS_POW = 1e-6
"""|x| is clipped to at least EPS_POW before fractional powers (M14). Avoids 0**negative and
undefined derivatives at 0; for positive exponents it changes the value at x = 0 only from 0 to
EPS_POW**a (e.g. ~1e-4 for a = 0.67), which is negligible."""


def sig(x: np.ndarray, a: float) -> np.ndarray:
    """sig(x)^a = sgn(x) |x|^a, element-wise, with sgn(0) = 0 and |x| clipped at EPS_POW (M14)."""
    x = np.asarray(x, dtype=float)
    return np.sign(x) * np.maximum(np.abs(x), EPS_POW) ** a


def abs_pow(x: np.ndarray, a: float) -> np.ndarray:
    """Element-wise |x|^a (the paper's floor-bracket notation), |x| clipped at EPS_POW (M14)."""
    return np.maximum(np.abs(np.asarray(x, dtype=float)), EPS_POW) ** a
