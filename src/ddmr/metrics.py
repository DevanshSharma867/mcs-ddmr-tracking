"""Performance metrics on a simulation log (one row per controller tick).

Paper table metrics (Tables 2-4), per error x_e [m], y_e [m], phi_e [rad]:
    RMS, MAX = max |e|, settling time.
Settling time (ASSUMED, M6): with bands |x_e|, |y_e| <= 1 cm and |phi_e| <= 0.02 rad, the time
after which the error stays inside its band = the tick following the last tick outside the band.
0 if never outside; NaN ("not settled") if still outside at the last tick.

CLAUDE.md extras, on the applied (saturated) wheel inputs u_a held over each tick Ts:
    effort      = sum_k ||u_a,k||^2 Ts                    (model units^2 s)
    chattering  = sum_k sum_wheels |u_a,k+1 - u_a,k| / duration   (total variation per second)
    saturated % = % of ticks with |u_cmd| > u_max on either wheel
"""

from __future__ import annotations

import numpy as np
import pandas as pd

BANDS = {"x_e": 0.01, "y_e": 0.01, "phi_e": 0.02}


def settling_time(t: np.ndarray, e: np.ndarray, band: float) -> float:
    """Time after which |e| <= band for the rest of the record (see module docstring)."""
    outside = np.flatnonzero(np.abs(e) > band)
    if outside.size == 0:
        return 0.0
    last = outside[-1]
    return float("nan") if last == len(e) - 1 else float(t[last + 1])


def _window(df: pd.DataFrame, window: tuple[float, float] | None) -> pd.DataFrame:
    if window is None:
        return df
    return df[(df["t"] >= window[0] - 1e-9) & (df["t"] <= window[1] + 1e-9)]


def tracking_metrics(df: pd.DataFrame,
                     window: tuple[float, float] | None = None) -> dict[str, dict[str, float]]:
    """RMS, MAX and settling time of x_e, y_e, phi_e (SI units), optionally on a time window."""
    w = _window(df, window)
    t = w["t"].to_numpy()
    out = {}
    for name, band in BANDS.items():
        e = w[name].to_numpy()
        out[name] = {"rms": float(np.sqrt(np.mean(e**2))), "max": float(np.max(np.abs(e))),
                     "settle": settling_time(t, e, band)}
    return out


def input_metrics(df: pd.DataFrame, Ts: float, u_max: float | None,
                  window: tuple[float, float] | None = None) -> dict[str, float]:
    """Control effort, chattering index and % of time saturated (torque controllers only)."""
    w = _window(df, window)
    ua = w[["ua_R", "ua_L"]].to_numpy()
    duration = len(ua) * Ts
    sat = 0.0
    if u_max is not None:
        sat = 100.0 * float(np.mean(np.any(np.abs(w[["u_R", "u_L"]].to_numpy()) > u_max, axis=1)))
    return {"effort": float(np.sum(ua**2) * Ts),
            "chattering": float(np.sum(np.abs(np.diff(ua, axis=0))) / duration),
            "saturated_pct": sat}


def markdown_table(df: pd.DataFrame, floatfmt: str = ".3f") -> str:
    """Render a DataFrame as a GitHub markdown table (no extra dependency)."""
    def cell(v: object) -> str:
        return format(v, floatfmt) if isinstance(v, float) else str(v)

    lines = ["| " + " | ".join(df.columns) + " |", "|" + "---|" * len(df.columns)]
    lines += ["| " + " | ".join(cell(v) for v in row) + " |" for row in df.itertuples(index=False)]
    return "\n".join(lines)
