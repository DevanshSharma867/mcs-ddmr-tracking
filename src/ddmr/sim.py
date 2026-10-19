"""Closed-loop simulator: fixed-step RK4 plant integration with zero-order-hold control.

Timing uses integer counters only: the controller runs at ticks t_k = k * n_sub * dt with
n_sub = Ts / dt (must be an integer); the plant takes n_sub RK4 steps of dt per tick, step j at
t_j = j * dt. Adaptive solvers are deliberately not used (sliding-mode switching).

Torque controllers: u_cmd -> saturation -> held over the tick -> plant (disturbance added inside).
Velocity controllers (ideal kinematic reference): z := u_cmd exactly, kinematics only.
One log row per controller tick, errors computed from the true (not measured) state.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np
import pandas as pd

from ddmr.controllers.base import Controller
from ddmr.errors import tracking_error
from ddmr.model import MeasurementNoise, Plant, kinematics
from ddmr.trajectories import Trajectory

STATE_NAMES = ("x", "y", "phi", "v", "omega")


def steps_ratio(a: float, b: float, what: str) -> int:
    """Return round(a / b), asserting a / b is within 1e-9 of an integer."""
    n = round(a / b)
    assert n >= 1 and abs(a / b - n) < 1e-9, f"{what} = {a / b} is not an integer"
    return n


def rk4_step(f: Callable[[float, np.ndarray], np.ndarray], t: float, x: np.ndarray,
             dt: float) -> np.ndarray:
    """One classical fourth-order Runge-Kutta step of x' = f(t, x)."""
    k1 = f(t, x)
    k2 = f(t + dt / 2, x + dt / 2 * k1)
    k3 = f(t + dt / 2, x + dt / 2 * k2)
    k4 = f(t + dt, x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(plant: Plant, controller: Controller, trajectory: Trajectory, x0: np.ndarray,
             T: float, Ts: float, dt: float,
             noise: MeasurementNoise | None = None) -> pd.DataFrame:
    """Run one closed-loop simulation over [0, T] and return the per-tick log."""
    n_sub = steps_ratio(Ts, dt, "Ts/dt")
    n_ticks = steps_ratio(T, Ts, "T/Ts")
    state = np.asarray(x0, dtype=float).copy()
    controller.reset()
    rows = []
    for k in range(n_ticks + 1):
        t = k * n_sub * dt
        meas = state.copy() if noise is None else noise.measure(state)
        ref = trajectory.sample(t)
        u_cmd = np.asarray(controller.compute(t, meas, ref), dtype=float)
        velocity_mode = controller.output == "velocity"
        u_app = u_cmd if velocity_mode else plant.saturate(u_cmd)
        d = np.zeros(2) if velocity_mode else plant.disturbance(t)
        p_e = tracking_error(state, ref.pose)
        rows.append({
            "t": t,
            **dict(zip(STATE_NAMES, state)),
            **{f"{n}_meas": val for n, val in zip(STATE_NAMES, meas)},
            "x_r": ref.x, "y_r": ref.y, "phi_r": ref.phi, "v_r": ref.v, "omega_r": ref.omega,
            "x_e": p_e[0], "y_e": p_e[1], "phi_e": p_e[2],
            "u_R": u_cmd[0], "u_L": u_cmd[1], "ua_R": u_app[0], "ua_L": u_app[1],
            "d_R": d[0], "d_L": d[1],
        })
        if k == n_ticks:
            break
        for i in range(n_sub):
            tj = (k * n_sub + i) * dt
            if velocity_mode:
                state[3:5] = u_app
                state[:3] = rk4_step(lambda _t, p: kinematics(p, u_app), tj, state[:3], dt)
            else:
                state = rk4_step(lambda _t, s: plant.deriv(_t, s, u_app), tj, state, dt)
        if not np.all(np.isfinite(state)):
            raise FloatingPointError(f"non-finite state {state} at t = {(k + 1) * n_sub * dt}")
    return pd.DataFrame(rows)


def save_log(df: pd.DataFrame, path: str | Path) -> None:
    """Write a simulation log to CSV, creating parent directories."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(path, index=False)
