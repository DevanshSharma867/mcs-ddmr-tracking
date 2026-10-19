"""Tracking-error function and the terms shared by the combined controllers (CSMDC, AFNTSMDC).

    xi      = [x_e, phi_e + rho/(|x_e|+rho) atan(y_e)]^T                        eq. (22)
    xi_dot  = H p_e_dot                                                          eq. (23)
    H       = [[1, 0, 0],
               [-rho (|x_e|+rho)^-2 sgn(x_e) atan(y_e), rho/((|x_e|+rho)(1+y_e^2)), 1]]  eq. (24)
    xi_ddot = H_dot p_e_dot + H (F_dot + G_dot z) + H G M0^-1 (B tau - delta)    eq. (26)
            = drift + D tau - H G M0^-1 delta,     D = H G M0^-1 B (decoupling matrix)

F, G from eq. (19). H_dot is derived analytically treating sgn(x_e) as piecewise constant;
sgn(0) = 0 (M14), which zeroes the sgn-dependent terms at x_e = 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ddmr.errors import error_rate
from ddmr.model import RobotParams
from ddmr.trajectories import RefSample


@dataclass(frozen=True)
class XiTerms:
    xi: np.ndarray        # (2,)  eq. (22)
    xi_dot: np.ndarray    # (2,)  eq. (23)
    H: np.ndarray         # (2,3) eq. (24)
    pe_dot: np.ndarray    # (3,)  eq. (18)
    drift: np.ndarray     # (2,)  H_dot p_e_dot + H (F_dot + G_dot z)
    D: np.ndarray         # (2,2) H G M0^-1 B


def xi_terms(p_e: np.ndarray, z: np.ndarray, ref: RefSample, rho: float,
             params: RobotParams) -> XiTerms:
    """Evaluate eq. (18)-(26) at error p_e, measured z = [v, omega], reference ref."""
    x_e, y_e, phi_e = p_e
    pe_dot = error_rate(p_e, z, ref)
    xd, yd, phid = pe_dot
    sg = float(np.sign(x_e))
    a = abs(x_e) + rho
    a_dot = sg * xd
    at = math.atan(y_e)
    q = 1.0 + y_e**2

    xi = np.array([x_e, phi_e + rho / a * at])
    H = np.array([[1.0, 0.0, 0.0],
                  [-rho * sg * at / a**2, rho / (a * q), 1.0]])
    h21_dot = -rho * sg * (yd / (q * a**2) - 2.0 * at * a_dot / a**3)
    h22_dot = -rho * a_dot / (a**2 * q) - 2.0 * rho * y_e * yd / (a * q**2)
    H_dot = np.array([[0.0, 0.0, 0.0], [h21_dot, h22_dot, 0.0]])

    c, s = math.cos(phi_e), math.sin(phi_e)
    F_dot = np.array([ref.v_dot * c - ref.v * s * phid,
                      ref.v_dot * s + ref.v * c * phid,
                      ref.omega_dot])
    G = np.array([[-1.0, y_e], [0.0, -x_e], [0.0, -1.0]])
    G_dot = np.array([[0.0, yd], [0.0, -xd], [0.0, 0.0]])
    M0_inv = np.diag([1.0 / params.m, 1.0 / params.I])

    return XiTerms(xi=xi, xi_dot=H @ pe_dot, H=H, pe_dot=pe_dot,
                   drift=H_dot @ pe_dot + H @ (F_dot + G_dot @ z),
                   D=H @ G @ M0_inv @ params.B)
