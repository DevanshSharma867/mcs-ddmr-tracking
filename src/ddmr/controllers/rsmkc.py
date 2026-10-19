"""Robust sliding-mode kinematic control (RSMKC, Nath et al. [15]), outer loop, paper eq. (58).

    s_r = [k_r1 phi_e + v_r atan(y_e), x_e]^T
    z_d = -B_r^{-1} (A_r F_r + D_r + K_r sgn(s_r))

Gains (paper): k_r1 = 0.40, k_r2 = 0.02, k_r3 = 0.02. Implemented in Phase 3.
"""
