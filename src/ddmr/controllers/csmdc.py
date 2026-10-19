"""Conventional sliding-mode dynamic control (CSMDC), combined controller, paper eq. (60).

    s_csm = lambda_c xi + xi_dot
    tau   = -(H G M0^{-1} B)^{-1} (lambda_c xi_dot + H_dot p_e_dot + H (F_dot + G_dot z) + K_c sgn(s_csm))

Gains (paper): K_c = diag(21, 16), lambda_c = diag(3, 4). Implemented in Phase 3.
"""
