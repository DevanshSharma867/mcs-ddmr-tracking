"""Classic kinematic control (CKC), Kanayama-type outer loop, paper eq. (56)-(57).

    v_d     = v_r cos(phi_e) + k_e1 x_e
    omega_d = omega_r + k_e2 v_r y_e + k_e3 v_r sin(phi_e)

Gains (paper): k_e1 = 1.7, k_e2 = 1.7, k_e3 = 1.5. Implemented in Phase 3.
"""
