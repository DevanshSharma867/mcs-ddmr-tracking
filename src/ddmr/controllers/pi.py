"""PI velocity inner loop for the cascaded baselines (CKC, RSMKC), paper eq. (59).

    u = k_P z_e + k_I * integral(z_e dt),   z_e = z_d - z
    u1 -> right wheel, u2 -> left wheel (model units; ASSUMED, reproduction_log M12)

Gains (paper): k_P = 30, k_I = 250. Clamping anti-windup at the saturation limit. Implemented in Phase 3.
"""
