"""Kanayama-type kinematic (outer-loop) controller.

    v_c     = v_r cos(e3) + k1 e1
    omega_c = omega_r + k2 v_r e2 + k3 v_r sin(e3)

Implemented in Phase 3.
"""
