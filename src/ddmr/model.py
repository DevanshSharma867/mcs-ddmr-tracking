"""Plant model of the differential-drive robot.

Kinematics (q = [x, y, theta], body velocities [v, omega]):
    x_dot = v cos(theta),  y_dot = v sin(theta),  theta_dot = omega

Dynamics (wheel torques tau_R, tau_L in N*m; final form to be taken from the paper):
    m v_dot     = (tau_R + tau_L) / r + d_v(t)
    I omega_dot = b (tau_R - tau_L) / r + d_omega(t)

Also hosts parameter mismatch, disturbance injection and actuator saturation.
Implemented in Phase 2.
"""
