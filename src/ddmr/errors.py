"""Tracking error expressed in the robot frame.

    e1 =  cos(theta) (x_r - x) + sin(theta) (y_r - y)
    e2 = -sin(theta) (x_r - x) + cos(theta) (y_r - y)
    e3 = wrap(theta_r - theta)

Implemented in Phase 2.
"""
