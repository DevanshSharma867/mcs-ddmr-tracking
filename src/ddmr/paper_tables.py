"""Values transcribed from the paper's result tables (Xie et al. 2022), for comparison only.

Units as printed: x_e, y_e RMS/MAX in cm; phi_e RMS/MAX in rad; settling times in s.
"""

# Table 2: circular trajectory with initial offset (scenario S1).
TABLE2 = {
    "ckc": {"x_e": (3.298, 11.604, 6.070), "y_e": (2.436, 10.021, 7.520), "phi_e": (0.015, 0.032, 5.930)},
    "rsmkc": {"x_e": (3.288, 12.115, 4.810), "y_e": (2.528, 10.014, 7.590), "phi_e": (0.023, 0.064, 7.390)},
    "csmdc": {"x_e": (3.304, 17.524, 2.380), "y_e": (3.754, 10.684, 9.360), "phi_e": (0.047, 0.215, 3.980)},
    "afntsmdc": {"x_e": (1.918, 13.019, 0.680), "y_e": (3.159, 10.210, 8.870), "phi_e": (0.030, 0.172, 4.460)},
}
"""Per controller and error: (RMS, MAX, settling time)."""

# Table 3: circular trajectory with load uncertainty (scenario S2).
TABLE3 = {
    "ckc": {"x_e": (3.224, 12.165, 6.490), "y_e": (2.148, 10.046, 8.380), "phi_e": (0.017, 0.041, 6.120)},
    "rsmkc": {"x_e": (3.883, 12.248, 6.280), "y_e": (3.408, 10.015, 9.270), "phi_e": (0.035, 0.075, 9.640)},
    "csmdc": {"x_e": (4.740, 21.436, 1.810), "y_e": (3.933, 10.727, 6.330), "phi_e": (0.065, 0.285, 5.350)},
    "afntsmdc": {"x_e": (2.806, 15.229, 1.120), "y_e": (2.587, 10.015, 7.180), "phi_e": (0.024, 0.125, 3.860)},
}

# Table 4: circular trajectory with disturbance (scenario S3). Window not stated in the paper.
TABLE4 = {
    "ckc": {"x_e": (3.227, 2.827, 1.830), "y_e": (2.144, 0.785, 2.370), "phi_e": (0.004, 0.013, 1.070)},
    "rsmkc": {"x_e": (3.887, 2.695, 0.870), "y_e": (3.407, 0.901, 2.660), "phi_e": (0.006, 0.025, 1.310)},
    "csmdc": {"x_e": (4.748, 2.035, 0.670), "y_e": (3.937, 1.234, 2.280), "phi_e": (0.008, 0.026, 1.080)},
    "afntsmdc": {"x_e": (2.805, 1.624, 0.520), "y_e": (2.682, 1.004, 2.090), "phi_e": (0.003, 0.011, 1.120)},
}
