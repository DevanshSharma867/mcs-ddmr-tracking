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
