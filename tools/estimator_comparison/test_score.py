"""Independent known-error checks for the comparison's scoring conventions."""

import unittest

import numpy as np

from score import errors, metrics


class ScoringTests(unittest.TestCase):
    def setUp(self):
        fields = [
            "t_s",
            "e_m",
            "n_m",
            "u_m",
            "ve_m_s",
            "vn_m_s",
            "vu_m_s",
            "qw",
            "qx",
            "qy",
            "qz",
            "pos_valid",
            "att_valid",
        ]
        self.truth = np.zeros(100, dtype=[(name, float) for name in fields])
        self.truth["t_s"] = np.arange(100) / 100
        self.truth["qw"] = 1
        self.estimate = self.truth.copy()

    def test_vector_rmse_and_invalid_states(self):
        for names in [("e_m", "n_m", "u_m"), ("ve_m_s", "vn_m_s", "vu_m_s")]:
            for name, value in zip(names, [3, 4, 12]):
                self.estimate[name] = value
        # All states deliberately invalid: finite error must still be scored.
        result = metrics(self.estimate, self.truth, 0, 1)
        self.assertEqual(result["horizontal_position_rmse_m"], 5)
        self.assertEqual(result["vertical_position_rmse_m"], 12)
        self.assertEqual(result["position_3d_rmse_m"], 13)
        self.assertEqual(result["velocity_3d_rmse_m_s"], 13)
        self.assertEqual(result["position_valid_fraction"], 0)
        self.assertEqual(result["row_coverage"], 1)

    def test_quaternion_sign_and_wrapped_heading(self):
        self.estimate["qw"] = -1
        self.assertTrue(np.allclose(errors(self.estimate, self.truth)[2], 0))
        for data, degrees in [(self.truth, 179), (self.estimate, -179)]:
            data["qw"] = np.cos(np.deg2rad(degrees) / 2)
            data["qz"] = np.sin(np.deg2rad(degrees) / 2)
        _, _, angle, yaw = errors(self.estimate, self.truth)
        self.assertTrue(np.allclose(angle, 2))
        self.assertTrue(np.allclose(yaw, 2))

    def test_no_extrapolation(self):
        self.estimate["t_s"] += 0.01
        with self.assertRaisesRegex(ValueError, "outside truth"):
            errors(self.estimate, self.truth)

    def test_nonfinite_output_fails_accuracy(self):
        self.estimate["e_m"][50] = np.nan
        result = metrics(self.estimate, self.truth, 0, 1)
        self.assertEqual(result["finite_fraction"], 0.99)
        self.assertNotIn("horizontal_position_rmse_m", result)


if __name__ == "__main__":
    unittest.main()
