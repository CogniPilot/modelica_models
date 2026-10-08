"""White-noise calibration, bias invariance and invalid capture refusal."""

import unittest

import numpy as np

from calibrate_imu_noise import calibrate


class ImuNoiseTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(20261007)
        time = np.arange(40001) / 800
        self.samples = np.column_stack(
            (
                time,
                rng.normal(size=(len(time), 6)) * np.array([0.0015] * 3 + [0.03] * 3),
            )
        )

    def test_white_noise_density_and_bias_invariance(self):
        result = calibrate(self.samples, 0, 50)
        expected = np.array([0.0015, 0.03]) / np.sqrt(800)
        np.testing.assert_allclose(result["imu_noise_density"], expected, rtol=0.02)
        shifted = self.samples.copy()
        shifted[:, 1:] += [0.003, -0.002, 0.001, 0.2, -0.1, 9.81]
        np.testing.assert_allclose(
            calibrate(shifted, 0, 50)["imu_noise_density"],
            result["imu_noise_density"],
            rtol=1e-12,
        )

    def test_bad_window_and_capture_refused(self):
        for start, end in [(0, 0), (1, 0), (-1, 5), (0, 51), (0, 0.1), (0, np.inf)]:
            with self.subTest(window=(start, end)), self.assertRaises(ValueError):
                calibrate(self.samples, start, end)
        for defect in ("nan", "duplicate", "irregular", "columns"):
            samples = self.samples.copy()
            if defect == "nan":
                samples[200, 1] = np.nan
            elif defect == "duplicate":
                samples[200, 0] = samples[199, 0]
            elif defect == "irregular":
                samples[200, 0] += 0.0001
            else:
                samples = samples[:, :6]
            with self.subTest(defect=defect), self.assertRaises(ValueError):
                calibrate(samples, 0, 50)


if __name__ == "__main__":
    unittest.main()
