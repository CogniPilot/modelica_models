"""Verify native noise units against the actual covariance recurrence."""

import unittest

from native_aiding_noise import sensor_informed_noise
from native_noise import PREDICTION_PERIOD_S


class NativeAidingNoiseTests(unittest.TestCase):
    def test_bias_random_walk_in_common_rate_coordinates(self):
        profile = sensor_informed_noise()
        for name, period in PREDICTION_PERIOD_S.items():
            parameters = profile[name]
            for native, target in zip(
                ("ekf2_gyr_b_noise", "ekf2_acc_b_noise")
                if name == "px4"
                else ("EK3_GBIAS_P_NSE", "EK3_ABIAS_P_NSE"),
                ("gyro_bias_psd_rad2_s3", "accel_bias_psd_m2_s5"),
                strict=True,
            ):
                increment_variance = (period * parameters[native]) ** 2
                if name == "ekf3":
                    native_increment_variance = (period**2 * parameters[native]) ** 2
                    self.assertAlmostEqual(
                        native_increment_variance / period**2,
                        increment_variance,
                        delta=increment_variance * 1e-12,
                    )
                self.assertAlmostEqual(
                    increment_variance / period,
                    profile["targets"][target],
                    delta=profile["targets"][target] * 1e-12,
                )

    def test_range_paths_and_gauss_units(self):
        profile = sensor_informed_noise()
        target = profile["targets"]
        self.assertAlmostEqual(
            profile["px4"]["ekf2_rng_noise"] ** 2, target["range_variance_m2"]
        )
        self.assertEqual(profile["ekf3"]["EK3_RNG_M_NSE"], target["range_variance_m2"])
        for name, parameter in (("px4", "ekf2_mag_noise"), ("ekf3", "EK3_MAG_M_NSE")):
            self.assertAlmostEqual(
                profile[name][parameter] * 1e-4, target["magnetic_std_T"]
            )
        self.assertTrue(profile["limitations"])


if __name__ == "__main__":
    unittest.main()
