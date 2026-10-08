"""Check physical flow axes, exposure integration and independent gyro noise."""

import unittest

import numpy as np

from flow_exposure import camera_rates, integrate_windows
from flow_native import ardupilot_rates, verify_px4


class FlowExposureTests(unittest.TestCase):
    def test_linear_exposure_and_missing_support(self):
        times = np.arange(21) * 0.01
        values = np.column_stack((1 + 2 * times, -3 + 4 * times))
        result = integrate_windows(times, values, [0.1, 0.2], 0.1)
        np.testing.assert_allclose(result, [[0.11, -0.28], [0.13, -0.24]], atol=1e-15)
        for endpoints, duration in (
            ([0], 0.1),
            ([0.3], 0.1),
            ([0.105], 0.1),
            ([0.1], 0),
        ):
            with self.assertRaises(ValueError):
                integrate_windows(times, values, endpoints, duration)
        with self.assertRaises(ValueError):
            integrate_windows(times[::-1], values, [0.1], 0.1)
        with self.assertRaises(ValueError):
            integrate_windows(times, values[:, 0], [0.1], 0.1)

    def test_translation_and_rotation_axes_at_native_boundaries(self):
        velocity = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 0]])
        gyro = np.array([[0, 0, 0], [0, 0, 0], [0.2, 0, 0], [0, 0.3, 0]])
        image = camera_rates(velocity, np.full(4, 2), gyro)
        np.testing.assert_allclose(image, [[0, 0.5], [-0.5, 0], [-0.2, 0], [0, -0.3]])
        flow = {
            "integration_time_s": np.full(4, 0.1),
            "integrated_los_x_rad": image[:, 0] * 0.1,
            "integrated_los_y_rad": image[:, 1] * 0.1,
            "integrated_gyro_x_rad": gyro[:, 0] * 0.1,
            "integrated_gyro_y_rad": gyro[:, 1] * 0.1,
        }
        native = np.array([ardupilot_rates(flow, i) for i in range(4)])
        np.testing.assert_allclose(
            native[:, :2], [[0, 0.5], [0.5, 0], [0.2, 0], [0, -0.3]]
        )
        np.testing.assert_allclose(
            -native[:, :2] + native[:, 2:], [[0, -0.5], [-0.5, 0], [0, 0], [0, 0]]
        )

    def test_camera_gyro_noise_is_not_cancelled_from_the_image(self):
        image = camera_rates(np.zeros((1, 3)), np.ones(1), np.array([[0.2, -0.3, 0.1]]))
        measured_gyro = np.array([[0.21, -0.32, 0.1]])
        np.testing.assert_allclose(image + measured_gyro[:, :2], [[0.01, -0.02]])

    def test_px4_api_rejects_epoch_and_sign_errors(self):
        flow = np.zeros(
            1,
            dtype=[
                (key, float)
                for key in (
                    "t_s",
                    "exposure_midpoint_s",
                    "integration_time_s",
                    "integrated_los_x_rad",
                    "integrated_los_y_rad",
                    "integrated_gyro_x_rad",
                    "integrated_gyro_y_rad",
                    "integrated_gyro_z_rad",
                )
            ],
        )
        flow[0] = (1, 0.95, 0.1, 0, 0.05, 0, 0, 0)
        self.assertEqual(verify_px4("FLOW_PACKET 1 .95 0 -.5 0 0 0", flow)["rows"], 1)
        for line in ("", "FLOW_PACKET 1 1 0 -.5 0 0 0", "FLOW_PACKET 1 .95 0 .5 0 0 0"):
            with self.assertRaises(ValueError):
                verify_px4(line, flow)


if __name__ == "__main__":
    unittest.main()
