"""Check actual common-capture draws, metadata and native vertical variance."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from flow_exposure import generate as exposure, integrate_windows
from check_preintegration_noise import rotation
from generate import generate, trajectory
from native_aiding_noise import configured_noise
from score import read
from sensor_noise import COMMON_NOISE_PROFILE, COMMON_SENSOR_NOISE


class SensorNoiseTests(unittest.TestCase):
    def test_native_vertical_velocity_uses_px4_scale(self):
        noise = configured_noise(SimpleNamespace(noise_profile=COMMON_NOISE_PROFILE))
        self.assertAlmostEqual(
            noise["ekf3"]["EK3_VELD_M_NSE"] ** 2,
            (1.5 * noise["px4"]["ekf2_gps_v_noise"]) ** 2,
        )
        self.assertEqual(noise["targets"]["gps_vertical_velocity_std_m_s"], 0.075)
        for groups in ([], ["gps"]):
            with self.assertRaises(ValueError):
                configured_noise(
                    SimpleNamespace(
                        noise_profile=COMMON_NOISE_PROFILE, noise_groups=groups
                    )
                )

    def test_capture_noise_matches_declared_packet_covariance(self):
        with tempfile.TemporaryDirectory() as directory:
            source, capture = Path(directory) / "raw", Path(directory) / "capture"
            generate(source, seed=20261008, speed=0.6, common_native_floors=True)
            exposure(source, capture)
            gps, flow, mag, imu = (
                read(capture / (name + ".csv"))
                for name in ("gps", "flow", "mag", "imu")
            )
            gps_truth = trajectory(gps["t_s"], 0.6)
            velocity = np.column_stack(
                [gps[field] for field in ("ve_m_s", "vn_m_s", "vd_m_s")]
            )
            velocity[:, 2] *= -1
            np.testing.assert_allclose(
                np.std(velocity - gps_truth[1], axis=0),
                COMMON_SENSOR_NOISE.gps_velocity_m_s,
                rtol=0.1,
            )
            magnetic = np.column_stack(
                [mag[field] for field in ("bx_T", "by_T", "bz_T")]
            )
            np.testing.assert_allclose(
                np.std(magnetic - trajectory(mag["t_s"], 0.6)[7], axis=0),
                COMMON_SENSOR_NOISE.magnetic_T,
                rtol=0.1,
            )
            position, velocity, _, quaternion, *_ = trajectory(imu["t_s"], 0.6)
            attitude = np.stack([rotation(q) for q in quaternion])
            body_velocity = np.einsum("nji,nj->ni", attitude, velocity)
            distance = (position[:, 2] + 1) / attitude[:, 2, 2]
            compensated_rate = (
                np.column_stack((-body_velocity[:, 1], body_velocity[:, 0]))
                / distance[:, None]
            )
            ideal = integrate_windows(imu["t_s"], compensated_rate, flow["t_s"], 0.1)
            actual = np.column_stack(
                [
                    flow["integrated_los_" + axis + "_rad"]
                    + flow["integrated_gyro_" + axis + "_rad"]
                    for axis in ("x", "y")
                ]
            )
            variance = np.column_stack(
                [
                    flow["los_variance_" + axis + "_rad2"]
                    + flow["gyro_variance_" + axis + "_rad2"]
                    for axis in ("x", "y")
                ]
            )
            np.testing.assert_allclose(
                variance / 0.1**2, COMMON_SENSOR_NOISE.flow_rate_rad_s**2, rtol=1e-10
            )
            np.testing.assert_allclose(
                np.std((actual - ideal) / 0.1, axis=0),
                COMMON_SENSOR_NOISE.flow_rate_rad_s,
                rtol=0.1,
            )
            np.testing.assert_allclose(
                np.mean((actual - ideal) / 0.1, axis=0), 0, atol=0.007
            )
            np.testing.assert_allclose(
                flow["vx_flu_m_s"], actual[:, 1] / 0.1 * flow["dist_m"], atol=1e-10
            )
            np.testing.assert_allclose(
                flow["vy_flu_m_s"], -actual[:, 0] / 0.1 * flow["dist_m"], atol=1e-10
            )


if __name__ == "__main__":
    unittest.main()
