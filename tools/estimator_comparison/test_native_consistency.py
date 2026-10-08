"""Validate native tangent conventions with physical finite perturbations."""

import unittest
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace

import numpy as np

from check_imu_coning import quaternion_product
from check_preintegration_noise import local_error
from native_consistency import check, common_state_and_jacobian
from score import POSITION, QUATERNION, VELOCITY


class NativeCovarianceCoordinatesTests(unittest.TestCase):
    def setUp(self):
        self.state = np.r_[
            [1, -2, 3],
            [-1, 0.3, 0.8],
            np.array([0.7, -0.2, 0.3, 0.6]) / np.linalg.norm([0.7, -0.2, 0.3, 0.6]),
            [0.02, -0.01, 0.03],
            [-0.1, 0.04, 0.07],
        ]

    def perturb(self, name, index, amount):
        state = self.state.copy()
        if name == "px4":
            if index < 3:
                vector = np.eye(3)[index] * np.sin(amount / 2)
                state[6:10] = quaternion_product(
                    np.r_[np.cos(amount / 2), vector], state[6:10]
                )
            elif index < 15:
                offset = {3: 3, 6: 0, 9: 10, 12: 13}[3 * (index // 3)]
                state[offset + index % 3] += amount
        else:
            if index < 4:
                state[6 + index] += amount
            elif index < 16:
                offset = {4: 3, 7: 0, 10: 10, 13: 13}
                start = next(start for start in offset if start <= index < start + 3)
                state[offset[start] + index - start] += amount
        return state

    def test_full_covariance_map_matches_physical_differentials(self):
        for name, period in (("px4", 1), ("ekf3", 0.0125)):
            common, jacobian = common_state_and_jacobian(self.state, name, period)
            for index in range(24):
                samples = [
                    common_state_and_jacobian(
                        self.perturb(name, index, sign * 1e-7), name, period
                    )[0]
                    for sign in (-1, 1)
                ]
                differences = [
                    np.r_[
                        local_error(common[:10], sample[:10]), sample[10:] - common[10:]
                    ]
                    for sample in samples
                ]
                numeric = (differences[1] - differences[0]) / 2e-7
                np.testing.assert_allclose(
                    numeric,
                    jacobian[:, index],
                    atol=2e-8,
                    rtol=1e-7,
                    err_msg=f"{name} component {index}",
                )

    def test_nees_is_invariant_under_px4_coordinate_change(self):
        _, jacobian = common_state_and_jacobian(self.state, "px4", 1)
        transform = jacobian[:, :15]
        rng = np.random.default_rng(73)
        root = rng.normal(size=(15, 15))
        covariance = root @ root.T + np.eye(15)
        error = rng.normal(size=15)
        transformed = transform @ error
        np.testing.assert_allclose(
            transformed
            @ np.linalg.solve(transform @ covariance @ transform.T, transformed),
            error @ np.linalg.solve(covariance, error),
            rtol=1e-12,
        )

    def test_quaternion_radial_noise_does_not_become_attitude_noise(self):
        _, jacobian = common_state_and_jacobian(self.state, "ekf3", 0.0125)
        radial = np.r_[self.state[6:10], np.zeros(20)]
        np.testing.assert_allclose(jacobian @ radial, 0, atol=1e-15)
        for period in (0, -1, np.nan):
            with self.assertRaises(ValueError):
                common_state_and_jacobian(self.state, "ekf3", period)
        with self.assertRaisesRegex(ValueError, "rates"):
            common_state_and_jacobian(self.state, "px4", 0.0125)

    def test_changed_posteriors_at_repeated_epochs_are_retained(self):
        state = self.state.copy()
        period = 0.0125
        state[10:] = (
            np.diag([1, -1, -1, 1, -1, -1])
            @ np.r_[[0.0008, -0.0005, 0.0004], [0.02, -0.01, 0.015]]
            * period
        )
        common, _ = common_state_and_jacobian(state, "ekf3", period)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "native.csv"
            times = np.sort(np.r_[np.arange(10, 60, 0.5), 35.5])
            with capture.open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    [
                        "publication_us",
                        "fusion_us",
                        "bias_period_s",
                        *(f"s{i}" for i in range(16)),
                        *(f"p{r}_{c}" for r in range(24) for c in range(24)),
                    ]
                )
                for index, timestamp in enumerate(times):
                    observed = state.copy()
                    if index and times[index - 1] == timestamp:
                        observed[0] += 1
                    writer.writerow(
                        [
                            (timestamp + 0.2) * 1e6,
                            timestamp * 1e6,
                            period,
                            *observed,
                            *np.eye(24).flat,
                        ]
                    )
            truth = root / "truth.csv"
            with truth.open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(["t_s", *POSITION, *VELOCITY, *QUATERNION])
                for timestamp in np.arange(61):
                    writer.writerow([timestamp, *common[:10]])
            evidence = root / "result.json"
            with contextlib.redirect_stdout(io.StringIO()):
                check(
                    SimpleNamespace(
                        filter="ekf3", covariance=capture, truth=truth, output=evidence
                    )
                )
            result = json.loads(evidence.read_text())
            self.assertEqual(result["repeated_changed_posterior_rows"], 1)
            self.assertEqual(result["scored_observer_rows"], len(times))
            self.assertEqual(result["windows"]["outage"]["rows"], 31)
            self.assertGreater(result["windows"]["outage"]["mean_nees_15d"], 0)


if __name__ == "__main__":
    unittest.main()
