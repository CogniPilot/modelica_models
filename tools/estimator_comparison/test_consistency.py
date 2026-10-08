"""Keep delayed covariance paired with the delayed state and truth epoch."""

import unittest

import numpy as np

from consistency import consistency, fusion_horizon_pair
from score import POSITION, QUATERNION, VELOCITY


class FusionEpochTests(unittest.TestCase):
    def setUp(self):
        fields = POSITION + VELOCITY + QUATERNION
        self.estimate = np.zeros(
            5,
            dtype=[(key, float) for key in ("t_s", "fusion_t_s", *fields)]
            + [("horizon_" + key, float) for key in fields],
        )
        self.covariance = np.zeros(
            5,
            dtype=[
                (key, float)
                for key in (
                    "t_s",
                    "fusion_t_s",
                    "fusion_ready",
                    "bgx",
                    "bgy",
                    "bgz",
                    "bax",
                    "bay",
                    "baz",
                    *(f"p{row}_{column}" for row in range(15) for column in range(15)),
                )
            ],
        )
        for array in (self.estimate, self.covariance):
            array["t_s"] = np.arange(5) * 0.1
            array["fusion_t_s"] = array["t_s"] - 0.2
        self.covariance["fusion_ready"] = [0, 0, 1, 1, 1]
        for axis in range(15):
            self.covariance[f"p{axis}_{axis}"] = 1
        self.estimate["qw"] = self.estimate["horizon_qw"] = 1
        self.estimate["ve_m_s"] = self.estimate["horizon_ve_m_s"] = 1
        self.estimate["e_m"] = self.estimate["t_s"]
        self.estimate["horizon_e_m"] = self.estimate["fusion_t_s"]
        self.truth = self.estimate.copy()

    def score(self):
        return consistency(
            self.estimate,
            self.covariance,
            self.truth,
            [0] * 3,
            [0] * 3,
            start=0,
            end=0.3,
            fusion_horizon=True,
        )

    def test_moving_truth_at_delayed_epoch(self):
        result = self.score()
        self.assertEqual(result["rows"], 3)
        self.assertAlmostEqual(result["mean_nees_15d"], 0)
        self.estimate["e_m"] += 100
        self.assertAlmostEqual(self.score()["mean_nees_15d"], 0)
        self.estimate["horizon_e_m"] += 0.25
        self.assertAlmostEqual(self.score()["mean_nees_15d"], 0.25**2)

    def test_mixed_epochs_and_unready_data_refused(self):
        with self.assertRaisesRegex(ValueError, "explicit fusion-horizon"):
            consistency(self.estimate, self.covariance, self.truth, [0] * 3, [0] * 3)
        for field, replacement in (
            ("t_s", np.arange(5)),
            ("fusion_t_s", np.arange(5)),
            ("fusion_ready", np.zeros(5)),
            ("fusion_ready", np.full(5, float("nan"))),
        ):
            broken = self.covariance.copy()
            broken[field] = replacement
            with self.assertRaises(ValueError):
                fusion_horizon_pair(self.estimate, broken)
        broken = self.estimate.copy()
        broken["fusion_t_s"][3] = broken["fusion_t_s"][2]
        covariance = self.covariance.copy()
        covariance["fusion_t_s"] = broken["fusion_t_s"]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            fusion_horizon_pair(broken, covariance)

    def add_root(self):
        rooted = np.zeros(
            len(self.covariance),
            dtype=self.covariance.dtype.descr
            + [
                (f"l{row}_{column}", float) for row in range(15) for column in range(15)
            ],
        )
        for name in self.covariance.dtype.names:
            rooted[name] = self.covariance[name]
        for axis in range(15):
            rooted[f"l{axis}_{axis}"] = 1
        self.covariance = rooted

    def test_root_whitening_at_fusion_epoch(self):
        self.add_root()
        self.estimate["horizon_e_m"] += 0.25
        self.covariance["l0_0"] = 2
        self.covariance["p0_0"] = 4
        result = self.score()
        self.assertAlmostEqual(result["mean_nees_15d"], 0.25**2 / 4)
        self.assertEqual(result["covariance_representation"], "square_root")
        self.assertEqual(result["dense_covariance_non_pd_samples"], 0)

    def test_root_refuses_invalid_or_mismatched_factors(self):
        self.add_root()
        original = self.covariance.copy()
        for field, value in (("l0_0", 0), ("l0_1", 1), ("l0_0", np.nan), ("p0_0", 2)):
            self.covariance = original.copy()
            self.covariance[field] = value
            with self.assertRaises(ValueError):
                self.score()
        self.covariance = original[
            [name for name in original.dtype.names if name != "l0_0"]
        ]
        with self.assertRaisesRegex(ValueError, "Incomplete"):
            self.score()

    def test_dense_rounding_failure_is_retained(self):
        self.add_root()
        self.covariance["l1_0"] = 1
        self.covariance["l1_1"] = 1e-4
        for field in ("p0_1", "p1_0", "p1_1"):
            self.covariance[field] = 1
        self.estimate["horizon_e_m"] += 0.25
        result = self.score()
        self.assertEqual(result["dense_covariance_non_pd_samples"], 3)
        self.assertAlmostEqual(result["mean_nees_15d"], 0.25**2 * (1 + 1e8), places=2)


if __name__ == "__main__":
    unittest.main()
