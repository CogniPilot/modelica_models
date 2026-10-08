"""Check that warmup shifts motion and every comparison phase together."""

import unittest

import numpy as np

from generate import trajectory
from mission import Mission
from score import transition


class MissionTests(unittest.TestCase):
    def test_shifted_motion_and_windows(self):
        original, shifted = Mission(), Mission(120)
        self.assertEqual(shifted.offset_s, 107)
        for attribute in ("end_s", "loss_s", "return_s"):
            self.assertEqual(
                getattr(shifted, attribute) - getattr(original, attribute), 107
            )
        for before, after in zip(original.windows, shifted.windows, strict=True):
            self.assertEqual(before[0], after[0])
            np.testing.assert_allclose(
                np.array(after[1:]) - before[1:], 107, atol=2e-14, rtol=0
            )
        times = np.arange(0, 6001) / 100
        for before, after in zip(
            trajectory(times, 0.6),
            trajectory(times + 107, 0.6, warmup_s=120),
            strict=True,
        ):
            np.testing.assert_allclose(before, after, atol=1e-12, rtol=1e-12)

    def test_invalid_warmup(self):
        for warmup in (0, 12, 601, 13.001, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                Mission(warmup)

    def test_recovery_is_relative_to_shifted_return(self):
        fields = (
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
            "gps_fused",
        )
        data = np.zeros(6001, dtype=[(field, float) for field in fields])
        data["t_s"] = np.arange(6001) / 100
        data["qw"] = 1
        data["gps_fused"] = data["t_s"] >= 40.2
        baseline = transition(data, data)
        data["t_s"] += 107
        shifted = transition(data, data, 107)
        for key in baseline:
            self.assertAlmostEqual(baseline[key], shifted[key], places=12)


if __name__ == "__main__":
    unittest.main()
