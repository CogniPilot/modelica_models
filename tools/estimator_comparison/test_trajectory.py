"""Check climb kinematics and the unchanged horizontal/attitude trajectory."""

import unittest

import numpy as np

from generate import trajectory


class ClimbTests(unittest.TestCase):
    def test_higher_climb_is_consistent_and_preserves_other_motion(self):
        times = np.arange(0, 20, 0.001)
        original = trajectory(times, 0.6)
        higher = trajectory(times, 0.6, 4)
        for index in (0, 1, 2):
            np.testing.assert_array_equal(original[index][:, :2], higher[index][:, :2])
            np.testing.assert_array_equal(
                2 * original[index][:, 2], higher[index][:, 2]
            )
        for index in (3, 4, 7):
            np.testing.assert_array_equal(original[index], higher[index])
        position, velocity, acceleration = higher[:3]
        np.testing.assert_allclose(
            np.gradient(position[:, 2], 0.001), velocity[:, 2], atol=2e-6
        )
        smooth = (np.abs(times - 13) > 0.002) & (np.abs(times - 16) > 0.002)
        np.testing.assert_allclose(
            np.gradient(velocity[:, 2], 0.001)[smooth],
            acceleration[smooth, 2],
            atol=5e-6,
        )
        self.assertEqual(position[-1, 2], 4)
        np.testing.assert_array_equal(position[times < 13], 0)
        np.testing.assert_array_equal(velocity[times < 13], 0)
        self.assertTrue(np.all(higher[-1] < 10))

    def test_invalid_height(self):
        for height in (0, -1, 9, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                trajectory(np.array([0, 15, 20]), 0.6, height)


if __name__ == "__main__":
    unittest.main()
