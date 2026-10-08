"""Known motion and quaternion-sign cases for common-epoch native scoring."""

import unittest

import numpy as np

from native_delay import common_epochs
from score import POSITION, QUATERNION, VELOCITY


class CommonEpochTests(unittest.TestCase):
    def setUp(self):
        fields = ("t_s", *POSITION, *VELOCITY, *QUATERNION, "pos_valid")
        self.states = np.zeros(4, dtype=[(name, float) for name in fields])
        self.states["t_s"] = [0.00625, 0.01625, 0.02625, 0.03625]
        self.states["e_m"] = 2 * self.states["t_s"]
        self.states["ve_m_s"] = 2
        self.states["qw"] = [1, -1, -1, 1]
        self.states["pos_valid"] = [0, 1, 0, 1]

    def test_linear_motion_and_sign_equivalence(self):
        result = common_epochs(self.states, np.array([0.01, 0.02, 0.03]))
        np.testing.assert_allclose(result["e_m"], [0.02, 0.04, 0.06])
        np.testing.assert_allclose(result["ve_m_s"], 2)
        np.testing.assert_allclose(result["qw"], 1)
        np.testing.assert_equal(result["pos_valid"], [0, 1, 0])

    def test_no_extrapolation_or_missing_interval(self):
        with self.assertRaises(ValueError):
            common_epochs(self.states, np.array([0, 0.02]))
        with self.assertRaises(ValueError):
            common_epochs(self.states[[0, 2, 3]], np.array([0.01, 0.02]))

    def test_zero_quaternion_rejected(self):
        self.states["qw"] = 0
        with self.assertRaises(ValueError):
            common_epochs(self.states, np.array([0.01, 0.02]))


if __name__ == "__main__":
    unittest.main()
