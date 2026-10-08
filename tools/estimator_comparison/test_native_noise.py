"""Check density conversion against the variance of accumulated white samples."""

import math
import unittest

from native_noise import PREDICTION_PERIOD_S, native_noise


class NativeNoiseTests(unittest.TestCase):
    def test_white_sample_accumulation(self):
        sample_std = (0.0015, 0.03)
        sample_period = 1 / 800
        density = [value * math.sqrt(sample_period) for value in sample_std]
        rates = native_noise(density)
        for name, period in PREDICTION_PERIOD_S.items():
            samples = round(period / sample_period)
            for measured, raw_std in zip(rates[name], sample_std, strict=True):
                accumulated_variance = samples * (raw_std * sample_period) ** 2
                self.assertAlmostEqual(measured**2 * period**2, accumulated_variance)
                self.assertAlmostEqual(measured, raw_std / math.sqrt(samples))

    def test_invalid_or_clamped_configuration_refused(self):
        for density in (
            [],
            [1],
            [0, 1],
            [-1, 1],
            [float("nan"), 1],
            [1, float("inf")],
            [1, 1],
            [0.01, 1],
        ):
            with self.assertRaises(ValueError):
                native_noise(density)


if __name__ == "__main__":
    unittest.main()
