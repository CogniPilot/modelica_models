"""Reject startup-only GPS transitions and accidental GPS use in denied runs."""

import csv
from pathlib import Path
import tempfile
import unittest

from mission import Mission
from native_readiness import check
from compare_readiness_covariance import validate_pilot


class ReadinessTests(unittest.TestCase):
    def evaluate(self, times, name="ekf3", scenario="transition"):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "innovations.csv"
            with path.open("w") as stream:
                writer = csv.writer(stream)
                writer.writerow(("fusion_us", "sensor", "stage", "fused", "axis"))
                writer.writerow((1_000_000, "barometer", "fusion", 1, 0))
                for time in times:
                    for sensor in ("gps_position", "gps_velocity"):
                        writer.writerow((round(time * 1e6), sensor, "fusion", 1, 0))
            return check(path, name, scenario, Mission(120))

    def test_gps_must_establish_before_flight_and_loss(self):
        self.assertTrue(self.evaluate([36.7, 125, 147.2])["qualified"])
        self.assertFalse(self.evaluate([147.2])["qualified"])
        self.assertFalse(self.evaluate([36.7, 125])["qualified"])

    def test_px4_clock_must_be_removed_before_phase_assignment(self):
        result = self.evaluate([37.7, 132.5, 148.2], "px4")
        self.assertTrue(result["qualified"])
        self.assertAlmostEqual(result["gps"]["gps_position"]["first_fusion_s"], 36.7)
        self.assertFalse(self.evaluate([37.7, 132.5, 148.2])["qualified"])

    def test_denied_never_accepts_gps(self):
        self.assertTrue(self.evaluate([], scenario="denied")["qualified"])
        self.assertFalse(self.evaluate([36.7], scenario="denied")["qualified"])

    def test_covariance_diagnostics_cannot_validate_empty_or_duplicate_campaigns(self):
        row = dict(name="px4", scenario="gps")
        for scores in ([], [row, row], [dict(name="unknown", scenario="gps")]):
            with self.assertRaises(ValueError):
                validate_pilot(
                    dict(complete=False, scores=scores), allow_incomplete=True
                )
        self.assertEqual(
            validate_pilot(dict(complete=False, scores=[row]), True), [row]
        )
        with self.assertRaisesRegex(ValueError, "complete declared"):
            validate_pilot(dict(complete=False, scores=[row]))

    def test_declared_campaign_needs_every_filter_in_every_scenario(self):
        scores = [
            dict(name=name, scenario=scenario)
            for name in (
                "horizon",
                "retrodiction",
                "horizon_joint",
                "retrodiction_joint",
                "px4",
                "ekf3",
            )
            for scenario in ("gps", "denied", "transition")
        ]
        self.assertEqual(len(validate_pilot(dict(complete=True, scores=scores))), 6)
        with self.assertRaisesRegex(ValueError, "complete declared"):
            validate_pilot(dict(complete=True, scores=scores[:-1]))


if __name__ == "__main__":
    unittest.main()
