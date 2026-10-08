"""Distinguish a native startup hold from a newly triggered bad-IMU event."""

import csv
from pathlib import Path
import tempfile
import unittest

from diagnose_ekf3_imu_integrity import FIELDS, diagnose


class NativeImuIntegrityTests(unittest.TestCase):
    def pair(
        self,
        time_ms,
        elapsed_ms,
        threshold,
        error,
        bad_before,
        bad_after,
        last_bad_after,
        last_good_before,
        last_good_after,
    ):
        before = dict.fromkeys(FIELDS, 0)
        before.update(
            publication_us=time_ms * 1000,
            fusion_us=(time_ms - 200) * 1000,
            imu_sample_ms=time_ms,
            gps_sample_ms=time_ms - 200,
            baro_sample_ms=time_ms - 200,
            height_error_m=error,
            vertical_velocity_error_m_s=error,
            height_observation_variance=0.01,
            velocity_observation_variance=0.01,
            threshold_multiplier=threshold,
            time_since_bad_ms=elapsed_ms,
            last_bad_ms=time_ms - elapsed_ms,
            last_good_ms=last_good_before,
            bad_imu_data=bad_before,
            estimated_velocity_down_m_s=error,
            estimated_position_down_m=error,
            height_source=1,
            gps_data_to_fuse=1,
        )
        after = dict(
            before,
            last_bad_ms=last_bad_after,
            last_good_ms=last_good_after,
            bad_imu_data=bad_after,
            estimated_velocity_down_m_s=0 if bad_after else error,
        )
        return [dict(stage="before", **before), dict(stage="after", **after)]

    def analyze(self, rows):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "trace.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=("stage", *FIELDS), lineterminator="\n"
                )
                writer.writeheader()
                writer.writerows(rows)
            return diagnose(path)

    def test_zero_timer_activates_hold_without_a_trigger(self):
        result = self.analyze(self.pair(1000, 1000, 1, 0, 0, 1, 0, 0, 1000))
        self.assertEqual(result["trigger_conditions"], 0)
        self.assertIsNone(result["first_trigger_condition"])
        self.assertFalse(result["first_false_to_true_activation"]["trigger_condition"])
        self.assertEqual(
            result["first_false_to_true_activation"]["last_bad_ms_after"], 0
        )

    def test_trigger_and_hold_use_distinct_epochs(self):
        rows = self.pair(11000, 11000, 1, 0.2, 0, 0, 11000, 10900, 10900)
        rows += self.pair(11100, 100, 1, 0.2, 0, 1, 11100, 10900, 10900)
        result = self.analyze(rows)
        self.assertEqual(result["trigger_conditions"], 2)
        self.assertEqual(result["first_trigger_condition"]["publication_s"], 11)
        self.assertFalse(result["first_trigger_condition"]["bad_imu_after"])
        self.assertEqual(
            result["first_false_to_true_activation"]["publication_s"], 11.1
        )

    def test_initial_three_sigma_threshold_does_not_trigger(self):
        result = self.analyze(self.pair(21000, 21000, 9, 0.2, 0, 0, 0, 20900, 21000))
        self.assertEqual(result["trigger_conditions"], 0)
        self.assertEqual(result["bad_imu_after_checks"], 0)

    def test_rejects_missing_pairs_and_false_override(self):
        rows = self.pair(1000, 1000, 1, 0, 0, 1, 0, 0, 1000)
        with self.assertRaisesRegex(ValueError, "complete before/after"):
            self.analyze(rows[:1])
        rows[1]["estimated_velocity_down_m_s"] = 0.01
        with self.assertRaisesRegex(ValueError, "differs from reconstructed"):
            self.analyze(rows)


if __name__ == "__main__":
    unittest.main()
