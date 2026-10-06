"""Negative controls for the GPS-to-Mocap survey discrimination gate."""

import unittest

import numpy as np

from Vehicles.Rdd2.Test import run_waypoint_qualification as q


def trace(offset=(0.0, 0.0, 0.0)):
    times = [14.0, 14.5, 14.99, 15.0, 15.25, 15.5, 20.0, 31.99, 32.0]
    values = {"time_s": times, "estimatorUpdatePeriod_s": [0.01] * len(times)}
    for i in range(1, 4):
        truth = np.arange(len(times), dtype=float)
        # A shared GPS correction larger than the survey offset must not
        # conceal the offset. Different controller paths also cancel against
        # each mission's own truth.
        motion = np.array([offset[i - 1] if t >= 15 else 0 for t in times])
        error = np.array([offset[i - 1] if 15 <= t < 32 else 0 for t in times])
        error[1] += 0.25
        values[f"position_m[{i}]"] = (truth + motion).tolist()
        values[f"estimator.estimate.positionWorldEnu_m[{i}]"] = (
            truth + motion + error
        ).tolist()
    return values


class HandoffComparisonTests(unittest.TestCase):
    def test_offset_survives_larger_unrelated_gps_correction(self):
        self.assertTrue(q.handoff_comparison(trace(q.HANDOFF_SURVEY_OFFSET_M), trace())["passed"])

    def test_missing_reversed_doubled_or_rotated_offset_fails(self):
        for offset in (
            (0.0, 0.0, 0.0),
            tuple(-x for x in q.HANDOFF_SURVEY_OFFSET_M),
            tuple(2 * x for x in q.HANDOFF_SURVEY_OFFSET_M),
            (-0.03, 0.05, 0.0),
        ):
            with self.subTest(offset=offset):
                result = q.handoff_comparison(trace(offset), trace())
                self.assertFalse(result["checks"]["survey_offset_is_visible_at_entry"])
                self.assertFalse(result["passed"])

    def test_offset_must_persist_after_entry(self):
        surveyed = trace(q.HANDOFF_SURVEY_OFFSET_M)
        ideal = trace()
        for i in range(1, 4):
            name = f"estimator.estimate.positionWorldEnu_m[{i}]"
            surveyed[name][5:] = ideal[name][5:]
        result = q.handoff_comparison(surveyed, ideal)
        self.assertFalse(result["checks"]["survey_offset_persists_in_coverage"])

    def test_navigation_must_agree_before_coverage(self):
        surveyed = trace(q.HANDOFF_SURVEY_OFFSET_M)
        surveyed["estimator.estimate.positionWorldEnu_m[1]"][0] += 0.05
        self.assertFalse(q.handoff_comparison(surveyed, trace())["passed"])

    def test_mismatched_ticks_fail(self):
        ideal = trace()
        ideal["time_s"][0] += 0.01
        self.assertFalse(q.handoff_comparison(trace(q.HANDOFF_SURVEY_OFFSET_M), ideal)["passed"])

    def test_missing_entry_or_settling_samples_fail(self):
        for start, end in ((15.0, 15.5), (15.5, 32.0)):
            surveyed, ideal = trace(q.HANDOFF_SURVEY_OFFSET_M), trace()
            keep = [i for i, t in enumerate(ideal["time_s"]) if not start <= t < end]
            for values in (surveyed, ideal):
                for name, samples in values.items():
                    values[name] = [samples[i] for i in keep]
            self.assertFalse(q.handoff_comparison(surveyed, ideal)["passed"])

    def test_nonfinite_errors_fail(self):
        surveyed = trace(q.HANDOFF_SURVEY_OFFSET_M)
        surveyed["estimator.estimate.positionWorldEnu_m[1]"][3] = float("nan")
        self.assertFalse(q.handoff_comparison(surveyed, trace())["passed"])


if __name__ == "__main__":
    unittest.main()
