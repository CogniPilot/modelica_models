"""Negative controls for the bounded ESKF performance measurement."""
import unittest
from tools.ci import TaskError, validate_eskf_throughput


class ESKFThroughputTests(unittest.TestCase):
    def report(self, seconds):
        return {"hot_average_seconds": seconds, "last_final_time": 0.1,
                "last_points": 21}

    def test_completed_measurement(self):
        self.assertEqual(validate_eskf_throughput(self.report(0.3)), 0.3)

    def test_interpreter_cliff_exceeds_budget(self):
        for seconds in (9.992, 12.019):
            with self.subTest(seconds=seconds), self.assertRaises(TaskError):
                validate_eskf_throughput(self.report(seconds))

    def test_missing_or_invalid_measurement_refused(self):
        for value in (None, False, 0, -1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(TaskError):
                validate_eskf_throughput(self.report(value))

    def test_non_object_report_refused(self):
        for report in (None, [], False):
            with self.subTest(report=report), self.assertRaises(TaskError):
                validate_eskf_throughput(report)

    def test_incomplete_run_refused(self):
        for final_time, points in ((0.05, 21), (None, 21), (0.1, 1), (0.1, None), (0.1, True)):
            report = self.report(0.3)
            report.update(last_final_time=final_time, last_points=points)
            with self.subTest(report=report), self.assertRaises(TaskError):
                validate_eskf_throughput(report)
