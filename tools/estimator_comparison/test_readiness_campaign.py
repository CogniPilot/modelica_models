"""Guard matched evidence joins and retain invalid or missing comparison pairs."""

import copy
import unittest

from report_readiness_campaign import METRICS, comparisons, innovation_summary, join


class ReadinessCampaignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = dict(
            filters=[
                "horizon",
                "retrodiction",
                "horizon_joint",
                "retrodiction_joint",
                "px4",
                "ekf3",
            ],
            scenarios=["gps", "denied", "transition"],
            gps_fix_after_s=21,
        )
        accuracy = dict(
            **{metric: 0.1 for metric in METRICS},
            rows=100,
            row_coverage=1,
            finite_fraction=1,
            position_valid_fraction=1,
            attitude_valid_fraction=1,
        )
        statistics = {
            window: dict(valid=True, mean_nees_15d=15)
            for window in ("flight", "outage", "after_return")
        }
        scores = [
            dict(
                name=name,
                scenario=scenario,
                output_sha256=f"state-{name}-{scenario}",
                arrival_trace_sha256=f"packets-{scenario}",
                consistency=copy.deepcopy(statistics),
                **{
                    window: dict(accuracy)
                    for window in ("flight", "outage_window", "after_return")
                },
            )
            for name in cls.plan["filters"]
            for scenario in cls.plan["scenarios"]
        ]
        cls.pilot_sha256 = "pilot-digest"
        cls.pilot = dict(
            complete=True,
            origin=dict(seed=911, speed=0.6, gps_fix_after_s=21),
            input_sha256="capture-digest",
            scores=scores,
        )
        cls.covariance = dict(
            complete=True,
            pilot_sha256=cls.pilot_sha256,
            scores=[
                dict(
                    score,
                    output_identical=True,
                    consistency=dict(valid=True, windows=copy.deepcopy(statistics)),
                )
                for score in scores
                if score["name"] in ("px4", "ekf3")
            ],
        )
        cls.capture = dict(
            name="diagnostic",
            seed=911,
            speed=0.6,
            height_m=2,
            audit=dict(input_sha256=cls.pilot["input_sha256"]),
        )

    def joined(self, pilot=None, covariance=None):
        return join(
            pilot or self.pilot,
            covariance or self.covariance,
            self.pilot_sha256,
            self.capture,
            self.plan,
        )

    def test_state_and_arrival_parity_are_required(self):
        covariance = copy.deepcopy(self.covariance)
        covariance["scores"][0]["output_sha256"] = "changed"
        with self.assertRaisesRegex(ValueError, "changed inputs or state"):
            self.joined(covariance=covariance)
        pilot = copy.deepcopy(self.pilot)
        pilot["scores"][0]["arrival_trace_sha256"] = "different packets"
        with self.assertRaisesRegex(ValueError, "different sensor arrival traces"):
            self.joined(pilot=pilot)
        pilot = copy.deepcopy(self.pilot)
        pilot["scores"].append(pilot["scores"][0])
        with self.assertRaisesRegex(ValueError, "Duplicated"):
            self.joined(pilot=pilot)

    def test_invalid_native_covariance_does_not_become_a_win(self):
        covariance = copy.deepcopy(self.covariance)
        covariance["scores"][0]["consistency"] = dict(valid=False, reason="indefinite")
        rows = self.joined(covariance=covariance)
        invalid = [
            row
            for row in rows
            if row["estimator"] == "px4" and row["scenario"] == "gps"
        ]
        self.assertEqual(len(invalid), 3)
        self.assertTrue(all(row["mean_nees_15d"] is None for row in invalid))
        self.assertTrue(all(row[METRICS[0]] > 0 for row in invalid))
        paired = comparisons(
            rows, self.plan["filters"], self.plan["scenarios"], ["diagnostic"]
        )
        selected = [
            row for row in paired if row["right"] == "px4" and row["scenario"] == "gps"
        ]
        self.assertTrue(
            all(
                row["valid_pairs"] == 0
                and row["left_lower"] == 0
                and row["invalid_pairs"] == ["diagnostic"]
                for row in selected
            )
        )

    def test_failed_capture_remains_in_pair_denominator(self):
        rows = self.joined()
        pairs = comparisons(
            rows, self.plan["filters"], self.plan["scenarios"], ["diagnostic", "failed"]
        )
        self.assertTrue(
            all(
                row["declared_pairs"] == 2
                and row["valid_pairs"] == 1
                and row["invalid_pairs"] == ["failed"]
                for row in pairs
            )
        )

    def test_nis_stages_and_invalid_cases_remain_separate(self):
        fusion = dict(
            estimator="px4",
            scenario="gps",
            sensor="gps_velocity",
            axis=0,
            stage="fusion",
            navigation=True,
            window="flight",
            valid=True,
            mean_scalar_nis=0.5,
            rows=10,
            invalid_rows=0,
            fused_rows=10,
            rejected_fusion_rows=0,
        )
        invalid = dict(fusion, valid=False, mean_scalar_nis=100, invalid_rows=1)
        gate = dict(fusion, stage="gate", mean_scalar_nis=2, fused_rows=0)
        summary = {
            row["stage"]: row for row in innovation_summary([fusion, invalid, gate])
        }
        self.assertEqual(summary["fusion"]["captures"], 2)
        self.assertEqual(summary["fusion"]["invalid_captures"], 1)
        self.assertEqual(summary["fusion"]["invalid_rows"], 1)
        self.assertEqual(summary["fusion"]["median_capture_mean_scalar_nis"], 0.5)
        self.assertEqual(summary["gate"]["median_capture_mean_scalar_nis"], 2)


if __name__ == "__main__":
    unittest.main()
