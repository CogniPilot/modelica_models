"""Negative controls for matched ablations and failed-condition accounting."""

import copy
import itertools
import unittest

from report_eskf_ablation import candidate_rows, paired, validate_candidate
from report_readiness_campaign import METRICS, WINDOWS
from replay_eskf_innovations import VARIANTS


def flat(estimator, capture="a", **changes):
    return dict(
        capture=capture,
        estimator=estimator,
        scenario="denied",
        window="flight",
        state_valid=True,
        covariance_valid=True,
        readiness_qualified=None,
        **dict.fromkeys(METRICS, 1.0),
        **changes,
    )


def candidates():
    reference = dict(mission=dict(windows=[("flight", 120, 166.7)]), scores=[])
    candidate = dict(
        complete=True,
        reference_sha256="reference",
        input_sha256={"imu.csv": "input"},
        scores=[],
    )
    for name, scenario in itertools.product(VARIANTS, ("gps", "denied", "transition")):
        reference["scores"].append(
            dict(
                name=name,
                scenario=scenario,
                transport={"transport_delivered": [1, 2, 3, 4]},
            )
        )
        candidate["scores"].append(
            dict(
                name=name,
                scenario=scenario,
                timing={"transport_delivered": [1, 2, 3, 4]},
                consistency={"flight": dict(valid=True)},
                full16_covariance={},
            )
        )
    capture = dict(audit=dict(input_sha256={"imu.csv": "input"}))
    return candidate, reference, capture


class AblationTests(unittest.TestCase):
    def test_invalid_covariance_cannot_count_as_a_win(self):
        a, b = flat("new"), flat("native")
        a["covariance_valid"] = False
        a[METRICS[0]] = 0
        result = paired([a, b], [("new", "native")], ["a"])
        row = next(
            r
            for r in result
            if r["scenario"] == "denied"
            and r["window"] == "flight"
            and r["metric"] == METRICS[0]
        )
        self.assertEqual(row["declared_pairs"], 1)
        self.assertEqual(row["valid_pairs"], 0)
        self.assertEqual(row["left_lower"], 0)
        self.assertEqual(row["invalid_pairs"], ["a"])

    def test_missing_capture_stays_in_denominator(self):
        rows = [flat("new"), flat("native")]
        result = paired(rows, [("new", "native")], ["a", "missing"])
        row = next(
            r for r in result if r["scenario"] == "denied" and r["window"] == "flight"
        )
        self.assertEqual(row["declared_pairs"], 2)
        self.assertEqual(row["valid_pairs"], 1)
        self.assertEqual(row["invalid_pairs"], ["missing"])

    def test_candidate_cannot_inherit_baseline_accuracy_or_coverage(self):
        accuracy = dict.fromkeys(METRICS, 2.0)
        accuracy.update(
            rows=5,
            row_coverage=0.5,
            finite_fraction=1,
            position_valid_fraction=1,
            attitude_valid_fraction=1,
            nonzero_step_status_rows=0,
            prediction_accepted_fraction=1,
        )
        score = dict(
            name="horizon",
            scenario="denied",
            output_sha256="new",
            consistency={
                "flight": dict(valid=True, mean_nees_15d=3),
                "outage": dict(valid=True, mean_nees_15d=4),
                "after_return": dict(valid=True, mean_nees_15d=5),
            },
            **{window: copy.deepcopy(accuracy) for window in WINDOWS},
        )
        originals = [
            dict(
                flat("horizon"),
                window=window,
                rows=10,
                row_coverage=1,
                output_sha256="old",
            )
            for window in WINDOWS
        ]
        rows = candidate_rows(dict(scores=[score]), originals, "stationary_")
        self.assertTrue(all(row[METRICS[0]] == 2 for row in rows))
        self.assertTrue(
            all(row["row_coverage"] == 0.5 and not row["state_valid"] for row in rows)
        )
        self.assertTrue(all(row["output_sha256"] == "new" for row in rows))

    def test_missing_scenario_rejected(self):
        candidate, reference, capture = candidates()
        candidate["scores"].pop()
        with self.assertRaisesRegex(ValueError, "every candidate"):
            validate_candidate(candidate, reference, "reference", capture)

    def test_changed_physical_input_rejected(self):
        candidate, reference, capture = candidates()
        candidate["input_sha256"]["imu.csv"] = "different"
        with self.assertRaisesRegex(ValueError, "frozen capture"):
            validate_candidate(candidate, reference, "reference", capture)

    def test_changed_delivery_rejected(self):
        candidate, reference, capture = candidates()
        candidate["scores"][0]["timing"]["transport_delivered"][0] = 2
        with self.assertRaisesRegex(ValueError, "delivery"):
            validate_candidate(candidate, reference, "reference", capture)

    def test_missing_full_pressure_covariance_rejected(self):
        candidate, reference, capture = candidates()
        next(s for s in candidate["scores"] if s["name"].endswith("_joint")).pop(
            "full16_covariance"
        )
        with self.assertRaisesRegex(ValueError, "pressure covariance"):
            validate_candidate(candidate, reference, "reference", capture)


if __name__ == "__main__":
    unittest.main()
