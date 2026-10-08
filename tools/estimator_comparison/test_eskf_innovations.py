"""Negative controls for joint NIS, selection accounting and generated hooks."""

import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from eskf_innovations import FIELDS, check, reconstruct, summarize
from instrument_eskf_innovations import function_span


def observation():
    row = dict.fromkeys(FIELDS, "0")
    row.update(
        sensor="optical_flow",
        state_epoch_s="125",
        sample_epoch_s="124.5",
        age_s=".5",
        dimension="2",
        computed="1",
        accepted="1",
        outcome="1",
        nis=str(float(np.float32(8 / 7))),
        gate="1",
        r0="1",
        r1="1",
        noise0=".1",
        noise1=".1",
        s0_0="1",
        s1_1="1",
        s0_1=".75",
        s1_0=".75",
    )
    return row


class EskfInnovationTests(unittest.TestCase):
    def evaluate(self, rows, valid=True):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "observations.csv"
            output = path.with_suffix(".json")
            with path.open("w") as stream:
                writer = csv.DictWriter(stream, fieldnames=sorted(FIELDS))
                writer.writeheader()
                writer.writerows(rows)
            args = SimpleNamespace(
                innovations=path, output=output, windows=[("flight", 120, 166.7)]
            )
            if valid:
                return check(args)
            with self.assertRaisesRegex(ValueError, "failed reconstruction"):
                check(args)
            return json.loads(output.read_text())

    def test_repeated_evaluations_are_checked_and_disclosed(self):
        result = self.evaluate([observation(), observation()])
        self.assertEqual(result["raw_rows"], 2)
        self.assertEqual(result["unique_candidates"], 1)
        self.assertEqual(result["repeated_identical_evaluations"], {"optical_flow": 1})
        self.assertEqual(result["records"][0]["all_candidates"]["rows"], 1)

    def test_conflicting_repeated_evaluations_invalidate_evidence(self):
        other = observation()
        other["noise0"] = ".2"
        result = self.evaluate([observation(), other], valid=False)
        self.assertFalse(result["valid"])
        self.assertEqual(result["invalid_rows"], 1)
        self.assertIn("Conflicting", result["failures"][0]["reason"])

    def test_joint_nis_uses_cross_covariance(self):
        value = reconstruct(observation())
        self.assertAlmostEqual(value["nis"], 8 / 7)
        self.assertAlmostEqual(value["normalized_nis"], 4 / 7)
        self.assertNotAlmostEqual(value["nis"], 2)

    def test_disagreement_with_actual_update_rejected(self):
        row = observation()
        row["nis"] = "2"
        with self.assertRaisesRegex(ValueError, "generated update"):
            reconstruct(row)

    def test_singular_and_asymmetric_covariances_rejected(self):
        for changes in ({"s0_1": "1", "s1_0": "1"}, {"s0_1": ".5"}):
            with self.subTest(changes=changes):
                row = observation()
                row.update(changes)
                with self.assertRaises(ValueError):
                    reconstruct(row)

    def test_gate_threshold_includes_measurement_dimension(self):
        row = observation()
        row.update(gate=".6")
        self.assertTrue(reconstruct(row)["accepted"])
        row.update(gate=".5")
        with self.assertRaisesRegex(ValueError, "gate decision"):
            reconstruct(row)
        row.update(accepted="0", outcome="3")
        self.assertFalse(reconstruct(row)["accepted"])

    def test_timestamp_rejection_never_counts_as_zero_nis(self):
        row = observation()
        row.update(computed="0", accepted="0", outcome="6", nis="0")
        result = summarize([reconstruct(row), reconstruct(observation())])
        self.assertEqual(result["attempted_rows"], 2)
        self.assertEqual(result["prelinear_rejected_rows"], 1)
        self.assertEqual(result["all_candidates"]["rows"], 1)
        self.assertAlmostEqual(
            result["all_candidates"]["mean_joint_nis_per_dimension"], 4 / 7
        )

    def test_rejected_candidates_remain_in_all_candidate_statistics(self):
        row = observation()
        row.update(accepted="0", outcome="3", gate=".5")
        result = summarize([reconstruct(row)])
        self.assertEqual(result["all_candidates"]["rows"], 1)
        self.assertEqual(result["accepted"]["rows"], 0)
        self.assertEqual(result["rejected"]["rows"], 1)

    def test_bad_decision_and_clock_rejected(self):
        for changes in ({"outcome": "3"}, {"state_epoch_s": "124"}, {"computed": "0"}):
            with self.subTest(changes=changes):
                row = observation()
                row.update(changes)
                with self.assertRaises(ValueError):
                    reconstruct(row)

    def test_definition_hook_skips_prototype_and_literal_braces(self):
        text = """static void leaf(
    int value);
static void leaf(
    int value) {
    const char *brace = "}";
    /* } */
    if (value) { use(brace); }
}
static void next(
    int value) {
}
"""
        start, end = function_span(text, "leaf")
        self.assertIn("use(brace)", text[start:end])
        self.assertNotIn("next", text[start:end])
        self.assertEqual(text[end : end + 2], "}\n")
        with self.assertRaisesRegex(ValueError, "one actual"):
            function_span(text + text, "leaf")


if __name__ == "__main__":
    unittest.main()
