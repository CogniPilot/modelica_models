"""Negative controls for scalar NIS evidence and selection-stage accounting."""

import csv
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

from native_innovations import check, summarize


def observation():
    return dict(
        publication_us=20_100_000,
        fusion_us=20_000_000,
        sample_us=19_990_000,
        sensor="barometer",
        axis=0,
        stage="fusion",
        innovation=2.0,
        innovation_variance=4.0,
        observation_variance=1.0,
        gate_ratio=0.04,
        fused=1,
        navigation=1,
    )


class InnovationTests(unittest.TestCase):
    def evaluate(self, rows, name="ekf3", **options):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "observations.csv"
            with path.open("w") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0])
                writer.writeheader()
                writer.writerows(rows)
            return check(
                SimpleNamespace(
                    filter=name,
                    innovations=path,
                    output=Path(temporary) / "result.json",
                    **options,
                )
            )

    def test_gate_ratio_is_not_nis(self):
        result = summarize([observation()])
        self.assertEqual(result["mean_scalar_nis"], 1.0)
        self.assertEqual(result["fused_rows"], 1)

    def test_invalid_rows_remain_counted(self):
        rows = [observation()]
        for field, value in (
            ("innovation_variance", -1),
            ("innovation", float("nan")),
            ("innovation", 1e300),
            ("observation_variance", -1),
        ):
            row = observation()
            row[field] = value
            rows.append(row)
        result = summarize(rows)
        self.assertEqual(result["rows"], 5)
        self.assertEqual(result["invalid_rows"], 4)
        self.assertFalse(result["valid"])
        self.assertEqual(result["mean_scalar_nis"], 1.0)

    def test_gates_do_not_count_as_fusions(self):
        gate = observation()
        gate.update(stage="gate", fused=-1)
        records = self.evaluate([gate, observation()])["records"]
        self.assertEqual(len(records), 2)
        for row in records:
            self.assertEqual(row["rows"], 1)
            self.assertEqual(row["fused_rows"], int(row["stage"] == "fusion"))

    def test_future_fusion_epoch_rejected(self):
        row = observation()
        row["fusion_us"] = 21_000_000
        with self.assertRaisesRegex(ValueError, "future fusion"):
            self.evaluate([row])

    def test_stage_and_decision_must_agree(self):
        row = observation()
        row["fused"] = -1
        with self.assertRaisesRegex(ValueError, "stay separate"):
            self.evaluate([row])

    def test_px4_clock_offset_at_outage_boundary(self):
        row = observation()
        row.update(publication_us=26_500_000, fusion_us=26_000_000)
        records = self.evaluate([row], "px4")["records"]
        self.assertEqual({record["window"] for record in records}, {"flight", "outage"})
        row.update(publication_us=26_499_999, fusion_us=25_999_999)
        records = self.evaluate([row], "px4")["records"]
        self.assertEqual({record["window"] for record in records}, {"flight"})

    def test_shifted_windows_use_physical_clock(self):
        row = observation()
        row.update(publication_us=133_500_000, fusion_us=133_000_000)
        result = self.evaluate([row], "px4", windows=(("outage", 132, 147),))
        self.assertEqual(result["native_clock_offset_s"], 1)
        self.assertEqual(result["records"][0]["window"], "outage")


if __name__ == "__main__":
    unittest.main()
