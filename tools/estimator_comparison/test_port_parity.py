"""Exercise exact parity failures that tolerant numerical comparisons conceal."""

from pathlib import Path
import tempfile
import unittest

from port_parity import compare


class PortParityTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.native = Path(self.directory.name) / "native.csv"
        self.modelica = Path(self.directory.name) / "modelica.csv"

    def traces(self, expected, actual):
        header = "case,step,field,precision,bits\n"
        self.native.write_text(header + expected)
        self.modelica.write_text(header + actual)
        return compare(self.native, self.modelica)

    def test_one_ulp_mutation_is_not_parity(self):
        row = "rotation,1,state[0],float32,3f800000\n"
        result = self.traces(row, row.replace("3f800000", "3f800001"))
        self.assertFalse(result["bitwise_equal"])
        self.assertEqual(result["max_ulp_distance"], 1)
        self.assertLess(result["max_scaled_error"], 1e-7)
        self.assertEqual(result["first_difference"]["field"], "state[0]")

    def test_signed_zero_is_numerically_equal_but_not_identical(self):
        row = "rest,0,P[0,0],float64,0000000000000000\n"
        # CSV field names containing commas must be quoted.
        row = row.replace("P[0,0]", '"P[0,0]"')
        result = self.traces(row, row.replace("0000000000000000", "8000000000000000"))
        self.assertFalse(result["bitwise_equal"])
        self.assertEqual(result["signed_zero_differences"], 1)
        self.assertEqual(result["max_absolute_error"], 0)

    def test_matching_negative_values_and_subnormals(self):
        rows = "a,0,state[0],float32,bf800000\na,0,state[1],float32,00000001\n"
        self.assertTrue(self.traces(rows, rows)["bitwise_equal"])
        result = self.traces(rows, rows.replace("bf800000", "bf800001"))
        self.assertEqual(result["max_ulp_distance"], 1)

    def test_missing_reordered_duplicate_and_nonfinite_data_refused(self):
        row = "a,0,state[0],float32,3f800000\n"
        other = "a,0,state[1],float32,3f800000\n"
        for expected, actual in (
            (row, ""),
            (row + other, other + row),
            (row + row, row + row),
            (row, row.replace("3f800000", "7fc00000")),
            (row, row.replace("3f800000", "7f800000")),
            (row, row.replace("3f800000", "3f80000")),
            (
                row,
                row.replace("float32,3f800000", "float64,3ff0000000000000"),
            ),
            (row, row.rstrip() + ",extra\n"),
            (row, "a,0,state[0],float32\n"),
        ):
            with self.subTest(actual=actual), self.assertRaises(ValueError):
                self.traces(expected, actual)


if __name__ == "__main__":
    unittest.main()
