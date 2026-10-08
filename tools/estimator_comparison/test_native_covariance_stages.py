"""Check constrained-gain diagnostics and observers spanning translation units."""

import csv
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from diagnose_ekf3_covariance_stages import diagnose, scalar_covariances


class NativeCovarianceStageTests(unittest.TestCase):
    def test_constrained_gain_requires_joseph_form(self):
        prior = np.array([[1, 0.9], [0.9, 1]])
        gain = np.array([1 / 1.01, 0])
        left, joseph = scalar_covariances(prior, gain, 0, 0.01)
        self.assertLess(np.linalg.eigvalsh((left + left.T) / 2).min(), 0)
        np.linalg.cholesky(joseph)
        optimal = prior[:, 0] / 1.01
        left, joseph = scalar_covariances(prior, optimal, 0, 0.01)
        np.testing.assert_allclose(left, joseph, atol=1e-15)

    def fixture(self):
        prior = np.eye(24)
        prior[7, 13] = prior[13, 7] = 0.9
        gain = np.zeros(24)
        gain[7] = 1 / 1.01
        left, _ = scalar_covariances(prior, gain, 7, 0.01)
        symmetric = (left + left.T) / 2
        state = np.zeros(24)
        state[0] = 1
        stages = (
            ("scalar.before", prior),
            ("scalar.ForceSymmetry.before", left),
            ("scalar.ForceSymmetry.after", symmetric),
            ("scalar.ConstrainVariances.before", symmetric),
            ("scalar.ConstrainVariances.after", symmetric),
        )
        return [
            dict(
                publication_us=1000000,
                fusion_us=800000,
                bias_period_s=0.01,
                stage=stage,
                axis=3,
                state_index_limit=15,
                gyro_inhibited=0,
                accel_inhibited=0,
                accel_x_inhibited=1,
                accel_y_inhibited=0,
                accel_z_inhibited=0,
                bad_imu_data=1,
                aiding_mode=0,
                noise_variance=0.01,
                **{f"s{i}": state[i] for i in range(24)},
                **{f"k{i}": gain[i] for i in range(24)},
                **{f"p{i}_{j}": covariance[i, j] for i in range(24) for j in range(24)},
            )
            for stage, covariance in stages
        ]

    def analyze(self, rows):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "trace.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)
            return diagnose(path)

    def test_locates_scalar_failure(self):
        result = self.analyze(self.fixture())
        self.assertEqual(
            result["first_symmetric_failure"]["stage"], "scalar.ForceSymmetry.after"
        )
        self.assertEqual(result["scalar_updates_checked"], 1)
        (failure,) = result["scalar_positive_to_invalid_updates"]
        self.assertEqual(failure["modified_gain_indices"], [13])
        self.assertTrue(failure["counterfactual_joseph"]["common_cholesky_valid"])
        self.assertEqual(failure["left_update_max_error"], 0)

    def test_rejects_incomplete_and_incorrect_operations(self):
        with self.assertRaisesRegex(ValueError, "Incomplete or reordered"):
            self.analyze(self.fixture()[:-1])
        rows = self.fixture()
        rows[1]["p7_7"] += 0.001
        with self.assertRaisesRegex(ValueError, "differs from reconstructed"):
            self.analyze(rows)

    def test_writer_shares_stream_between_translation_units(self):
        compiler = shlex.split(os.environ.get("CXX", "c++"))
        if not compiler or not shutil.which(compiler[0]):
            self.skipTest("C++ compiler unavailable")
        header = Path(__file__).with_name("native_covariance_stage_dump.h")
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            shutil.copyfile(header, directory / header.name)
            declaration = """#include "native_covariance_stage_dump.h"
inline void snapshot(unsigned long long time, const char *stage) {
    double state[24] = {}, covariance[24][24] = {};
    bool inhibited[3] = {};
    writeNativeCovarianceStage(time, time, .01, stage, -1, 15, false,
        false, inhibited, false, 3, state, static_cast<const double*>(nullptr), 0, covariance);
}
"""
            (directory / "main.cpp").write_text(
                declaration + 'void second(); int main() { snapshot(99, "excluded");'
                'snapshot(100, "first"); second(); snapshot(102, "excluded"); }\n'
            )
            (directory / "second.cpp").write_text(
                declaration + 'void second() { snapshot(101, "second"); }\n'
            )
            binary = directory / "observer"
            subprocess.run(
                [*compiler, "-std=c++11", "main.cpp", "second.cpp", "-o", str(binary)],
                cwd=directory,
                check=True,
                capture_output=True,
                text=True,
            )
            path = directory / "trace.csv"
            environment = dict(
                os.environ,
                NATIVE_COVARIANCE_STAGE_PATH=str(path),
                NATIVE_COVARIANCE_STAGE_START_US="100",
                NATIVE_COVARIANCE_STAGE_END_US="101",
            )
            subprocess.run([str(binary)], env=environment, check=True)
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["stage"] for row in rows], ["first", "second"])
            self.assertEqual(len(rows[0]), 638)
            previous = path.read_bytes()
            again = subprocess.run([str(binary)], env=environment, capture_output=True)
            self.assertNotEqual(again.returncode, 0)
            self.assertEqual(previous, path.read_bytes())


if __name__ == "__main__":
    unittest.main()
