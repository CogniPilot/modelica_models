"""Check generated binary32 SE23 covariance reset against a Lie-adjoint series."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


MODEL = """within;
block ResetProbe
  input Real tangent[9];
  output Real resetJacobian[9, 9];
algorithm
  when sample(0, 0.01) then
    resetJacobian := LieGroups.SE23.Quat.right_jacobian(tangent);
  end when;
end ResetProbe;
"""

HARNESS = """#include "ResetProbe.h"
#include <stdio.h>
#include <string.h>

int main(void) {
  float tangent[9], output[82];
  ResetProbeState state;
  while (fread(tangent, sizeof(float), 9, stdin) == 9) {
    ResetProbe_startup(&state);
    memcpy(state.tangent, tangent, sizeof(tangent));
    ResetProbe_dostep(&state);
    memcpy(output, state.resetJacobian, 81 * sizeof(float));
    output[81] = state.rumoca_galec_error_signal_status;
    if (fwrite(output, sizeof(float), 82, stdout) != 82) return 2;
  }
  return 0;
}
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def reference(tangent):
    adjoint = np.zeros((9, 9))
    for start in (0, 3, 6):
        adjoint[start : start + 3, start : start + 3] = skew(tangent[6:9])
    adjoint[:3, 6:9] = skew(tangent[:3])
    adjoint[3:6, 6:9] = skew(tangent[3:6])
    term = np.eye(9)
    jacobian = term.copy()
    for order in range(1, 25):
        term = term @ (-adjoint) / (order + 1)
        jacobian += term
    return jacobian


def check(args):
    args.work.mkdir(parents=True)
    model = args.work / "ResetProbe.mo"
    model.write_text(MODEL)
    harness = args.work / "reset_probe.c"
    harness.write_text(HARNESS)
    export = args.work / "export"
    production = export / "ResetProbe/ProductionCode"
    binary = args.work / "reset_probe"
    commands = [
        [
            str(args.rumoca),
            "compile",
            str(model),
            "--source-root",
            str(args.source_root),
            "--model",
            "ResetProbe",
            "--target",
            "galec-production",
            "--cache-dir",
            str(args.cache_dir),
            "--output",
            str(export),
        ],
        [
            str(args.cc),
            "-O2",
            "-I" + str(production),
            str(harness),
            str(production / "ResetProbe.c"),
            str(production / "rumoca_galec_kernels.c"),
            "-lm",
            "-o",
            str(binary),
        ],
    ]
    for index, command in enumerate(commands):
        completed = subprocess.run(command, capture_output=True, text=True)
        (args.work / f"build-{index}.log").write_text(
            completed.stdout + completed.stderr
        )
        completed.check_returncode()
    rng = np.random.default_rng(2026100813)
    angles = np.r_[
        0, 1e-8, np.geomspace(1e-7, 0.1, 32), 0.099999, 0.100001, 0.12, 0.14999, 0.15
    ]
    cases = []
    for angle in angles:
        for _ in range(16):
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            cases.append(np.r_[rng.normal(size=6) * 5, angle * axis])
    inputs = np.asarray(cases, dtype=np.float32)
    completed = subprocess.run(
        [str(binary)], input=inputs.tobytes(), capture_output=True, check=True
    )
    (args.work / "runtime.log").write_bytes(completed.stderr)
    (args.work / "output.bin").write_bytes(completed.stdout)
    actual = np.frombuffer(completed.stdout, np.float32).reshape(-1, 82)
    if len(actual) != len(inputs):
        raise ValueError("Incomplete generated reset output")
    maximum_error = 0.0
    failures = []
    for index, (tangent, output) in enumerate(zip(inputs, actual)):
        expected = reference(tangent.astype(float))
        error = float(np.max(np.abs(output[:81].reshape(9, 9) - expected)))
        maximum_error = max(maximum_error, error)
        if output[81] != 0 or not np.allclose(
            output[:81].reshape(9, 9), expected, atol=3e-6, rtol=4e-6
        ):
            failures.append(
                dict(
                    case=index,
                    angle_rad=float(np.linalg.norm(tangent[6:9])),
                    maximum_error=error,
                    runtime_status=int(output[81]),
                )
            )
    source = args.source_root / "LieGroups/SE3/Quat/left_Q.mo"
    result = dict(
        complete=True,
        passed=not failures,
        cases=len(inputs),
        maximum_error=maximum_error,
        failures=failures,
        commands=commands,
        source_sha256=digest(source),
        checker_sha256=digest(Path(__file__)),
        binary_sha256=digest(binary),
        input_sha256=hashlib.sha256(inputs.tobytes()).hexdigest(),
        generated_sha256={
            p.name: digest(p) for p in production.iterdir() if p.is_file()
        },
        scope="Actual Rumoca binary32 right Jacobian, angle 0 through ESKF correction limit 0.15 rad. Independent 24-term Lie-adjoint series. No flight or global precision claim.",
    )
    (args.work / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("passed", "cases", "maximum_error")}))
    return not failures


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source-root", "work", "rumoca", "cc", "cache-dir"):
        parser.add_argument(
            "--" + name, type=lambda value: Path(value).resolve(), required=True
        )
    args = parser.parse_args()
    raise SystemExit(0 if check(args) else 1)
