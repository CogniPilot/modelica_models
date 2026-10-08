"""Check deployment raw prediction against noiseless inertial kinematics."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np


def check(args):
    function = ctypes.CDLL(str(args.library.resolve())).raw_prediction
    array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
    function.argtypes = [array, array, ctypes.c_float, ctypes.c_int, array]
    function.restype = ctypes.c_int
    rng = np.random.default_rng(args.seed)
    cases = []
    for trial in range(args.trials):
        factor = rng.normal(size=(15, 30)) * 10.0 ** rng.uniform(-2, 1, (15, 1))
        covariance = (factor @ factor.T).astype(np.float32)
        prior_root = np.linalg.cholesky(covariance.astype(float)).astype(np.float32)
        for dt in (0.0, 0.001, 0.01, 0.1, 1.0):
            dt = float(np.float32(dt))
            sensitivity = np.eye(15)
            sensitivity[:3, 3:6] = dt * np.eye(3)
            sensitivity[:3, 12:15] = -0.5 * dt**2 * np.eye(3)
            sensitivity[3:6, 12:15] = -dt * np.eye(3)
            sensitivity[6:9, 9:12] = -dt * np.eye(3)
            expected = sensitivity @ covariance.astype(float) @ sensitivity.T
            scale = np.sqrt(np.diag(expected))
            expected_nominal = np.r_[
                dt * np.array([1, -2, 0.5]), [1, -2, 0.5], 1, np.zeros(9)
            ]
            for square_root in (False, True):
                output = np.empty(241, dtype=np.float32)
                status = function(
                    covariance.ravel(), prior_root.ravel(), dt, square_root, output
                )
                actual = output[16:].reshape(15, 15).astype(float)
                error = float(
                    np.max(np.abs(actual - expected) / np.outer(scale, scale))
                )
                mean_error = float(np.max(np.abs(output[:16] - expected_nominal)))
                if (
                    status
                    or not np.isfinite(output).all()
                    or error > 3e-5
                    or mean_error > 1e-7
                ):
                    raise ValueError(
                        f"Raw prediction lost its prior or kinematics: {trial}/{dt}/{square_root}/{error}/{mean_error}"
                    )
                np.linalg.cholesky((actual + actual.T) / 2)
                cases.append(
                    dict(
                        trial=trial,
                        dt=dt,
                        square_root=square_root,
                        normalized_covariance_error=error,
                        nominal_error=mean_error,
                    )
                )
    report = dict(
        seed=args.seed,
        cases=cases,
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        note="Zero measured rotation, specific force and gravity, with zero process noise and zero nominal bias. Reference sensitivities follow constant-velocity motion and constant gyro/accelerometer bias perturbations. Includes zero-time prior preservation, full correlated priors, and both covariance representations.",
    )
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(
        json.dumps(
            dict(
                cases=len(cases),
                maximum_normalized_covariance_error=max(
                    r["normalized_covariance_error"] for r in cases
                ),
            )
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261007)
    parser.add_argument("--trials", type=int, default=32)
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("At least one kinematic trial is required")
    check(args)
