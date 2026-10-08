"""Check the generated Modelica basis against independent Lie-injection differences."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


def rotation(quaternion):
    scalar, vector = quaternion[0], quaternion[1:]
    x, y, z = vector
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return (scalar * scalar - vector @ vector) * np.eye(3) + 2 * (
        np.outer(vector, vector) + scalar * skew
    )


def physical_error(nominal, injected):
    reference = nominal / np.linalg.norm(nominal)
    corrected = injected[6:10] / np.linalg.norm(injected[6:10])
    scalar = reference @ corrected
    vector = (
        reference[0] * corrected[1:]
        - corrected[0] * reference[1:]
        - np.cross(reference[1:], corrected[1:])
    )
    if scalar < 0:
        scalar, vector = -scalar, -vector
    magnitude = np.linalg.norm(vector)
    angle = 2 * np.arctan2(magnitude, scalar)
    attitude = vector * (angle / magnitude if magnitude > 1e-15 else 2)
    return np.concatenate((injected[:6], attitude, injected[13:16], injected[10:13]))


def check(executable):
    rng = np.random.default_rng(20261008)
    quaternions = np.vstack((np.eye(4), rng.normal(size=(32, 4))))
    quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
    quaternions = quaternions.astype(np.float32).astype(float)
    steps = (0.002, 0.001)
    requests = []
    for quaternion in quaternions:
        for step in steps:
            for component in range(15):
                for sign in (1, -1):
                    correction = np.eye(15)[component] * sign * step
                    requests.append(np.r_[quaternion, correction])
    run = subprocess.run(
        [str(executable)],
        input="\n".join(" ".join(format(x, ".9g") for x in row) for row in requests),
        text=True,
        capture_output=True,
        check=True,
    )
    rows = np.array(
        [[float(x) for x in line.split()] for line in run.stdout.splitlines()]
    )
    if rows.shape != (len(requests), 241) or not np.isfinite(rows).all():
        raise AssertionError(f"Invalid generated output: {rows.shape}")
    outputs = rows.reshape(len(quaternions), len(steps), 15, 2, 241)
    basis_error = derivative_error = covariance_error = nees_error = 0.0
    permutation_negative_control = cross_negative_control = 0.0
    for quaternion, results in zip(quaternions, outputs, strict=True):
        expected = np.zeros((15, 15))
        expected[:3, :3] = expected[3:6, 3:6] = rotation(quaternion)
        expected[6:9, 6:9] = np.eye(3)
        expected[9:12, 12:15] = expected[12:15, 9:12] = np.eye(3)
        transform = results[0, 0, 0, :225].reshape(15, 15)
        basis_error = max(basis_error, float(np.max(np.abs(transform - expected))))
        for step, samples in zip(steps, results, strict=True):
            derivatives = np.column_stack(
                [
                    (
                        physical_error(quaternion, sample[0, 225:])
                        - physical_error(quaternion, sample[1, 225:])
                    )
                    / (2 * step)
                    for sample in samples
                ]
            )
            derivative_error = max(
                derivative_error, float(np.max(np.abs(derivatives - transform)))
            )
            missing_permutation = transform.copy()
            missing_permutation[9:15, 9:15] = np.eye(6)
            permutation_negative_control = max(
                permutation_negative_control,
                float(np.max(np.abs(derivatives - missing_permutation))),
            )
        reference_quaternion = rng.normal(size=4)
        reference_rotation = rotation(
            reference_quaternion / np.linalg.norm(reference_quaternion)
        )
        joint = np.zeros((21, 21))
        joint[:15, :15] = transform
        joint[15:18, 15:18] = reference_rotation
        joint[18:21, 18:21] = np.eye(3)
        root = rng.normal(size=(21, 21))
        covariance = root @ root.T + np.eye(21)
        converted = joint @ covariance @ joint.T
        recovered = np.linalg.solve(joint, np.linalg.solve(joint, converted).T).T
        covariance_error = max(
            covariance_error, float(np.max(np.abs(recovered - covariance)))
        )
        error = rng.normal(size=21)
        converted_error = joint @ error
        nees = error @ np.linalg.solve(covariance, error)
        converted_nees = converted_error @ np.linalg.solve(converted, converted_error)
        nees_error = max(nees_error, float(abs(nees - converted_nees)))
        dropped = converted.copy()
        dropped[:15, 15:] = dropped[15:, :15] = 0
        difference_variance = np.r_[np.ones(3), np.zeros(12), -np.ones(3), np.zeros(3)]
        cross_negative_control = max(
            cross_negative_control,
            float(
                abs(difference_variance @ (dropped - converted) @ difference_variance)
            ),
        )
    passed = (
        basis_error < 5e-7
        and derivative_error < 2e-4
        and covariance_error < 1e-10
        and nees_error < 1e-10
        and permutation_negative_control > 0.9
        and cross_negative_control > 1
    )
    return {
        "complete": True,
        "passed": passed,
        "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "orientation_cases": len(quaternions),
        "actual_generated_injections": len(requests),
        "finite_difference_steps": steps,
        "maximum_basis_error": basis_error,
        "maximum_injection_derivative_error": derivative_error,
        "maximum_joint_covariance_roundtrip_error": covariance_error,
        "maximum_joint_nees_invariance_error": nees_error,
        "missing_bias_permutation_negative_control_error": permutation_negative_control,
        "dropped_cross_covariance_negative_control_variance_error": cross_negative_control,
        "scope": "First-order coordinate bridge; finite updates and visual fusion are not qualified.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = check(args.executable.resolve())
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
