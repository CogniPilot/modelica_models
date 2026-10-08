"""Qualify actual RGB-D Jacobians and matched linearized loose/tight information."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from check_visual_tangent import rotation


def run(executable, inputs):
    execution = subprocess.run(
        [str(executable)],
        input="\n".join(" ".join(format(x, ".9g") for x in row) for row in inputs),
        text=True,
        capture_output=True,
        check=True,
    )
    outputs = np.array(
        [[float(x) for x in line.split()] for line in execution.stdout.splitlines()]
    )
    if outputs.shape != (len(inputs), 59) or not np.isfinite(outputs).all():
        raise AssertionError(f"Invalid generated output {outputs.shape}")
    return outputs


def observation(inputs):
    quaternion, position, landmark = inputs[:4], inputs[4:7], inputs[7:10]
    extrinsic = inputs[10:19].reshape(3, 3)
    lever, intrinsics = inputs[19:22], inputs[22:26]
    camera = extrinsic.T @ (rotation(quaternion).T @ (landmark - position) - lever)
    return np.r_[intrinsics[:2] * camera[:2] / camera[2] + intrinsics[2:], camera[2]]


def fixture(rng):
    quaternion = rng.normal(size=4)
    quaternion /= np.linalg.norm(quaternion)
    extrinsic_quaternion = rng.normal(size=4)
    extrinsic_quaternion /= np.linalg.norm(extrinsic_quaternion)
    extrinsic = rotation(extrinsic_quaternion)
    position, lever = rng.normal(size=(2, 3))
    lever *= 0.15
    camera = np.r_[rng.uniform(-1, 1, 2), rng.uniform(3, 15)]
    landmark = position + rotation(quaternion) @ (lever + extrinsic @ camera)
    return np.r_[
        quaternion,
        position,
        landmark,
        extrinsic.ravel(),
        lever,
        [400, 420, 320, 240],
        np.zeros(15),
    ]


def linearized_comparison(executable, rng):
    nominal = fixture(rng)
    body_rotation = rotation(nominal[:4])
    extrinsic = nominal[10:19].reshape(3, 3)
    inputs = []
    for _ in range(20):
        camera = np.r_[rng.uniform(-2, 2, 2), rng.uniform(3, 12)]
        feature = nominal.copy()
        feature[7:10] = nominal[4:7] + body_rotation @ (
            nominal[19:22] + extrinsic @ camera
        )
        inputs.append(feature)
    features = run(executable, inputs)
    if not np.all(features[:, -2] == 1) or not np.all(features[:, -1] == 0):
        raise AssertionError("Valid comparison features were rejected")
    measurement_matrix = features[:, 3:48].reshape(60, 15)
    selector = np.eye(15)[[0, 1, 2, 6, 7, 8]]
    pose_matrix = measurement_matrix @ selector.T
    noise_root = np.array([[0.7, 0, 0], [0.12, 0.8, 0], [0.003, -0.004, 0.02]])
    noise = np.kron(np.eye(20), noise_root @ noise_root.T)
    weighted_pose = np.linalg.solve(noise, pose_matrix)
    pose_information = pose_matrix.T @ weighted_pose
    pose_covariance = np.linalg.solve(pose_information, np.eye(6))
    compression = pose_covariance @ weighted_pose.T
    prior_root = rng.normal(size=(15, 15)) * 0.01
    prior = prior_root @ prior_root.T + np.eye(15) * 0.001
    innovation = measurement_matrix @ prior @ measurement_matrix.T + noise
    gain_raw = np.linalg.solve(innovation, measurement_matrix @ prior).T
    pose_innovation = selector @ prior @ selector.T + pose_covariance
    gain_pose = np.linalg.solve(pose_innovation, selector @ prior).T
    raw_factor = np.eye(15) - gain_raw @ measurement_matrix
    pose_factor = np.eye(15) - gain_pose @ selector
    posterior_raw = raw_factor @ prior @ raw_factor.T + gain_raw @ noise @ gain_raw.T
    posterior_pose = (
        pose_factor @ prior @ pose_factor.T + gain_pose @ pose_covariance @ gain_pose.T
    )
    # Independent error/noise draws test one linearized update, not flight ticks.
    prior_errors = rng.multivariate_normal(np.zeros(15), prior, size=1024).T
    observation_noise = rng.multivariate_normal(np.zeros(60), noise, size=1024).T
    residuals = measurement_matrix @ prior_errors + observation_noise
    compressed = compression @ residuals
    correction_raw, correction_pose = gain_raw @ residuals, gain_pose @ compressed
    error_raw = prior_errors - correction_raw
    nees = np.sum(error_raw * np.linalg.solve(posterior_raw, error_raw), axis=0)
    nis_raw = np.sum(residuals * np.linalg.solve(innovation, residuals), axis=0)
    nis_pose = np.sum(compressed * np.linalg.solve(pose_innovation, compressed), axis=0)
    discarded = residuals - pose_matrix @ compressed
    discarded_energy = np.sum(discarded * np.linalg.solve(noise, discarded), axis=0)
    diagonal_pose_noise = np.diag(np.diag(pose_covariance))
    diagonal_gain = np.linalg.solve(
        selector @ prior @ selector.T + diagonal_pose_noise, selector @ prior
    ).T
    return {
        "features": 20,
        "independent_draws": 1024,
        "pose_information_condition_number": float(np.linalg.cond(pose_information)),
        "maximum_correction_difference": float(
            np.max(np.abs(correction_raw - correction_pose))
        ),
        "maximum_covariance_difference": float(
            np.max(np.abs(posterior_raw - posterior_pose))
        ),
        "maximum_nis_decomposition_error": float(
            np.max(np.abs(nis_raw - nis_pose - discarded_energy))
        ),
        "mean_nees_15d": float(np.mean(nees)),
        "mean_raw_nis_60d": float(np.mean(nis_raw)),
        "mean_compressed_nis_6d": float(np.mean(nis_pose)),
        "mean_discarded_residual_energy_54d": float(np.mean(discarded_energy)),
        "diagonal_pose_covariance_negative_control_correction_difference": float(
            np.max(np.abs(diagonal_gain @ compressed - correction_raw))
        ),
        "scope": "One linearized Gaussian update with known landmarks, independent RGB-D noise and full 6D sufficient pose statistic. Not persistent SLAM, uncertain-map fusion, nonlinear registration or flight superiority.",
    }


def check(executable):
    rng = np.random.default_rng(20261009)
    requests, groups = [], []
    for _ in range(32):
        nominal = fixture(rng).astype(np.float32).astype(float)
        first = len(requests)
        requests.append(nominal)
        for step in (0.002, 0.001):
            for component in range(18):
                for sign in (1, -1):
                    perturbed = nominal.copy()
                    if component < 15:
                        perturbed[26 + component] = sign * step
                    else:
                        perturbed[7 + component - 15] += sign * step
                    requests.append(perturbed)
        groups.append((nominal, first))
    invalid = []
    nominal = fixture(rng)
    extrinsic = nominal[10:19].reshape(3, 3)
    for depth in (-1.0, 0.0, 0.05):
        rejected = nominal.copy()
        rejected[7:10] = nominal[4:7] + rotation(nominal[:4]) @ (
            nominal[19:22] + extrinsic @ np.array([0.1, -0.1, depth])
        )
        invalid.append(rejected)
    for focal_length in (0.0, -1.0, float("nan"), float("inf")):
        rejected = nominal.copy()
        rejected[22] = focal_length
        invalid.append(rejected)
    requests.extend(invalid)
    outputs = run(executable, requests)
    expected_error = jacobian_error = 0.0
    attitude_sign_control = lever_control = 0.0
    for nominal, first in groups:
        actual = outputs[first]
        if actual[-2] != 1 or actual[-1] != 0:
            raise AssertionError("Valid geometry rejected")
        expected_error = max(
            expected_error, float(np.max(np.abs(actual[:3] - observation(nominal))))
        )
        jacobian = np.column_stack(
            (actual[3:48].reshape(3, 15), actual[48:57].reshape(3, 3))
        )
        samples = outputs[first + 1 : first + 73].reshape(2, 18, 2, 59)
        if not np.all(samples[:, :, :, -2] == 1) or not np.all(
            samples[:, :, :, -1] == 0
        ):
            raise AssertionError("Valid perturbed geometry rejected")
        for step, sample in zip((0.002, 0.001), samples, strict=True):
            derivative = ((sample[:, 0, :3] - sample[:, 1, :3]) / (2 * step)).T
            # Pixel and metric depth rows have different units and scales.
            scale = np.maximum(1.0, np.max(np.abs(jacobian), axis=1))[:, None]
            jacobian_error = max(
                jacobian_error, float(np.max(np.abs(derivative - jacobian) / scale))
            )
            wrong_sign = jacobian.copy()
            wrong_sign[:, 6:9] *= -1
            attitude_sign_control = max(
                attitude_sign_control,
                float(np.max(np.abs(derivative - wrong_sign) / scale)),
            )
        no_lever = nominal.copy()
        no_lever[19:22] = 0
        lever_control = max(
            lever_control, float(np.max(np.abs(observation(no_lever) - actual[:3])))
        )
    invalid_outputs = outputs[-len(invalid) :]
    rejected_cleanly = bool(np.all(invalid_outputs[:, :-1] == 0))
    comparison = linearized_comparison(executable, rng)
    passed = (
        expected_error < 3e-4
        and jacobian_error < 0.003
        and attitude_sign_control > 0.9
        and lever_control > 1
        and rejected_cleanly
        and comparison["maximum_correction_difference"] < 1e-10
        and comparison["maximum_covariance_difference"] < 1e-12
        and comparison["maximum_nis_decomposition_error"] < 1e-8
        and comparison[
            "diagonal_pose_covariance_negative_control_correction_difference"
        ]
        > 1e-4
    )
    return {
        "complete": True,
        "passed": passed,
        "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "geometry_cases": len(groups),
        "generated_observations": len(requests) + 20,
        "maximum_independent_prediction_error": expected_error,
        "maximum_row_scaled_jacobian_error": jacobian_error,
        "wrong_attitude_sign_negative_control_error": attitude_sign_control,
        "missing_lever_arm_negative_control_prediction_error": lever_control,
        "invalid_geometry_cases": len(invalid),
        "invalid_geometry_rejected_with_zero_outputs": rejected_cleanly,
        "invalid_geometry_error_signal_status": invalid_outputs[:, -1]
        .astype(int)
        .tolist(),
        "matched_linearized_comparison": comparison,
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
