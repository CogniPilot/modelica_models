"""Compare both generated Modelica visual corrections with an independent oracle."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from check_visual_tangent import rotation


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def exponential(tangent):
    angle = np.linalg.norm(tangent[6:9])
    cross = skew(tangent[6:9])
    if angle < 1e-5:
        left = np.eye(3) + cross / 2 + cross @ cross / 6
        attitude = np.eye(3) + cross + cross @ cross / 2
    else:
        left = (
            np.eye(3)
            + (1 - np.cos(angle)) / angle**2 * cross
            + (angle - np.sin(angle)) / angle**3 * cross @ cross
        )
        attitude = (
            np.eye(3)
            + np.sin(angle) / angle * cross
            + (1 - np.cos(angle)) / angle**2 * cross @ cross
        )
    group = np.eye(5)
    group[:3, :3] = attitude
    group[:3, 3] = left @ tangent[:3]
    group[:3, 4] = left @ tangent[3:6]
    return group


def logarithm(group):
    attitude = group[:3, :3]
    vector = (
        np.array(
            [
                attitude[2, 1] - attitude[1, 2],
                attitude[0, 2] - attitude[2, 0],
                attitude[1, 0] - attitude[0, 1],
            ]
        )
        / 2
    )
    sine = np.linalg.norm(vector)
    angle = np.arctan2(sine, (np.trace(attitude) - 1) / 2)
    theta = vector * (angle / sine if sine > 1e-12 else 1)
    cross = skew(theta)
    inverse_left = (
        np.eye(3)
        - cross / 2
        + cross
        @ cross
        * (1 / 12 if angle < 1e-5 else (1 - angle / (2 * np.tan(angle / 2))) / angle**2)
    )
    return np.r_[inverse_left @ group[:3, 3], inverse_left @ group[:3, 4], theta]


def reference(
    quaternion,
    position,
    prior,
    landmarks,
    observations,
    noise,
    extrinsic,
    lever,
    intrinsics,
):
    attitude = rotation(quaternion)
    residuals, jacobians = [], []
    for landmark, observed in zip(landmarks, observations, strict=True):
        body = attitude.T @ (landmark - position)
        camera = extrinsic.T @ (body - lever)
        x, y, depth = camera
        expected = np.r_[intrinsics[:2] * camera[:2] / depth + intrinsics[2:], depth]
        projection = np.array(
            [
                [intrinsics[0] / depth, 0, -intrinsics[0] * x / depth**2],
                [0, intrinsics[1] / depth, -intrinsics[1] * y / depth**2],
                [0, 0, 1],
            ]
        )
        jacobian = np.zeros((3, 15))
        jacobian[:, :3] = -projection @ extrinsic.T
        jacobian[:, 6:9] = projection @ extrinsic.T @ skew(body)
        residuals.append(observed - expected)
        jacobians.append(jacobian)
    residual, jacobian = np.concatenate(residuals), np.vstack(jacobians)
    innovation = jacobian @ prior @ jacobian.T + noise
    gain = np.linalg.solve(innovation, jacobian @ prior).T
    correction = gain @ residual
    increment = exponential(correction[:9])
    corrected_position = position + attitude @ increment[:3, 3]
    corrected_velocity = attitude @ increment[:3, 4]
    corrected_rotation = attitude @ increment[:3, :3]
    factor = np.eye(15) - gain @ jacobian
    posterior = factor @ prior @ factor.T + gain @ noise @ gain.T
    reset = np.eye(15)
    inverse = np.linalg.inv(increment)
    for component in range(9):
        delta = np.eye(9)[component] * 1e-6
        reset[:9, component] = (
            logarithm(inverse @ exponential(correction[:9] + delta))
            - logarithm(inverse @ exponential(correction[:9] - delta))
        ) / 2e-6
    return (
        corrected_position,
        corrected_velocity,
        corrected_rotation,
        correction[9:15],
        reset @ posterior @ reset.T,
    )


def check(executable):
    rng = np.random.default_rng(20261010)
    inputs, fixtures = [], []
    for _ in range(64):
        quaternion = rng.normal(size=4)
        quaternion /= np.linalg.norm(quaternion)
        camera_quaternion = rng.normal(size=4)
        camera_quaternion /= np.linalg.norm(camera_quaternion)
        extrinsic = rotation(camera_quaternion)
        position, lever = rng.normal(size=(2, 3))
        lever *= 0.1
        camera_points = np.column_stack(
            (rng.uniform(-2, 2, (4, 2)), rng.uniform(3, 8, 4))
        )
        landmarks = (
            position
            + (rotation(quaternion) @ (lever[:, None] + extrinsic @ camera_points.T)).T
        )
        intrinsics = np.array([400, 420, 320, 240])
        observations = np.column_stack(
            (
                camera_points[:, :2] / camera_points[:, 2:3] * intrinsics[:2]
                + intrinsics[2:],
                camera_points[:, 2],
            )
        )
        noise_root = np.kron(
            np.eye(4), np.array([[0.7, 0, 0], [0.1, 0.8, 0], [0.003, -0.002, 0.03]])
        )
        noise = noise_root @ noise_root.T
        common = rng.normal(size=12) * np.tile([0.05, 0.05, 0.001], 4)
        noise += np.outer(common, common)
        observations += rng.multivariate_normal(np.zeros(12), noise).reshape(4, 3)
        scales = np.r_[
            np.full(6, 0.02), np.full(3, 0.005), np.full(3, 0.001), np.full(3, 0.005)
        ]
        prior_root = rng.normal(size=(15, 15)) * scales[:, None] / 5
        prior = prior_root @ prior_root.T + np.diag(scales**2)
        fields = [
            quaternion,
            position,
            prior,
            landmarks,
            observations,
            noise,
            extrinsic,
            lever,
            intrinsics,
        ]
        row = (
            np.concatenate([np.asarray(field).ravel() for field in fields])
            .astype(np.float32)
            .astype(float)
        )
        inputs.append(row)
        fixtures.append(row)
    behind_camera = inputs[0].copy()
    quaternion, position = behind_camera[:4], behind_camera[4:7]
    extrinsic = behind_camera[400:409].reshape(3, 3)
    lever = behind_camera[409:412]
    behind_camera[232:244] = np.tile(
        position + rotation(quaternion) @ (lever + extrinsic @ np.array([0, 0, -5])), 4
    )
    invalid_covariance = inputs[0].copy()
    invalid_covariance[256] = -1
    degenerate = inputs[0].copy()
    degenerate[232:244] = np.tile(degenerate[232:235], 4)
    inputs.extend((behind_camera, invalid_covariance, degenerate))
    result = subprocess.run(
        [str(executable)],
        input="\n".join(" ".join(format(x, ".9g") for x in row) for row in inputs),
        capture_output=True,
        text=True,
        check=True,
    )
    outputs = np.array(
        [[float(x) for x in line.split()] for line in result.stdout.splitlines()]
    )
    if outputs.shape != (67, 487) or not np.isfinite(outputs).all():
        raise AssertionError(f"Invalid generated output {outputs.shape}")
    state_difference = covariance_difference = reference_state_error = (
        reference_covariance_error
    ) = 0.0
    accepted = int(np.sum(np.all(outputs[:64, 482:484] == 1, axis=1)))
    controls = outputs[64:]
    refusal_preserved_prior = True
    for input_row, output in zip(inputs[64:66], controls[:2], strict=True):
        expected_state = np.r_[input_row[4:7], np.zeros(3), input_row[:4], np.zeros(6)]
        for state, covariance in (
            (output[:16], output[32:257]),
            (output[16:32], output[257:482]),
        ):
            refusal_preserved_prior = (
                refusal_preserved_prior
                and np.allclose(state, expected_state, rtol=1e-8, atol=1e-9)
                and np.allclose(covariance, input_row[7:232], rtol=1e-8, atol=1e-12)
            )
    control_passed = bool(
        np.all(controls[:2, 482:484] == 0)
        and np.array_equal(controls[2, 482:484], [0, 1])
        and refusal_preserved_prior
    )
    for row, output in zip(fixtures, outputs[:64], strict=True):
        if not np.all(output[482:484] == 1) or output[-1] != 0:
            continue
        fields = np.split(row, [4, 7, 232, 244, 256, 400, 409, 412])
        (
            quaternion,
            position,
            prior,
            landmarks,
            observed,
            noise,
            extrinsic,
            lever,
            intrinsics,
        ) = fields
        prior, landmarks, observed, noise, extrinsic = (
            prior.reshape(15, 15),
            landmarks.reshape(4, 3),
            observed.reshape(4, 3),
            noise.reshape(12, 12),
            extrinsic.reshape(3, 3),
        )
        (
            expected_position,
            expected_velocity,
            expected_rotation,
            expected_bias,
            expected_covariance,
        ) = reference(
            quaternion,
            position,
            prior,
            landmarks,
            observed,
            noise,
            extrinsic,
            lever,
            intrinsics,
        )
        state_difference = max(
            state_difference, float(np.max(np.abs(output[:16] - output[16:32])))
        )
        covariance_difference = max(
            covariance_difference,
            float(np.max(np.abs(output[32:257] - output[257:482]))),
        )
        for state, covariance in (
            (output[:16], output[32:257].reshape(15, 15)),
            (output[16:32], output[257:482].reshape(15, 15)),
        ):
            np.linalg.cholesky(covariance)
            state_error = max(
                np.max(np.abs(state[:3] - expected_position)),
                np.max(np.abs(state[3:6] - expected_velocity)),
                np.max(np.abs(rotation(state[6:10]) - expected_rotation)),
                np.max(np.abs(state[10:16] - expected_bias)),
            )
            reference_state_error = max(reference_state_error, float(state_error))
            reference_covariance_error = max(
                reference_covariance_error,
                float(np.max(np.abs(covariance - expected_covariance))),
            )
    passed = (
        accepted == 64
        and np.all(outputs[:, -1] == 0)
        and state_difference < 2e-4
        and covariance_difference < 1e-6
        and reference_state_error < 2e-4
        and reference_covariance_error < 1e-6
        and control_passed
    )
    return dict(
        complete=True,
        passed=bool(passed),
        cases=64,
        both_accepted=accepted,
        runtime_error_rows=int(np.count_nonzero(outputs[:, -1])),
        negative_controls_passed=control_passed,
        negative_control_acceptance={
            "behind_camera": controls[0, 482:484].astype(int).tolist(),
            "indefinite_measurement_covariance": controls[1, 482:484]
            .astype(int)
            .tolist(),
            "rank_deficient_pose_compression": controls[2, 482:484]
            .astype(int)
            .tolist(),
        },
        rejected_updates_preserve_prior=bool(refusal_preserved_prior),
        maximum_loose_tight_state_difference=state_difference,
        maximum_loose_tight_covariance_difference=covariance_difference,
        maximum_independent_state_error=reference_state_error,
        maximum_independent_reset_covariance_error=reference_covariance_error,
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        scope="Both visual paths and ESKF correction/injection/reset execute from generated Modelica. Single independent fixed-map updates; not persistent SLAM or a GPS flight comparison.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = check(args.executable.resolve())
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["passed"]:
        raise SystemExit(1)
