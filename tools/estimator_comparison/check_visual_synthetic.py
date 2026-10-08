"""Independently check generated synthetic landmark projections and visibility."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from check_visual_tangent import rotation


def check(executable):
    rng = np.random.default_rng(20271051)
    rows, expected, masks = [], [], []
    labels = []
    for case in range(256):
        quaternion = rng.normal(size=4)
        quaternion /= np.linalg.norm(quaternion)
        attitude = rotation(quaternion)
        camera_quaternion = rng.normal(size=4)
        camera_quaternion /= np.linalg.norm(camera_quaternion)
        extrinsic = rotation(camera_quaternion)
        position = rng.normal(size=3)
        lever = rng.normal(size=3) * 0.2
        points_camera = np.column_stack(
            (rng.uniform(-4, 4, (4, 2)), rng.uniform(-1, 12, 4))
        )
        landmarks = (
            position + (attitude @ (lever[:, None] + extrinsic @ points_camera.T)).T
        )
        intrinsics = np.array([350.0, 370.0, 320.0, 240.0])
        size, depth_range = np.array([640, 480]), np.array([0.1, 10.0])
        noise = rng.normal(size=(4, 3)) * [0.7, 0.8, 0.03]
        detected = rng.random(4) > 0.15
        available = case % 13 != 0
        label = "random_geometry"
        if case < 8:
            points_camera = np.array([[-1, -1, 4], [1, -1, 5], [-1, 1, 6], [1, 1, 7]])
            landmarks = (
                position + (attitude @ (lever[:, None] + extrinsic @ points_camera.T)).T
            )
            detected[:] = True
            available = True
            label = [
                "visible",
                "camera_outage",
                "missed_detection",
                "behind_camera",
                "too_far",
                "outside_image",
                "invalid_calibration",
                "invalid_noise",
            ][case]
            if case == 1:
                available = False
            elif case == 2:
                detected[2] = False
            elif case in (3, 4, 5):
                points_camera[1] = {3: [0, 0, -5], 4: [0, 0, 11], 5: [20, 0, 4]}[case]
                landmarks = (
                    position
                    + (attitude @ (lever[:, None] + extrinsic @ points_camera.T)).T
                )
            elif case == 6:
                intrinsics[0] = -1
            elif case == 7:
                noise[1, 0] = np.nan
        row = (
            np.concatenate(
                [
                    np.asarray(field).ravel()
                    for field in (
                        position,
                        quaternion,
                        landmarks,
                        extrinsic,
                        lever,
                        intrinsics,
                        size,
                        depth_range,
                        noise,
                        detected,
                        [available],
                    )
                ]
            )
            .astype(np.float32)
            .astype(float)
        )
        (
            position,
            quaternion,
            landmarks,
            extrinsic,
            lever,
            intrinsics,
            size,
            depth_range,
            noise,
            detected,
            available,
        ) = np.split(row, [3, 7, 19, 28, 31, 35, 37, 39, 51, 55])
        landmarks, extrinsic, noise = (
            landmarks.reshape(4, 3),
            extrinsic.reshape(3, 3),
            noise.reshape(4, 3),
        )
        attitude = rotation(quaternion)
        points = (landmarks - position - attitude @ lever) @ (attitude @ extrinsic)
        measurements = np.zeros((4, 3))
        visible = np.zeros(4, dtype=bool)
        for landmark, point in enumerate(points):
            if (
                not available[0]
                or not detected[landmark]
                or intrinsics[0] <= 0
                or not depth_range[0] <= point[2] <= depth_range[1]
                or not np.isfinite(noise[landmark]).all()
            ):
                continue
            projected = np.r_[
                point[:2] / point[2] * intrinsics[:2] + intrinsics[2:], point[2]
            ]
            measured = projected + noise[landmark]
            visible[landmark] = (
                np.all(projected[:2] >= 0)
                and np.all(projected[:2] < size)
                and np.all(measured[:2] >= 0)
                and np.all(measured[:2] < size)
                and depth_range[0] <= measured[2] <= depth_range[1]
            )
            if visible[landmark]:
                measurements[landmark] = measured
        rows.append(row)
        expected.append(measurements)
        masks.append(visible)
        labels.append(label)
    run = subprocess.run(
        [str(executable)],
        input="\n".join(
            " ".join(format(value, ".9g") for value in row) for row in rows
        ),
        capture_output=True,
        text=True,
        check=True,
    )
    outputs = np.array(
        [[float(value) for value in line.split()] for line in run.stdout.splitlines()]
    )
    if outputs.shape != (256, 89) or not np.isfinite(outputs).all():
        raise AssertionError(
            f"Invalid generated output shape or values: {outputs.shape}"
        )
    observations = outputs[:, :12].reshape(-1, 4, 3)
    visibility = outputs[:, 12:16].astype(bool)
    error = float(np.max(np.abs(observations - np.array(expected))))
    grid = np.array([[-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 1, 0]])
    matches = np.array_equal(visibility, masks)
    default_grid = np.array(
        [[x, y, 0] for y in (-0.75, 0, 0.75) for x in (-1.125, -0.375, 0.375, 1.125)]
    )
    default_expected = np.column_stack(
        (
            320 + 350 * default_grid[:, 0] / 4,
            240 - 350 * default_grid[:, 1] / 4,
            np.full(12, 4),
        )
    )
    default_error = float(np.max(np.abs(outputs[:, 28:64] - default_expected.ravel())))
    default_scene_passed = (
        default_error < 0.002
        and np.all(outputs[:, 64:76] == 1)
        and np.array_equal(outputs[:, 76:88], np.tile(np.arange(1, 13), (256, 1)))
    )
    controls = {
        labels[case]: visibility[case].astype(int).tolist() for case in range(8)
    }
    expected_status = np.zeros(256)
    expected_status[7] = 4
    status_matches = np.array_equal(outputs[:, -1], expected_status)
    invisible_zero = bool(np.all(observations[~visibility] == 0))
    return dict(
        complete=True,
        passed=bool(
            matches
            and default_scene_passed
            and error < 0.002
            and status_matches
            and invisible_zero
            and np.array_equal(outputs[:, 16:28], np.tile(grid.ravel(), (256, 1)))
        ),
        cases=256,
        visibility_matches_independent_oracle=matches,
        maximum_projection_error=error,
        default_scene_passed=bool(default_scene_passed),
        maximum_default_scene_projection_error=default_error,
        negative_controls=controls,
        invisible_observations_zero=invisible_zero,
        runtime_error_rows=int(np.count_nonzero(outputs[:, -1])),
        error_status_matches_contract=status_matches,
        invalid_noise_error_status=int(outputs[7, -1]),
        unexpected_runtime_error_rows=int(
            np.count_nonzero(outputs[:, -1] != expected_status)
        ),
        executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
        scope="Generated Modelica scene projection, explicit additive errors, visibility and fixed grid; not image processing or uncertain-map estimation.",
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
