"""Independent group-action and differential checks for a 15-state bias geometry."""

import argparse
import ctypes
import json
from pathlib import Path
import numpy as np


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0.0]])


def rotation_exp(vector):
    angle = np.linalg.norm(vector)
    cross = skew(vector)
    return (
        np.eye(3)
        + np.sinc(angle / np.pi) * cross
        + (0.5 * np.sinc(angle / (2 * np.pi)) ** 2) * cross @ cross
    )


def rotation_log(rotation):
    vector = (
        np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        )
        / 2
    )
    sine = np.linalg.norm(vector)
    cosine = (np.trace(rotation) - 1) / 2
    return vector * (np.arctan2(sine, cosine) / sine if sine > 1e-12 else 1)


def rotation_jacobian(vector):
    angle = np.linalg.norm(vector)
    cross = skew(vector)
    cubic = (
        (angle - np.sin(angle)) / angle**3
        if angle > 1e-4
        else 1 / 6 - angle**2 / 120 + angle**4 / 5040
    )
    return (
        np.eye(3)
        + 0.5 * np.sinc(angle / (2 * np.pi)) ** 2 * cross
        + cubic * cross @ cross
    )


def pose_exp(tangent):
    rotation = rotation_exp(tangent[6:9])
    jacobian = rotation_jacobian(tangent[6:9])
    return rotation, jacobian @ tangent[:3], jacobian @ tangent[3:6]


def pose_product(left, right):
    rotation, position, velocity = left
    other_rotation, other_position, other_velocity = right
    return (
        rotation @ other_rotation,
        position + rotation @ other_position,
        velocity + rotation @ other_velocity,
    )


def pose_inverse(pose):
    rotation, position, velocity = pose
    return rotation.T, -rotation.T @ position, -rotation.T @ velocity


def pose_log(pose):
    rotation, position, velocity = pose
    angle = rotation_log(rotation)
    jacobian = rotation_jacobian(angle)
    return np.r_[
        np.linalg.solve(jacobian, position), np.linalg.solve(jacobian, velocity), angle
    ]


def bias_adjoint(pose):
    rotation, _, velocity = pose
    return np.block(
        [[rotation, skew(velocity) @ rotation], [np.zeros((3, 3)), rotation]]
    )


def pose_adjoint(pose):
    rotation, position, velocity = pose
    zero = np.zeros((3, 3))
    return np.block(
        [
            [rotation, zero, skew(position) @ rotation],
            [zero, rotation, skew(velocity) @ rotation],
            [zero, zero, rotation],
        ]
    )


def bias_algebra(tangent):
    translation, rotation = tangent[:3], tangent[3:]
    return np.block(
        [[skew(rotation), skew(translation)], [np.zeros((3, 3)), skew(rotation)]]
    )


def pose_algebra(tangent):
    position, velocity, rotation = tangent[:3], tangent[3:6], tangent[6:9]
    zero = np.zeros((3, 3))
    return np.block(
        [
            [skew(rotation), zero, skew(position)],
            [zero, skew(rotation), skew(velocity)],
            [zero, zero, skew(rotation)],
        ]
    )


def jacobian(algebra, side):
    result = np.eye(len(algebra))
    power = result.copy()
    coefficient = 1
    for order in range(1, 25):
        power = power @ algebra
        coefficient *= side / (order + 1)
        result += coefficient * power
    return result


def coordinate_map(pose):
    result = np.zeros((15, 15))
    result[:9, :9] = pose_adjoint(pose)
    result[9:, 9:] = -bias_adjoint(pose)
    return result


def retract(state, tangent):
    pose, bias = state
    return pose_product(pose, pose_exp(tangent[:9])), bias + jacobian(
        bias_algebra(tangent[3:9]), -1
    ) @ tangent[9:]


def local_error(reference, truth):
    pose, bias = reference
    other_pose, other_bias = truth
    tangent = pose_log(pose_product(pose_inverse(pose), other_pose))
    return np.r_[
        tangent,
        np.linalg.solve(jacobian(bias_algebra(tangent[3:9]), -1), other_bias - bias),
    ]


def reset_jacobian(tangent, orders=12):
    result = np.zeros((15, 15))
    result[:9, :9] = jacobian(pose_algebra(tangent[:9]), -1)
    algebra = bias_algebra(tangent[3:9])
    result[9:, 9:] = jacobian(algebra, -1)
    power_vector = tangent[9:].copy()
    sensitivity = np.zeros((6, 6))
    derivative = sensitivity.copy()
    coefficient = 1
    for order in range(1, orders + 1):
        sensitivity = algebra @ sensitivity - bias_algebra(power_vector)
        power_vector = algebra @ power_vector
        coefficient *= -1 / (order + 1)
        derivative += coefficient * sensitivity
    result[9:, 3:9] = derivative
    return result


def group_product(left, right):
    pose, bias = left
    other_pose, other_bias = right
    return pose_product(pose, other_pose), bias + bias_adjoint(pose) @ other_bias


def group_of(state):
    pose, bias = state
    return pose, -bias_adjoint(pose) @ bias


def group_exp(tangent):
    return pose_exp(tangent[:9]), jacobian(bias_algebra(tangent[3:9]), 1) @ tangent[9:]


def state_of(group):
    pose, bias = group
    return pose, -np.linalg.solve(bias_adjoint(pose), bias)


def body_dynamics(rate, force):
    result = np.zeros((15, 15))
    rate_cross = skew(rate)
    for start in (0, 3, 6):
        result[start : start + 3, start : start + 3] = -rate_cross
    result[:3, 3:6] = np.eye(3)
    result[3:6, 6:9] = -skew(force)
    result[3:6, 9:12] = -np.eye(3)
    result[6:9, 12:15] = -np.eye(3)
    return result


def transformed_dynamics(pose, rate, force, gravity):
    rotation, position, velocity = pose
    acceleration = rotation @ force + gravity
    world_rate = rotation @ rate
    result = np.zeros((15, 15))
    result[:3, 3:6] = np.eye(3)
    result[:3, 12:15] = skew(position)
    result[3:6, 6:9] = skew(gravity)
    result[3:6, 9:12] = np.eye(3)
    result[6:9, 12:15] = np.eye(3)
    result[9:12, 9:12] = skew(world_rate)
    result[12:15, 12:15] = skew(world_rate)
    result[9:12, 12:15] = skew(acceleration + np.cross(velocity, world_rate))
    return result


def advance_pose(pose, rate, force, gravity, interval):
    rotation, position, velocity = pose
    acceleration = rotation @ force + gravity
    return (
        rotation @ rotation_exp(rate * interval),
        position + velocity * interval,
        velocity + acceleration * interval,
    )


def check(args):
    if args.trials < 1:
        raise ValueError("At least one independent trial is required")
    library = ctypes.CDLL(str(args.library.resolve())) if args.library else None
    if library:
        array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        library.semi_direct_correction.argtypes = [array, array, array, array]
        library.semi_direct_correction.restype = ctypes.c_int
    maximum_generated_mean_error = maximum_generated_reset_error = 0.0
    rng = np.random.default_rng(args.seed)
    maximum_reset_error = maximum_dynamics_error = maximum_group_error = 0.0
    for trial in range(args.trials):
        pose = pose_exp(rng.normal(size=9) * np.r_[np.full(6, 5), np.full(3, 0.7)])
        state = pose, rng.normal(size=6) * 0.1
        correction = (
            rng.normal(size=15)
            * np.r_[np.full(6, 0.8), np.full(3, 0.15), np.full(6, 0.02)]
        )
        if trial == 0:
            correction[:] = 0
        elif trial == 1:
            correction[6:9] = 0
        elif trial == 2:
            correction[6:9] *= 1e-7
        mean = retract(state, correction)
        pose_only = correction.copy()
        pose_only[9:] = 0
        assert np.array_equal(retract(state, pose_only)[1], state[1])
        step = 2e-6
        numerical = np.column_stack(
            [
                (
                    local_error(mean, retract(state, correction + step * axis))
                    - local_error(mean, retract(state, correction - step * axis))
                )
                / (2 * step)
                for axis in np.eye(15)
            ]
        )
        reset = reset_jacobian(correction)
        maximum_reset_error = max(
            maximum_reset_error, float(np.max(np.abs(reset - numerical)))
        )
        if library:
            order = np.r_[np.arange(9), np.arange(12, 15), np.arange(9, 12)]
            angle = rotation_log(pose[0])
            quaternion = np.r_[
                np.cos(np.linalg.norm(angle) / 2),
                0.5 * np.sinc(np.linalg.norm(angle) / (2 * np.pi)) * angle,
            ]
            nominal = np.r_[
                pose[1], pose[2], quaternion, state[1][3:], state[1][:3]
            ].astype(np.float32)
            corrected = np.empty(16, dtype=np.float32)
            generated_reset = np.empty((15, 15), dtype=np.float32)
            status = library.semi_direct_correction(
                nominal,
                correction[order].astype(np.float32),
                corrected,
                generated_reset,
            )
            if status:
                raise ValueError(
                    f"Generated semi-direct correction error {status} at trial {trial}"
                )
            vector = corrected[7:10].astype(float)
            magnitude = np.linalg.norm(vector)
            rotation_vector = vector * (
                2 * np.arctan2(magnitude, corrected[6]) / magnitude
                if magnitude > 1e-12
                else 2
            )
            generated_mean = (
                (
                    rotation_exp(rotation_vector),
                    corrected[:3].astype(float),
                    corrected[3:6].astype(float),
                ),
                np.r_[corrected[13:16], corrected[10:13]].astype(float),
            )
            maximum_generated_mean_error = max(
                maximum_generated_mean_error,
                float(np.max(np.abs(local_error(mean, generated_mean)))),
            )
            maximum_generated_reset_error = max(
                maximum_generated_reset_error,
                float(np.max(np.abs(reset[np.ix_(order, order)] - generated_reset))),
            )
        group_mean = state_of(
            group_product(group_exp(coordinate_map(pose) @ correction), group_of(state))
        )
        maximum_group_error = max(
            maximum_group_error, float(np.max(np.abs(local_error(mean, group_mean))))
        )
        rate = rng.normal(size=3)
        force = rng.normal(size=3) * 4
        gravity = np.array([0, 0, -9.81])
        mapping = coordinate_map(pose)
        inverse = np.linalg.inv(mapping)
        derivative = (
            coordinate_map(advance_pose(pose, rate, force, gravity, step))
            - coordinate_map(advance_pose(pose, rate, force, gravity, -step))
        ) / (2 * step)
        transformed = (mapping @ body_dynamics(rate, force) + derivative) @ inverse
        maximum_dynamics_error = max(
            maximum_dynamics_error,
            float(
                np.max(
                    np.abs(
                        transformed - transformed_dynamics(pose, rate, force, gravity)
                    )
                )
            ),
        )
    assert maximum_reset_error < 2e-8, maximum_reset_error
    assert maximum_group_error < 1e-12, maximum_group_error
    assert maximum_dynamics_error < 2e-8, maximum_dynamics_error
    assert maximum_generated_mean_error < 2e-5, maximum_generated_mean_error
    assert maximum_generated_reset_error < 2e-5, maximum_generated_reset_error
    result = dict(
        trials=args.trials,
        seed=args.seed,
        maximum_reset_jacobian_error=maximum_reset_error,
        maximum_group_action_error=maximum_group_error,
        maximum_dynamics_coordinate_transform_error=maximum_dynamics_error,
        generated_helpers_checked=library is not None,
        maximum_generated_mean_error=maximum_generated_mean_error if library else None,
        maximum_generated_reset_error=maximum_generated_reset_error
        if library
        else None,
        production_filter_integrated=False,
        performance_evaluated=False,
        note="Independent physical retraction, semidirect group multiplication, finite-difference reset and body-to-world dynamics checks; bias ordering is accelerometer then gyroscope for reuse of translation-first SE3 helpers.",
    )
    print(json.dumps(result, indent=2))
    args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--library", type=Path)
    parser.add_argument("--trials", type=int, default=120)
    parser.add_argument("--seed", type=int, default=20261007)
    check(parser.parse_args())
