"""Compare deployment C with a full augmented Gaussian Joseph calculation."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np

from check_semidirect_bias import jacobian, pose_algebra, pose_exp, reset_jacobian
from check_semidirect_correction import physical_state, reference, state_vector


def augmented_update(
    covariance,
    cross,
    variance,
    residual,
    observation,
    noise,
    state_noise,
    datum_noise,
    axis,
    heading,
    geometry,
    joint_estimation=False,
):
    joint = np.block(
        [
            [covariance, cross[:, None], state_noise],
            [cross[None, :], np.array([[variance]]), datum_noise[None, :]],
            [state_noise.T, datum_noise[:, None], noise],
        ]
    )
    length = len(residual)
    measurement = np.c_[observation, np.zeros(length), np.eye(length)]
    innovation = measurement @ joint @ measurement.T
    gain = np.linalg.solve(innovation, (joint[:15] @ measurement.T).T).T
    projector = np.outer(axis, axis) if axis @ axis > 0.5 else np.eye(3)
    gain[6:9] = projector @ gain[6:9]
    if heading:
        gain[:6] = 0
        gain[9:12] = projector @ gain[9:12]
        gain[12:15] = 0
    correction = gain @ residual
    angle = np.linalg.norm(correction[6:9])
    if angle > 0.15:
        gain[6:9] *= 0.15 / angle
        correction = gain @ residual
    full_gain = np.r_[gain, np.zeros((1, length))]
    if joint_estimation:
        full_gain[15] = np.linalg.solve(innovation, joint[15] @ measurement.T)
    factor = np.c_[np.eye(16), np.zeros((16, length))] - full_gain @ measurement
    posterior = factor @ joint @ factor.T
    order = np.r_[np.arange(9), np.arange(12, 15), np.arange(9, 12)]
    tangent = correction[order]
    reset = np.eye(16)
    if geometry:
        reset[:15, :15] = reset_jacobian(tangent)[np.ix_(order, order)]
    else:
        reset[:9, :9] = jacobian(pose_algebra(tangent[:9]), -1)
    return reset @ posterior @ reset.T


class Kernel:
    def __init__(self, path):
        self.library = ctypes.CDLL(str(path.resolve()))
        array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        self.library.barometer_consider.argtypes = (
            [array] * 7 + [ctypes.c_int] * 2 + [array]
        )
        self.library.barometer_consider.restype = ctypes.c_int

    def __call__(
        self,
        nominal,
        covariance,
        cross,
        residual=None,
        observation=None,
        noise=None,
        state_noise=None,
        datum_noise=None,
        axis=None,
        gate=0.0,
        pressure=None,
        transition=None,
        bounds=None,
        geometry=False,
        heading=False,
        root=False,
        operation=0,
        joint=False,
    ):
        packed = np.r_[
            np.zeros(3) if residual is None else residual,
            (np.zeros((3, 15)) if observation is None else observation).ravel(),
            (np.eye(3) if noise is None else noise).ravel(),
            (np.zeros((15, 3)) if state_noise is None else state_noise).ravel(),
            np.zeros(3) if axis is None else axis,
            gate,
            np.zeros(3) if datum_noise is None else datum_noise,
        ]
        arguments = [
            nominal,
            covariance,
            cross,
            packed,
            np.eye(15) if transition is None else transition,
            np.ones(15) if bounds is None else bounds,
            np.zeros(6)
            if pressure is None
            else np.pad(pressure, (0, 6 - len(pressure))),
        ]
        output = np.empty(261, dtype=np.float32)
        status = self.library.barometer_consider(
            *[np.ascontiguousarray(value, dtype=np.float32) for value in arguments],
            int(geometry) | (int(heading) << 1) | (int(root) << 2) | (int(joint) << 3),
            operation,
            output,
        )
        if status:
            raise ValueError(f"Generated pressure-consider runtime error {status}")
        return (
            output[:16],
            output[16:241].reshape(15, 15),
            output[241:256],
            output[256],
            int(output[257]),
            bool(output[258]),
            output[259],
            output[260],
        )


def check(args):
    if args.trials < 1:
        raise ValueError("At least one randomized trial is required")
    kernel = Kernel(args.library)
    rng = np.random.default_rng(args.seed)
    maximum = dict(covariance=0.0, cross_covariance=0.0, nis=0.0)
    counts = dict(corrections=0, rejections=0, transformations=0, pressure=0)

    def compare(actual, expected, variance):
        covariance, cross = actual[1:3]
        scale = 1 + np.max(np.abs(expected))
        maximum["covariance"] = max(
            maximum["covariance"],
            float(np.max(np.abs(covariance - expected[:15, :15])) / scale),
        )
        maximum["cross_covariance"] = max(
            maximum["cross_covariance"],
            float(np.max(np.abs(cross - expected[:15, 15])) / scale),
        )
        joint = np.block(
            [
                [covariance.astype(float), cross[:, None]],
                [cross[None, :], np.array([[variance]])],
            ]
        )
        if np.linalg.eigvalsh(joint).min() < -2e-6 * (1 + np.linalg.norm(joint, 2)):
            raise ValueError("Deployment update lost joint positive semidefiniteness")

    for trial in range(args.trials):
        source = rng.normal(size=(19, 19))
        joint = source @ source.T / 19 + 0.1 * np.eye(19)
        covariance = joint[:15, :15].astype(np.float32).astype(float)
        cross = joint[:15, 15].astype(np.float32).astype(float)
        variance = float(np.float32(joint[15, 15]))
        noise = joint[16:, 16:].astype(np.float32).astype(float)
        state_noise = joint[:15, 16:].astype(np.float32).astype(float)
        datum_noise = joint[15, 16:].astype(np.float32).astype(float)
        if trial % 3 == 0:
            state_noise = np.zeros((15, 3))
            datum_noise = np.zeros(3)
        nominal = state_vector(
            (pose_exp(rng.normal(size=9) * 0.4), rng.normal(size=6) * 0.01)
        ).astype(np.float32)
        observation = rng.normal(size=(3, 15)).astype(np.float32).astype(float)
        residual = (
            (rng.normal(size=3) * (0.01 if trial % 2 else 3))
            .astype(np.float32)
            .astype(float)
        )
        gate = 1e-5 if trial % 7 == 6 else 0.0
        for geometry in (False, True):
            for heading in (False, True):
                axis = (
                    physical_state(nominal)[0][0][2].astype(np.float32).astype(float)
                    if heading
                    else np.zeros(3)
                )
                expected = augmented_update(
                    covariance,
                    cross,
                    variance,
                    residual,
                    observation,
                    noise,
                    state_noise,
                    datum_noise,
                    axis,
                    heading,
                    geometry,
                )
                mean, nav_covariance, nis, accepted, _ = reference(
                    physical_state(nominal),
                    covariance,
                    residual,
                    observation,
                    noise,
                    state_noise,
                    axis,
                    gate,
                    heading,
                    geometry,
                )
                if accepted:
                    np.testing.assert_allclose(
                        expected[:15, :15], nav_covariance, atol=1e-10, rtol=1e-10
                    )
                for root in (False, True):
                    actual = kernel(
                        nominal,
                        covariance,
                        cross,
                        residual,
                        observation,
                        noise,
                        state_noise,
                        datum_noise,
                        axis,
                        gate,
                        geometry=geometry,
                        heading=heading,
                        root=root,
                    )
                    assert actual[5] == accepted
                    assert actual[4] == (1 if accepted else 3)
                    maximum["nis"] = max(
                        maximum["nis"], abs(float(actual[3]) - nis) / (1 + nis)
                    )
                    if accepted:
                        compare(actual, expected, variance)
                        np.testing.assert_allclose(
                            actual[0], state_vector(mean), atol=3e-5, rtol=3e-5
                        )
                        counts["corrections"] += 1
                    else:
                        assert np.array_equal(actual[0], nominal)
                        assert np.array_equal(actual[1], covariance)
                        assert np.array_equal(actual[2], cross)
                        counts["rejections"] += 1

        bounds = np.linspace(0.2, 1.4, 15).astype(np.float32).astype(float)
        for operation in (2, 3, 4, 5):
            transformation = np.eye(16)
            addend = np.zeros((16, 16))
            transition = np.eye(15) + rng.normal(size=(15, 15)) * 0.03
            transition = transition.astype(np.float32).astype(float)
            if operation == 2:
                transformation[:15, :15] = transition
                addend[:15, :15] = (
                    np.diag(np.repeat([0, 0.02, 0.01, 0.03, 0.04], 3)) * 0.02
                )
            elif operation == 3:
                transformation[:3, 3:6] = 0.02 * np.eye(3)
                addend[9:15, 9:15] = np.diag(np.repeat([0.03, 0.04], 3)) * 0.02
            elif operation == 4:
                scale = np.minimum(1, np.sqrt(bounds / np.diag(covariance)))
                transformation[:15, :15] = np.diag(scale)
            else:
                transformation[:6] = 0
                addend[:6, :6] = np.diag(bounds[:6])
            prior = joint[:16, :16].copy()
            prior[:15, :15], prior[:15, 15], prior[15, :15] = covariance, cross, cross
            prior[15, 15] = variance
            expected = transformation @ prior @ transformation.T + addend
            for root in (False, True):
                actual = kernel(
                    nominal,
                    covariance,
                    cross,
                    transition=transition,
                    bounds=bounds,
                    root=root,
                    operation=operation,
                )
                compare(actual, expected, variance)
                counts["transformations"] += 1

    nominal = np.r_[np.zeros(6), 1, np.zeros(9)].astype(np.float32)
    for root in (False, True):
        covariance, cross, variance = np.eye(15), np.zeros(15), 0.25
        for packet in range(300):
            observation = np.zeros((1, 15))
            observation[0, 2] = 1
            expected = augmented_update(
                covariance,
                cross,
                variance,
                np.zeros(1),
                observation,
                np.array([[variance + 0.01]]),
                cross[:, None],
                np.array([variance]),
                np.zeros(3),
                False,
                False,
            )
            actual = kernel(
                nominal,
                covariance,
                cross,
                pressure=[variance, 0, 0, 0.01, 0],
                root=root,
                operation=1,
            )
            assert actual[5] and actual[4] == 1
            compare(actual, expected, variance)
            covariance, cross = actual[1].astype(float), actual[2].astype(float)
            counts["pressure"] += 1

        for pressure in ([0.001, 0.1, 0, 0.01, 0.1],):
            actual = kernel(
                nominal, covariance, cross, pressure=pressure, root=root, operation=1
            )
            assert not actual[5] and actual[4] == 5
            assert np.array_equal(actual[1], covariance.astype(np.float32))
            assert np.array_equal(actual[2], cross.astype(np.float32))
            counts["rejections"] += 1
        assert 0.2 <= covariance[2, 2] < 0.21
        np.testing.assert_allclose(cross[2], -covariance[2, 2], atol=3e-5)

        for age in (-0.01, 0.3, 0.1):
            pressure = [variance, 0.1, 0, 0.01, age]
            if 0 <= age <= 0.25:
                observation[0, 5] = -age
                observation[0, 14] = -0.5 * age**2
                observed_variance = variance - 0.1 * age
                # Start this delayed random-walk case from an independent prior.
                covariance, cross = np.eye(15), np.zeros(15)
                actual = kernel(
                    nominal,
                    covariance,
                    cross,
                    pressure=pressure,
                    root=root,
                    operation=1,
                )
                expected = augmented_update(
                    covariance,
                    cross,
                    variance,
                    np.zeros(1),
                    observation,
                    np.array([[observed_variance + 0.01]]),
                    cross[:, None],
                    np.array([observed_variance]),
                    np.zeros(3),
                    False,
                    False,
                )
                compare(actual, expected, variance)
                assert actual[5]
            else:
                actual = kernel(
                    nominal,
                    covariance,
                    cross,
                    pressure=pressure,
                    root=root,
                    operation=1,
                )
                assert not actual[5] and actual[4] == 6
                assert np.array_equal(actual[1], covariance.astype(np.float32))
                assert np.array_equal(actual[2], cross.astype(np.float32))
            counts["pressure"] += 1

    assert maximum["covariance"] < 2e-5, maximum
    assert maximum["cross_covariance"] < 2e-5, maximum
    assert maximum["nis"] < 1e-5, maximum
    result = dict(
        seed=args.seed,
        trials=args.trials,
        counts=counts,
        maximum_errors=maximum,
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        reference="Full augmented Gaussian Joseph update with zero datum gain, followed by the matching Lie reset; no covariance repair.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261008)
    parser.add_argument("--trials", type=int, default=48)
    check(parser.parse_args())
