"""Check joint pressure-bias estimation against independent augmented updates."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from check_barometer_consider import Kernel, augmented_update
from check_semidirect_bias import pose_exp
from check_semidirect_correction import physical_state, reference, state_vector


def check(args):
    kernel = Kernel(args.library)
    rng = np.random.default_rng(args.seed)
    maximum = dict(joint_covariance=0.0, bias_mean=0.0)
    counts = dict(accepted=0, rejected=0, constrained=0, pressure=0, gps=0)

    def compare(actual, expected):
        joint = np.block(
            [
                [actual[1].astype(float), actual[2][:, None]],
                [actual[2][None, :], np.array([[actual[7]]])],
            ]
        )
        error = np.max(np.abs(joint - expected)) / (1 + np.max(np.abs(expected)))
        maximum["joint_covariance"] = max(maximum["joint_covariance"], float(error))
        if np.linalg.eigvalsh(joint).min() < -2e-6 * (1 + np.linalg.norm(joint, 2)):
            raise ValueError(
                "Joint deployment covariance lost positive semidefiniteness"
            )

    for trial in range(args.trials):
        source = rng.normal(size=(19, 19))
        joint = (
            (source @ source.T / 19 + 0.1 * np.eye(19)).astype(np.float32).astype(float)
        )
        covariance, cross, variance = joint[:15, :15], joint[:15, 15], joint[15, 15]
        noise, state_noise, datum_noise = (
            joint[16:, 16:],
            joint[:15, 16:],
            joint[15, 16:],
        )
        if trial % 3 == 0:
            state_noise, datum_noise = np.zeros((15, 3)), np.zeros(3)
        nominal = state_vector(
            (pose_exp(rng.normal(size=9) * 0.4), rng.normal(size=6) * 0.01)
        ).astype(np.float32)
        observation = rng.normal(size=(3, 15)).astype(np.float32).astype(float)
        residual = (
            (rng.normal(size=3) * (3 if trial % 2 else 0.01))
            .astype(np.float32)
            .astype(float)
        )
        bias_mean = float(np.float32(rng.normal() * 0.1))
        gate = 1e-5 if trial % 7 == 6 else 0.0
        innovation = (
            observation @ covariance @ observation.T
            + observation @ state_noise
            + state_noise.T @ observation.T
            + noise
        )
        datum_gain = np.linalg.solve(innovation, observation @ cross + datum_noise)
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
                    joint_estimation=True,
                )
                mean, _, _, accepted, _ = reference(
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
                expected_mean = (
                    bias_mean + datum_gain @ residual if accepted else bias_mean
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
                        pressure=[variance, 0, 0, 0.01, 0, bias_mean],
                        geometry=geometry,
                        heading=heading,
                        root=root,
                        joint=True,
                    )
                    assert actual[5] == accepted and actual[4] == (1 if accepted else 3)
                    maximum["bias_mean"] = max(
                        maximum["bias_mean"],
                        abs(float(actual[6]) - expected_mean)
                        / (1 + abs(expected_mean)),
                    )
                    if accepted:
                        compare(actual, expected)
                        np.testing.assert_allclose(
                            actual[0], state_vector(mean), atol=3e-5, rtol=3e-5
                        )
                    else:
                        assert np.array_equal(actual[0], nominal)
                        assert np.array_equal(actual[1], covariance)
                        assert np.array_equal(actual[2], cross)
                        assert actual[6] == bias_mean and actual[7] == variance
                    counts["accepted" if accepted else "rejected"] += 1
                    counts["constrained"] += int(heading)

    assert maximum["joint_covariance"] < 2e-5, maximum
    batch_cases = []
    for root in (False, True):
        for gps_enabled in (False, True):
            nominal = np.r_[np.zeros(6), 1, np.zeros(9)].astype(np.float32)
            covariance, cross = np.eye(15), np.zeros(15)
            variance, bias_mean = 0.25, 0.0
            pressure_observation = np.zeros(16)
            pressure_observation[2] = pressure_observation[15] = 1
            gps_observation = np.c_[np.eye(3), np.zeros((3, 13))]
            information = np.diag(np.r_[np.ones(15), 4.0])
            information_mean = information @ np.r_[np.zeros(15), bias_mean]
            for packet in range(300):
                actual = kernel(
                    nominal,
                    covariance,
                    cross,
                    pressure=[variance, 0, 0, 0.01, 0, bias_mean],
                    root=root,
                    joint=True,
                    operation=1,
                )
                assert actual[5] and actual[4] == 1
                information += (
                    np.outer(pressure_observation, pressure_observation) / 0.01
                )
                nominal, covariance, cross = (
                    actual[0],
                    actual[1].astype(float),
                    actual[2].astype(float),
                )
                bias_mean, variance = float(actual[6]), float(actual[7])
                counts["pressure"] += 1
                if gps_enabled and packet % 7 == 0:
                    observation = gps_observation[:, :15]
                    actual = kernel(
                        nominal,
                        covariance,
                        cross,
                        residual=-nominal[:3],
                        observation=observation,
                        noise=0.04 * np.eye(3),
                        pressure=[variance, 0, 0, 0.01, 0, bias_mean],
                        root=root,
                        joint=True,
                    )
                    assert actual[5] and actual[4] == 1
                    information += gps_observation.T @ gps_observation / 0.04
                    nominal, covariance, cross = (
                        actual[0],
                        actual[1].astype(float),
                        actual[2].astype(float),
                    )
                    bias_mean, variance = float(actual[6]), float(actual[7])
                    counts["gps"] += 1
                expected = np.linalg.inv(information)
                compare(actual, expected)
                expected_mean = expected @ information_mean
                np.testing.assert_allclose(
                    nominal[:3], expected_mean[:3], atol=3e-5, rtol=3e-5
                )
                np.testing.assert_allclose(
                    bias_mean, expected_mean[15], atol=3e-5, rtol=3e-5
                )
            batch_cases.append(
                dict(
                    root=root,
                    gps=gps_enabled,
                    vertical_variance_m2=covariance[2, 2],
                    bias_variance_m2=variance,
                    bias_mean_m=bias_mean,
                )
            )
            if not gps_enabled:
                np.testing.assert_allclose(covariance[2, 2], 0.2, atol=3e-5)
    assert maximum["joint_covariance"] < 2e-5, maximum
    assert maximum["bias_mean"] < 1e-5, maximum
    result = dict(
        seed=args.seed,
        trials=args.trials,
        counts=counts,
        batch_cases=batch_cases,
        maximum_errors=maximum,
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        reference="Full augmented Gaussian Joseph correction and independent batch-information posterior; real Lie reset oracle; no covariance repair.",
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
