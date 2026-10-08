#!/usr/bin/env python3
"""Check generated stationary prediction and its correlated six-axis correction."""

import argparse
import ctypes
import json
from pathlib import Path

import numpy as np

from check_preintegration_noise import rotation
from check_semidirect_bias import pose_exp, rotation_jacobian, skew
from check_semidirect_correction import physical_state, reference, state_vector


def check(args):
    if args.trials < 1:
        raise ValueError("At least one numerical trial is required")
    library = ctypes.CDLL(str(args.library.resolve()))
    array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
    library.stationary_imu_update.argtypes = [array] * 6
    library.stationary_imu_update.restype = ctypes.c_int
    rng = np.random.default_rng(20271008)
    maxima = dict(prediction=0.0, mean=0.0, covariance=0.0, nis=0.0)
    for trial in range(args.trials):
        nominal = state_vector(
            (pose_exp(rng.normal(size=9)), rng.normal(size=6) * 0.01)
        ).astype(np.float32)
        source = rng.normal(size=(15, 15))
        covariance = (0.001 * (source @ source.T / 15 + np.eye(15))).astype(np.float32)
        dt = np.float32((0.00125, 0.01, 0.02)[trial % 3])
        gravity = np.array([0, 0, -9.81], dtype=np.float32)
        body_gravity = -rotation(nominal[6:10]).T @ gravity
        measured_rate = nominal[10:13] + rng.normal(size=3) * 0.001
        measured_force = body_gravity + nominal[13:16] + rng.normal(size=3) * 0.03
        anchors = rng.normal(size=6) * 0.1
        angle = (measured_rate - anchors[:3]) * dt
        magnitude = np.linalg.norm(angle)
        quaternion = np.r_[
            np.cos(magnitude / 2), 0.5 * np.sinc(magnitude / (2 * np.pi)) * angle
        ]
        packet = np.r_[
            quaternion,
            rotation_jacobian(angle) @ (measured_force - anchors[3:]) * dt,
            anchors,
            dt,
        ].astype(np.float32)
        noise = np.array(
            [np.eye(3) * scale for scale in (3e-9, 1e-6, 1e-8, 1e-5)], dtype=np.float32
        )
        output = np.empty(485, dtype=np.float32)
        status = library.stationary_imu_update(
            nominal, covariance.ravel(), packet, noise.ravel(), gravity, output
        )
        if status or output[484] != 1:
            raise ValueError(
                f"Generated stationary correction failed: {status}, {output[-3:]}"
            )
        predicted = nominal.astype(float)
        predicted[:3] += dt * predicted[3:6]
        transition = np.eye(15)
        transition[:3, 3:6] = dt * np.eye(3)
        predicted_covariance = transition @ covariance @ transition.T
        predicted_covariance[9:12, 9:12] += dt * noise[2]
        predicted_covariance[12:15, 12:15] += dt * noise[3]
        maxima["prediction"] = max(
            maxima["prediction"],
            np.max(np.abs(output[:16] - predicted)),
            np.max(np.abs(output[16:241].reshape(15, 15) - predicted_covariance)),
        )
        observation = np.zeros((6, 15))
        observation[:3, 9:12] = np.eye(3)
        observation[3:, 6:9] = skew(body_gravity)
        observation[3:, 12:15] = np.eye(3)
        measurement_noise = np.zeros((6, 6))
        measurement_noise[:3, :3] = noise[0] / dt + dt / 3 * noise[2]
        measurement_noise[3:, 3:] = noise[1] / dt + dt / 3 * noise[3]
        cross = np.zeros((15, 6))
        cross[9:12, :3] = -0.5 * dt * noise[2]
        cross[12:15, 3:] = -0.5 * dt * noise[3]
        residual = np.r_[
            measured_rate - predicted[10:13],
            measured_force - body_gravity - predicted[13:16],
        ]
        mean, posterior, nis, accepted, _ = reference(
            physical_state(predicted),
            predicted_covariance,
            residual,
            observation,
            measurement_noise,
            cross,
            np.zeros(3),
            0,
            False,
            False,
        )
        if not accepted:
            raise ValueError("Independent stationary correction rejected")
        maxima["mean"] = max(
            maxima["mean"], np.max(np.abs(output[241:257] - state_vector(mean)))
        )
        maxima["covariance"] = max(
            maxima["covariance"],
            np.max(np.abs(output[257:482].reshape(15, 15) - posterior)),
        )
        maxima["nis"] = max(maxima["nis"], abs(output[482] - nis))
        np.linalg.cholesky(output[257:482].reshape(15, 15).astype(float))
    if (
        maxima["prediction"] > 2e-6
        or maxima["mean"] > 2e-5
        or maxima["covariance"] > 2e-6
        or maxima["nis"] > 2e-3
    ):
        raise ValueError(
            f"Generated stationary kernel differs from reference: {maxima}"
        )
    result = dict(
        trials=args.trials,
        seed=20271008,
        max_absolute_error={k: float(v) for k, v in maxima.items()},
        scope="Held-signal packets with arbitrary anchors; independent stationary process and correlated Gaussian update. This does not establish exact FOH packet noise or flight performance.",
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=120)
    check(parser.parse_args())
