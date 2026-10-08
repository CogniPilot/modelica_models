#!/usr/bin/env python3
"""Measure generated FOH/ZOH packet noise against the prediction covariance."""

import argparse
import ctypes
import json
from pathlib import Path

import numpy as np


def rotation(quaternion):
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_vector(quaternion):
    quaternion = quaternion / np.linalg.norm(quaternion)
    if quaternion[0] < 0:
        quaternion = -quaternion
    magnitude = np.linalg.norm(quaternion[1:])
    return quaternion[1:] * (
        2 * np.arctan2(magnitude, quaternion[0]) / magnitude
        if magnitude > 1e-12
        else 2.0
    )


def local_error(reference, sample):
    nominal_rotation = rotation(reference[6:])
    w, v = reference[6], -reference[7:]
    s, u = sample[6], sample[7:]
    relative = np.r_[w * s - v @ u, w * u + s * v + np.cross(v, u)]
    return np.r_[
        nominal_rotation.T @ (sample[:3] - reference[:3]),
        nominal_rotation.T @ (sample[3:6] - reference[3:6]),
        rotation_vector(relative),
    ]


def equivalent_inputs(packet, interval):
    angle = rotation_vector(packet[6:])
    magnitude = np.linalg.norm(angle)
    x, y, z = angle
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    coefficient = (
        (1 - 0.5 * magnitude / np.tan(0.5 * magnitude)) / magnitude**2
        if magnitude > 1e-5
        else 1 / 12 + magnitude**2 / 720
    )
    inverse_jacobian = np.eye(3) - 0.5 * cross + coefficient * cross @ cross
    return angle / interval, inverse_jacobian @ packet[3:6] / interval


def check(args):
    library = ctypes.CDLL(str(args.library.resolve()))
    array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
    library.integrate_packet.argtypes = [
        ctypes.c_int,
        ctypes.c_int,
        array,
        array,
        array,
    ]
    library.integrate_packet.restype = ctypes.c_int
    library.packet_covariance.argtypes = [
        array,
        array,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        array,
    ]
    library.packet_covariance.restype = ctypes.c_int
    rng = np.random.default_rng(args.seed)
    interval = 0.01
    sample_period = interval / 8
    densities = np.array([1e-4, 3e-2])
    results = []

    def integrate(mode, rates, forces):
        packet = np.empty(10, dtype=np.float32)
        if library.integrate_packet(
            mode,
            8,
            np.ascontiguousarray(rates, dtype=np.float32),
            np.ascontiguousarray(forces, dtype=np.float32),
            packet,
        ):
            raise ValueError("Generated preintegrator rejected a noise trial")
        return packet.astype(float)

    for name, rate, slope, force in [
        ("hover", [0, 0, 0], [0, 0, 0], [0, 0, 9.81]),
        ("turn", [0.7, -0.4, 0.2], [0, 0, 0], [0.5, -0.3, 9.81]),
        ("varying", [2, -1, 0.5], [20, -15, 10], [2, -1, 9.81]),
    ]:
        times = np.arange(17) * sample_period
        rates = np.asarray(rate) + times[:, None] * slope
        forces = np.tile(force, (17, 1))
        gyro_noise = rng.normal(size=(args.draws, 17, 3)) * np.sqrt(
            densities[0] / sample_period
        )
        accel_noise = rng.normal(size=(args.draws, 17, 3)) * np.sqrt(
            densities[1] / sample_period
        )
        for mode in (0, 1):
            nominal = [
                integrate(mode, rates[start : start + 9], forces[start : start + 9])
                for start in (0, 8)
            ]
            errors = np.empty((2, args.draws, 9))
            for draw in range(args.draws):
                for packet_index, start in enumerate((0, 8)):
                    packet = integrate(
                        mode,
                        rates[start : start + 9] + gyro_noise[draw, start : start + 9],
                        forces[start : start + 9]
                        + accel_noise[draw, start : start + 9],
                    )
                    errors[packet_index, draw] = local_error(
                        nominal[packet_index], packet
                    )
            rate_equivalent, force_equivalent = equivalent_inputs(nominal[0], interval)
            covariance = np.empty((15, 15), dtype=np.float32)
            if library.packet_covariance(
                np.asarray(rate_equivalent, dtype=np.float32),
                np.asarray(force_equivalent, dtype=np.float32),
                interval,
                *densities,
                covariance,
            ):
                raise ValueError("Generated covariance rejected the trial")
            prior = covariance[:9, :9].astype(float)
            whitened = np.linalg.solve(np.linalg.cholesky(prior), errors[0].T).T
            empirical = np.cov(errors[0], rowvar=False)
            lag = np.mean(
                (errors[0] - errors[0].mean(axis=0))
                * (errors[1] - errors[1].mean(axis=0)),
                axis=0,
            ) / np.diag(prior)
            result = dict(
                motion=name,
                integration="foh" if mode else "zoh",
                draws=args.draws,
                seed=args.seed,
                gyro_density=float(densities[0]),
                accelerometer_density=float(densities[1]),
                mean_nees_9d=float(np.mean(np.sum(whitened**2, axis=1))),
                covariance_ratio_diagonal=(
                    np.diag(empirical) / np.diag(prior)
                ).tolist(),
                whitened_covariance_eigenvalues=np.linalg.eigvalsh(
                    np.cov(whitened, rowvar=False)
                ).tolist(),
                lag_one_covariance_over_predicted_variance=lag.tolist(),
            )
            results.append(result)
            print(
                name, result["integration"], "NEES", result["mean_nees_9d"], flush=True
            )
    args.output.write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--draws", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=20271007)
    check(parser.parse_args())
