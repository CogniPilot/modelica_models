"""Check the generated correction kernel against an independent Gaussian update."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np

from check_preintegration_noise import rotation
from check_semidirect_bias import (
    jacobian,
    local_error,
    pose_algebra,
    pose_exp,
    pose_product,
    reset_jacobian,
    retract,
    rotation_log,
)


def state_vector(state):
    pose, bias = state
    angle = rotation_log(pose[0])
    magnitude = np.linalg.norm(angle)
    quaternion = np.r_[
        np.cos(magnitude / 2), 0.5 * np.sinc(magnitude / (2 * np.pi)) * angle
    ]
    return np.r_[pose[1], pose[2], quaternion, bias[3:], bias[:3]]


def physical_state(vector):
    return (rotation(vector[6:10]), vector[:3], vector[3:6]), np.r_[
        vector[13:16], vector[10:13]
    ]


def reference(
    prior,
    covariance,
    residual,
    observation,
    noise,
    cross,
    axis,
    gate,
    heading,
    geometry,
):
    innovation = (
        observation @ covariance @ observation.T
        + observation @ cross
        + cross.T @ observation.T
        + noise
    )
    nis = residual @ np.linalg.solve(innovation, residual)
    if gate > 0 and nis > 3 * gate:
        return prior, covariance, nis, False, False
    gain = np.linalg.solve(innovation, (covariance @ observation.T + cross).T).T
    projector = np.outer(axis, axis) if axis @ axis > 0.5 else np.eye(3)
    gain[6:9] = projector @ gain[6:9]
    if heading:
        gain[:6] = 0
        gain[9:12] = projector @ gain[9:12]
        gain[12:15] = 0
    correction = gain @ residual
    angle = np.linalg.norm(correction[6:9])
    limited = angle > 0.15
    if limited:
        gain[6:9] *= 0.15 / angle
        correction = gain @ residual
    factor = np.eye(15) - gain @ observation
    posterior = (
        factor @ covariance @ factor.T
        + gain @ noise @ gain.T
        - factor @ cross @ gain.T
        - gain @ cross.T @ factor.T
    )
    order = np.r_[np.arange(9), np.arange(12, 15), np.arange(9, 12)]
    tangent = correction[order]
    if geometry:
        mean = retract(prior, tangent)
        reset = reset_jacobian(tangent)[np.ix_(order, order)]
    else:
        mean = pose_product(prior[0], pose_exp(tangent[:9])), prior[1] + tangent[9:]
        reset = np.eye(15)
        reset[:9, :9] = jacobian(pose_algebra(tangent[:9]), -1)
    return mean, reset @ posterior @ reset.T, nis, True, limited


def check(args):
    if args.trials < 1:
        raise ValueError("At least one trial is required")
    library = ctypes.CDLL(str(args.library.resolve()))
    array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
    library.semi_direct_update.argtypes = [array, array, array, ctypes.c_int, array]
    library.semi_direct_update.restype = ctypes.c_int
    rng = np.random.default_rng(args.seed)
    maxima = dict(mean=0.0, covariance=0.0, nis=0.0)
    counts = dict(accepted=0, rejected=0, limited=0, correlated=0, heading=0)
    for trial in range(args.trials):
        nominal = state_vector(
            (pose_exp(rng.normal(size=9)), rng.normal(size=6) * 0.1)
        ).astype(np.float32)
        prior = physical_state(nominal.astype(float))
        source = rng.normal(size=(18, 18))
        joint = source @ source.T / 18 + 0.05 * np.eye(18)
        covariance = joint[:15, :15].astype(np.float32)
        noise = joint[15:, 15:].astype(np.float32)
        cross = (
            joint[:15, 15:].astype(np.float32)
            if trial % 3
            else np.zeros((15, 3), dtype=np.float32)
        )
        observation = rng.normal(size=(3, 15)).astype(np.float32)
        residual = (rng.normal(size=3) * (10 if trial % 4 else 0.1)).astype(np.float32)
        gate = 0.25 if trial % 8 == 7 else 0.0
        for heading, projected in ((False, False), (False, True), (True, True)):
            axis = (
                prior[0][0][2].astype(np.float32)
                if projected
                else np.zeros(3, dtype=np.float32)
            )
            packed = np.r_[
                residual, observation.ravel(), noise.ravel(), cross.ravel(), axis, gate
            ].astype(np.float32)
            for geometry in (False, True):
                output = np.empty(244, dtype=np.float32)
                status = library.semi_direct_update(
                    nominal,
                    covariance,
                    packed,
                    int(geometry) | (int(heading) << 1),
                    output,
                )
                if status:
                    raise ValueError(
                        f"Generated correction error {status} at trial {trial}"
                    )
                mean, posterior, nis, accepted, limited = reference(
                    prior,
                    covariance.astype(float),
                    residual.astype(float),
                    observation.astype(float),
                    noise.astype(float),
                    cross.astype(float),
                    axis.astype(float),
                    gate,
                    heading,
                    geometry,
                )
                assert bool(output[243]) == accepted, (trial, geometry, heading)
                assert output[242] == (1 if accepted else 3), (trial, output[242])
                if not accepted:
                    assert np.array_equal(output[:16], nominal)
                    assert np.array_equal(output[16:241].reshape(15, 15), covariance)
                actual_covariance = output[16:241].reshape(15, 15).astype(float)
                np.linalg.cholesky(actual_covariance)
                assert np.max(np.abs(actual_covariance - actual_covariance.T)) < 1e-6
                maxima["mean"] = max(
                    maxima["mean"],
                    float(
                        np.max(
                            np.abs(
                                local_error(
                                    mean, physical_state(output[:16].astype(float))
                                )
                            )
                        )
                    ),
                )
                maxima["covariance"] = max(
                    maxima["covariance"],
                    float(np.max(np.abs(posterior - actual_covariance)))
                    / (1 + float(np.max(np.abs(posterior)))),
                )
                maxima["nis"] = max(
                    maxima["nis"], abs(float(output[241]) - nis) / (1 + nis)
                )
                counts["accepted" if accepted else "rejected"] += 1
                counts["limited"] += int(limited)
                counts["correlated"] += int(np.any(cross))
                counts["heading"] += int(heading)
    assert maxima["mean"] < 1e-4, maxima
    assert maxima["covariance"] < 1e-4, maxima
    assert maxima["nis"] < 1e-5, maxima
    assert all(counts.values()), counts
    result = dict(
        seed=args.seed,
        trials=args.trials,
        configurations=6,
        counts=counts,
        maximum_errors=maxima,
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        note="Independent float64 Gaussian solve and reset, evaluated against float32 deployment C; includes both correction geometries, projected/heading gain, correlated noise, trust limits and exact rejection preservation.",
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261007)
    parser.add_argument("--trials", type=int, default=120)
    check(parser.parse_args())
