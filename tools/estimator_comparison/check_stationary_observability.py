#!/usr/bin/env python3
"""Independently exhibit the stationary magnetic-axis/accelerometer-bias ambiguity."""

import argparse
import json
from pathlib import Path

import numpy as np

from check_semidirect_bias import rotation_exp, skew


def check(args):
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    rng = np.random.default_rng(20271007)
    field = np.array([-1.59e-6, 20.04e-6, -47.91e-6])
    axis_world = field / np.linalg.norm(field)
    gravity = np.array([0.0, 0.0, -9.81])
    maxima = dict(
        acceleration=0.0,
        magnetic_vector=0.0,
        dynamics_nullspace=0.0,
        observation_nullspace=0.0,
    )
    cases = 0
    for _ in range(24):
        orientation = rotation_exp(rng.normal(size=3))
        bias = rng.normal(size=3) * 0.1
        force = -orientation.T @ gravity
        measured_force = force + bias
        axis_body = orientation.T @ axis_world
        direction = np.zeros(15)
        direction[6:9] = axis_body
        direction[12:15] = -skew(force) @ axis_body
        dynamics = np.zeros((15, 15))
        dynamics[:3, 3:6] = np.eye(3)
        dynamics[3:6, 6:9] = -skew(force)
        dynamics[3:6, 12:15] = -np.eye(3)
        dynamics[6:9, 9:12] = -np.eye(3)
        observation = np.zeros((12, 15))
        observation[:3, :3] = np.eye(3)
        observation[3:6, 3:6] = np.eye(3)
        observation[6:9, 6:9] = skew(axis_body)
        observation[9:, 6:9] = skew(force)
        observation[9:, 12:15] = np.eye(3)
        maxima["dynamics_nullspace"] = max(
            maxima["dynamics_nullspace"], float(np.max(np.abs(dynamics @ direction)))
        )
        maxima["observation_nullspace"] = max(
            maxima["observation_nullspace"],
            float(np.max(np.abs(observation @ direction))),
        )
        for angle in (-0.7, -0.2, 0.2, 0.7):
            alternative = rotation_exp(angle * axis_world) @ orientation
            alternative_bias = measured_force + alternative.T @ gravity
            maxima["acceleration"] = max(
                maxima["acceleration"],
                float(
                    np.linalg.norm(
                        alternative @ (measured_force - alternative_bias) + gravity
                    )
                ),
            )
            maxima["magnetic_vector"] = max(
                maxima["magnetic_vector"],
                float(np.linalg.norm(alternative.T @ field - orientation.T @ field)),
            )
            cases += 1
    if (
        maxima["acceleration"] > 1e-12
        or maxima["magnetic_vector"] > 1e-18
        or max(maxima[k] for k in ("dynamics_nullspace", "observation_nullspace"))
        > 1e-12
    ):
        raise ValueError("Stationary equivalence or tangent nullspace check failed")
    args.output.write_text(
        json.dumps(
            dict(
                seed=20271007,
                finite_cases=cases,
                orientations=24,
                coordinates="position, velocity, local right attitude, gyro bias, accelerometer bias",
                transformation="R' = Exp(alpha * unit(field_world)) R; ba' = measured_force + R'^T gravity",
                tangent="N_theta = R^T unit(field_world); N_ba = -skew(-R^T gravity) N_theta; other blocks zero",
                maxima=maxima,
                scope="Ideal stationary IMU, known constant magnetic field, position/velocity and gravity observations with unknown constant accelerometer bias. Zero-velocity optical flow and barometric height add no information in this direction. No explicit slant-range/terrain attitude constraint is modeled.",
                interpretation="Finite numerical examples verify an independently constructed equivalence and tangent nullspace, not the current filter's consistency or the root cause of seed 41. Maneuvering can break the ambiguity because the alternative bias would need to vary.",
            ),
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    check(parser.parse_args())
