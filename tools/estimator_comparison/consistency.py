#!/usr/bin/env python3
"""Score ESKF covariance against independent pose truth and declared IMU biases."""

import argparse
import json
from pathlib import Path

import numpy as np

from check_preintegration_noise import local_error
from score import POSITION, QUATERNION, VELOCITY, read


def fusion_horizon_pair(estimate, diagnostics):
    if not np.array_equal(estimate["t_s"], diagnostics["t_s"]):
        raise ValueError("Estimate and covariance publication timestamps differ")
    if not {"fusion_t_s", "fusion_ready"}.issubset(diagnostics.dtype.names):
        raise ValueError("Missing covariance fusion epoch or readiness")
    if not {
        "fusion_t_s",
        *("horizon_" + k for k in POSITION + VELOCITY + QUATERNION),
    }.issubset(estimate.dtype.names):
        raise ValueError("Missing state at the covariance fusion epoch")
    if not np.array_equal(estimate["fusion_t_s"], diagnostics["fusion_t_s"]):
        raise ValueError("State and covariance fusion timestamps differ")
    if (
        not np.isfinite(diagnostics["fusion_t_s"]).all()
        or not np.isin(diagnostics["fusion_ready"], (0, 1)).all()
        or np.any(diagnostics["fusion_t_s"] > diagnostics["t_s"] + 1e-6)
    ):
        raise ValueError("Invalid covariance fusion epoch or readiness")
    selected = (diagnostics["fusion_ready"] > 0.5) & (diagnostics["fusion_t_s"] >= 0)
    state, covariance = estimate[selected].copy(), diagnostics[selected].copy()
    state["t_s"] = state["fusion_t_s"]
    covariance["t_s"] = covariance["fusion_t_s"]
    if not len(state) or not np.all(np.diff(state["t_s"]) > 0):
        raise ValueError("Fusion epochs must be nonempty and strictly increasing")
    for field in POSITION + VELOCITY + QUATERNION:
        state[field] = state["horizon_" + field]
    return state, covariance


def consistency(
    estimate,
    diagnostics,
    truth,
    gyro_bias,
    accel_bias,
    start=13,
    end=60,
    fusion_horizon=False,
):
    if fusion_horizon:
        estimate, diagnostics = fusion_horizon_pair(estimate, diagnostics)
    elif "fusion_t_s" in diagnostics.dtype.names:
        raise ValueError("Delayed covariance requires explicit fusion-horizon scoring")
    if not np.array_equal(estimate["t_s"], diagnostics["t_s"]):
        raise ValueError("Estimate and covariance timestamps differ")
    if estimate["t_s"][0] < truth["t_s"][0] or estimate["t_s"][-1] > truth["t_s"][-1]:
        raise ValueError("Truth extrapolation is forbidden")
    selected = (estimate["t_s"] >= start) & (estimate["t_s"] < end)
    estimate, diagnostics = estimate[selected], diagnostics[selected]
    if not len(estimate):
        raise ValueError("Empty consistency window")
    names = POSITION + VELOCITY + QUATERNION
    states = np.column_stack([estimate[name] for name in names])
    aligned = np.column_stack(
        [np.interp(estimate["t_s"], truth["t_s"], truth[name]) for name in names]
    )
    pose_error = np.array(
        [local_error(state, target) for state, target in zip(states, aligned)]
    )
    bias = np.column_stack(
        [diagnostics[name] for name in ("bgx", "bgy", "bgz", "bax", "bay", "baz")]
    )
    error = np.column_stack((pose_error, np.r_[gyro_bias, accel_bias] - bias))
    covariance = np.column_stack(
        [diagnostics[f"p{row}_{column}"] for row in range(15) for column in range(15)]
    ).reshape(-1, 15, 15)
    if not np.isfinite(error).all() or not np.isfinite(covariance).all():
        raise ValueError("Nonfinite state error or covariance")
    asymmetry = np.max(np.abs(covariance - covariance.transpose(0, 2, 1)))
    symmetric = 0.5 * (covariance + covariance.transpose(0, 2, 1))
    root_names = [f"l{row}_{column}" for row in range(15) for column in range(15)]
    root_present = set(root_names).intersection(diagnostics.dtype.names)
    root_checks = {}
    if root_present:
        if root_present != set(root_names):
            raise ValueError("Incomplete covariance-root diagnostic")
        root = np.column_stack([diagnostics[name] for name in root_names]).reshape(
            -1, 15, 15
        )
        if (
            not np.isfinite(root).all()
            or np.any(np.triu(root, 1))
            or np.any(np.diagonal(root, axis1=1, axis2=2) <= 0)
        ):
            raise ValueError("Invalid lower covariance root")
        reconstructed = root @ root.transpose(0, 2, 1)
        scale = np.sqrt(np.diagonal(reconstructed, axis1=1, axis2=2))
        reconstruction_error = np.max(
            np.abs(reconstructed - covariance) / (scale[:, :, None] * scale[:, None, :])
        )
        if reconstruction_error > 1e-5:
            raise ValueError("Covariance root and dense diagnostic differ")
        dense_failures = 0
        for matrix in symmetric:
            try:
                np.linalg.cholesky(matrix)
            except np.linalg.LinAlgError:
                dense_failures += 1
        root_checks = dict(
            covariance_representation="square_root",
            dense_covariance_non_pd_samples=dense_failures,
            max_covariance_root_reconstruction_error=float(reconstruction_error),
        )
    else:
        root = np.linalg.cholesky(symmetric)
    whitened = np.linalg.solve(root, error[:, :, None])[:, :, 0]
    nees = np.sum(whitened**2, axis=1)
    return dict(
        start_s=start,
        end_s=end,
        rows=len(estimate),
        mean_nees_15d=float(np.mean(nees)),
        median_nees_15d=float(np.median(nees)),
        p95_nees_15d=float(np.percentile(nees, 95)),
        max_covariance_asymmetry=float(asymmetry),
        mean_squared_error_over_marginal_variance=np.mean(
            error**2 / np.diagonal(symmetric, axis1=1, axis2=2), axis=0
        ).tolist(),
        terminal_bias_error=error[-1, 9:].tolist(),
        gyroscope_bias_truth=list(gyro_bias),
        accelerometer_bias_truth=list(accel_bias),
        **root_checks,
        note="Local first-order right-error coordinates. Flight samples are correlated; these statistics are descriptive, not independent Monte Carlo draws. Root diagnostics, when present, are authoritative; dense rounding failures are counted separately.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("estimate", "covariance", "truth", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--gyro-bias", type=float, nargs=3, required=True)
    parser.add_argument("--accel-bias", type=float, nargs=3, required=True)
    parser.add_argument("--fusion-horizon", action="store_true")
    args = parser.parse_args()
    result = consistency(
        read(args.estimate),
        read(args.covariance),
        read(args.truth),
        args.gyro_bias,
        args.accel_bias,
        fusion_horizon=args.fusion_horizon,
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
