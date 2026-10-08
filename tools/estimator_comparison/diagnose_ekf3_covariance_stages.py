"""Locate an EKF3 covariance failure without repairing or rescoring its replay."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from native_consistency import common_state_and_jacobian


def scalar_covariances(prior, gain, observed_state, noise):
    observation = np.eye(len(prior))[observed_state]
    transform = np.eye(len(prior)) - np.outer(gain, observation)
    left = transform @ prior
    joseph = transform @ prior @ transform.T + noise * np.outer(gain, gain)
    return left, joseph


def matrix(row):
    return np.array(
        [row[f"p{i}_{j}"] for i in range(24) for j in range(24)], dtype=float
    ).reshape(24, 24)


def jacobian(row):
    state = np.array([row[f"s{i}"] for i in range(24)], dtype=float)
    navigation = np.r_[state[7:10], state[4:7], state[:4], state[10:16]]
    return common_state_and_jacobian(navigation, "ekf3", float(row["bias_period_s"]))[1]


def covariance_summary(covariance, transform):
    marginal = transform @ covariance @ transform.T
    asymmetry = float(np.max(np.abs(marginal - marginal.T)))
    symmetric = asymmetry <= 1e-12
    positive = False
    if symmetric:
        try:
            np.linalg.cholesky(marginal)
            positive = True
        except np.linalg.LinAlgError:
            pass
    return {
        "common_cholesky_valid": positive,
        "common_symmetric": symmetric,
        "common_max_asymmetry": asymmetry,
        "common_symmetric_part_min_eigenvalue": float(
            np.linalg.eigvalsh((marginal + marginal.T) / 2).min()
        ),
        "native_symmetric_part_min_eigenvalue": float(
            np.linalg.eigvalsh((covariance + covariance.T) / 2).min()
        ),
    }


def locate(row):
    return {
        "publication_s": float(row["publication_us"]) / 1e6,
        "fusion_s": float(row["fusion_us"]) / 1e6,
        "stage": str(row["stage"]),
        "axis": int(row["axis"]),
    }


def diagnose(path):
    rows = np.atleast_1d(
        np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    )
    expected = (
        "publication_us",
        "fusion_us",
        "bias_period_s",
        "stage",
        "axis",
        "state_index_limit",
        "gyro_inhibited",
        "accel_inhibited",
        "accel_x_inhibited",
        "accel_y_inhibited",
        "accel_z_inhibited",
        "bad_imu_data",
        "aiding_mode",
        "noise_variance",
        *(f"s{i}" for i in range(24)),
        *(f"k{i}" for i in range(24)),
        *(f"p{i}_{j}" for i in range(24) for j in range(24)),
    )
    if not len(rows) or rows.dtype.names != expected:
        raise ValueError("Require a complete native operation trace")
    if any(not np.isfinite(rows[key]).all() for key in expected if key != "stage"):
        raise ValueError("Nonfinite operation trace")
    if np.any(np.diff(rows["publication_us"].astype(float)) < 0):
        raise ValueError("Operation trace runs backwards")
    if np.any(rows["fusion_us"] > rows["publication_us"]):
        raise ValueError("Future fusion epoch in operation trace")
    summaries = [covariance_summary(matrix(row), jacobian(row)) for row in rows]
    invalid = [
        index
        for index, summary in enumerate(summaries)
        if summary["common_symmetric"] and not summary["common_cholesky_valid"]
    ]
    scalar_checks = []
    stages = (
        "scalar.before",
        "scalar.ForceSymmetry.before",
        "scalar.ForceSymmetry.after",
        "scalar.ConstrainVariances.before",
        "scalar.ConstrainVariances.after",
    )
    for index, row in enumerate(rows):
        if row["stage"] != "scalar.before":
            continue
        group = rows[index : index + len(stages)]
        if tuple(group["stage"]) != stages:
            raise ValueError("Incomplete or reordered scalar update snapshots")
        if any(
            not np.all(group[key] == row[key])
            for key in ("publication_us", "fusion_us", "axis", "state_index_limit")
        ):
            raise ValueError("Mixed scalar updates")
        axis, limit = int(row["axis"]), int(row["state_index_limit"])
        noise = float(row["noise_variance"])
        if not 0 <= axis <= 5 or not 9 <= limit < 24 or noise <= 0:
            raise ValueError("Invalid direct-state scalar observation")
        prior, transform = matrix(row), jacobian(row)
        gain = np.array([row[f"k{i}"] for i in range(24)], dtype=float)
        left, joseph = scalar_covariances(prior, gain, axis + 4, noise)
        expected_left = prior.copy()
        expected_left[: limit + 1, : limit + 1] = left[: limit + 1, : limit + 1]
        expected_symmetric = expected_left.copy()
        active = expected_left[: limit + 1, : limit + 1]
        expected_symmetric[: limit + 1, : limit + 1] = (active + active.T) / 2
        left_error = float(np.max(np.abs(matrix(group[1]) - expected_left)))
        symmetric_error = float(np.max(np.abs(matrix(group[2]) - expected_symmetric)))
        if left_error > 1e-11 or symmetric_error > 1e-11:
            raise ValueError(
                "Native covariance update differs from reconstructed operation"
            )
        optimal_gain = prior[:, axis + 4] / (prior[axis + 4, axis + 4] + noise)
        checks = {
            **locate(row),
            "observed_native_state": axis + 4,
            "noise_variance": noise,
            "prior": summaries[index],
            "after_left_update": summaries[index + 1],
            "after_force_symmetry": summaries[index + 2],
            "after_constrain_variances": summaries[index + 4],
            "counterfactual_joseph": covariance_summary(joseph, transform),
            "left_update_max_error": left_error,
            "force_symmetry_max_error": symmetric_error,
            "gain": gain.tolist(),
            "unconstrained_gain": optimal_gain.tolist(),
            "modified_gain_indices": np.flatnonzero(
                np.abs(gain - optimal_gain) > 1e-12
            ).tolist(),
            "inhibition": {key: bool(row[key]) for key in expected[6:11]},
            "bad_imu_data": bool(row["bad_imu_data"]),
            "aiding_mode": int(row["aiding_mode"]),
        }
        scalar_checks.append(checks)
    failures = [
        check
        for check in scalar_checks
        if check["prior"]["common_cholesky_valid"]
        and not check["after_force_symmetry"]["common_cholesky_valid"]
    ]
    first = invalid[0] if invalid else None
    return {
        "trace_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "trace_rows": len(rows),
        "publication_window_s": [
            float(rows[0]["publication_us"]) / 1e6,
            float(rows[-1]["publication_us"]) / 1e6,
        ],
        "first_symmetric_failure": (
            {**locate(rows[first]), **summaries[first]} if first is not None else None
        ),
        "preceding_snapshot": (
            {**locate(rows[first - 1]), **summaries[first - 1]}
            if first is not None and first > 0
            else None
        ),
        "scalar_updates_checked": len(scalar_checks),
        "scalar_positive_to_invalid_updates": failures,
        "scope": (
            "Read-only native stage diagnostics. Eigenvalues of symmetric parts "
            "are labeled explicitly; asymmetric snapshots are never treated as "
            "valid covariances. Joseph matrices are offline counterfactuals, "
            "not native replay repairs, NEES replacements, or rankings."
        ),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    args.output.write_text(
        json.dumps(diagnose(args.trace), indent=2, allow_nan=False) + "\n"
    )
