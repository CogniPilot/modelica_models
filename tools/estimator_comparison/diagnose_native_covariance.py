"""Locate covariance failures without changing matrices or discarding epochs."""

import argparse
import json
from pathlib import Path

import numpy as np

from compare_exposure import digest
from native_consistency import common_state_and_jacobian


def diagnose(args):
    if args.output.exists():
        raise ValueError("Choose a fresh evidence path")
    raw = np.atleast_1d(np.genfromtxt(args.covariance, delimiter=",", names=True))
    expected = (
        "publication_us",
        "fusion_us",
        "bias_period_s",
        *(f"s{component}" for component in range(16)),
        *(f"p{row}_{column}" for row in range(24) for column in range(24)),
    )
    if (
        raw.dtype.names != expected
        or not len(raw)
        or any(not np.isfinite(raw[key]).all() for key in expected)
    ):
        raise ValueError("Require a complete finite native state/covariance trace")
    offset = 1e6 if args.filter == "px4" else 0
    scored = (raw["fusion_us"] >= offset + 10e6) & (raw["fusion_us"] < offset + 59.7e6)
    selected = raw[scored]
    if not len(selected):
        raise ValueError("Empty declared covariance window")
    if np.any(selected["fusion_us"] > selected["publication_us"]) or np.any(
        np.diff(selected["fusion_us"]) < 0
    ):
        raise ValueError("Invalid native fusion epochs inside the declared window")
    failures = []
    first = None
    for index, row in enumerate(selected):
        state = np.array([row[f"s{component}"] for component in range(16)])
        native = np.array([[row[f"p{r}_{c}"] for c in range(24)] for r in range(24)])
        if not np.isfinite(native).all():
            raise ValueError("Nonfinite native covariance")
        _, jacobian = common_state_and_jacobian(
            state, args.filter, row["bias_period_s"]
        )
        marginal = jacobian @ native @ jacobian.T
        symmetric = 0.5 * (marginal + marginal.T)
        try:
            np.linalg.cholesky(symmetric)
        except np.linalg.LinAlgError:
            eigenvalues = np.linalg.eigvalsh(symmetric)
            failures.append(float(eigenvalues[0]))
            if first is None:
                native_eigenvalues = np.linalg.eigvalsh(0.5 * (native + native.T))
                first = dict(
                    scored_row_index=index,
                    publication_s=float((row["publication_us"] - offset) / 1e6),
                    fusion_s=float((row["fusion_us"] - offset) / 1e6),
                    bias_period_s=float(row["bias_period_s"]),
                    marginal_eigenvalue_range=[
                        float(eigenvalues[0]),
                        float(eigenvalues[-1]),
                    ],
                    marginal_min_diagonal=float(np.min(np.diag(marginal))),
                    marginal_max_asymmetry=float(np.max(np.abs(marginal - marginal.T))),
                    native_eigenvalue_range=[
                        float(native_eigenvalues[0]),
                        float(native_eigenvalues[-1]),
                    ],
                    native_min_diagonal=float(np.min(np.diag(native))),
                    native_max_asymmetry=float(np.max(np.abs(native - native.T))),
                )
                diagonal = np.diag(symmetric)
                if np.all(diagonal > 0):
                    scale = np.sqrt(diagonal)
                    correlation = symmetric / (scale[:, None] * scale[None, :])
                    values, vectors = np.linalg.eigh(correlation)
                    first["standardized_marginal_eigenvalue_range"] = [
                        float(values[0]),
                        float(values[-1]),
                    ]
                    first["standardized_negative_mode_squared_components"] = dict(
                        zip(
                            (
                                "position_x",
                                "position_y",
                                "position_z",
                                "velocity_x",
                                "velocity_y",
                                "velocity_z",
                                "attitude_x",
                                "attitude_y",
                                "attitude_z",
                                "gyro_bias_x",
                                "gyro_bias_y",
                                "gyro_bias_z",
                                "accel_bias_x",
                                "accel_bias_y",
                                "accel_bias_z",
                            ),
                            (vectors[:, 0] ** 2).tolist(),
                            strict=True,
                        )
                    )
    result = dict(
        filter=args.filter,
        covariance_sha256=digest(args.covariance),
        scored_rows=len(selected),
        unscored_rows=len(raw) - len(selected),
        unscored_future_fusion_rows=int(
            np.count_nonzero((raw["fusion_us"] > raw["publication_us"]) & ~scored)
        ),
        cholesky_failed_rows=len(failures),
        first_failure=first,
        minimum_failed_marginal_eigenvalue=min(failures) if failures else None,
        scope="Diagnostic only: native 24D and common 15D eigenvalues have mixed physical units. No regularization, covariance repair, epoch removal or NEES replacement.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filter", choices=("px4", "ekf3"))
    for name in ("covariance", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    diagnose(parser.parse_args())
