"""Check the complete float32 navigation/pressure covariance without repair."""

import argparse
import json
from pathlib import Path

import numpy as np

from manifest import digest
from score import read


def check(path, horizon, start_s, end_s):
    values = read(path)
    if ("fusion_t_s" in values.dtype.names) != horizon:
        raise ValueError("Choose the actual covariance state epoch; horizon flag differs")
    epoch = values["fusion_t_s" if horizon else "t_s"]
    values = values[(epoch >= start_s) & (epoch < end_s)]
    if not len(values):
        raise ValueError("No covariance rows in the requested window")
    covariance = np.empty((len(values), 16, 16))
    covariance[:, :15, :15] = np.column_stack(
        [values[f"p{row}_{column}"] for row in range(15) for column in range(15)]
    ).reshape(-1, 15, 15)
    cross = np.column_stack([values[f"barometer_cross_{axis}"] for axis in range(15)])
    covariance[:, :15, 15] = cross
    covariance[:, 15, :15] = cross
    covariance[:, 15, 15] = values["barometer_bias_variance_m2"]
    covariance = covariance.astype(np.float32).astype(np.float64)
    if not np.isfinite(covariance).all() or not np.array_equal(
        covariance, np.swapaxes(covariance, 1, 2)
    ):
        raise ValueError("Nonfinite or asymmetric augmented covariance")
    np.linalg.cholesky(covariance)
    cross = covariance[:, :15, 15]
    solved = np.linalg.solve(covariance[:, :15, :15], cross[:, :, None])[:, :, 0]
    schur = covariance[:, 15, 15] - np.sum(cross * solved, axis=1)
    if not np.all(schur > 0):
        raise ValueError("Nonpositive pressure conditional variance")
    return dict(
        covariance_sha256=digest(path),
        rows=len(values),
        start_s=start_s,
        end_s=end_s,
        horizon=horizon,
        minimum_datum_schur_variance_m2=float(schur.min()),
        scope="Round-trip-safe 9-digit CSV recovered to actual float32 values. Full 16D Cholesky and pressure Schur checks; no symmetrization, jitter or repair. State-output parity must be verified separately.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--horizon", action="store_true")
    parser.add_argument("--start-s", type=float, default=117)
    parser.add_argument("--end-s", type=float, default=166.7)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    args.output.write_text(
        json.dumps(
            check(args.input, args.horizon, args.start_s, args.end_s),
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
