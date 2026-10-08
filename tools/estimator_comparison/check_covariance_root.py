#!/usr/bin/env python3
"""Validate a generated QR covariance root against an independent NumPy QR."""

import argparse
import ctypes
import json
from pathlib import Path

import numpy as np


def reference(columns):
    _, upper = np.linalg.qr(columns.astype(float).T, mode="reduced")
    return upper.T * np.where(np.diag(upper) < 0, -1, 1)


def check(args):
    library = ctypes.CDLL(str(args.library.resolve()))
    array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
    library.covariance_root.argtypes = [array, array]
    library.covariance_root.restype = ctypes.c_int
    rng = np.random.default_rng(args.seed)
    maxima = dict(
        normalized_covariance_error=0.0,
        normalized_root_error=0.0,
        worst_relative_nees_error=0.0,
    )
    cases = []

    def evaluate(label, columns, full_rank):
        columns = np.ascontiguousarray(columns, dtype=np.float32)
        output = np.empty((15, 15), dtype=np.float32)
        status = library.covariance_root(columns.ravel(), output.ravel())
        if status or not np.isfinite(output).all():
            raise ValueError(f"Generated covariance root failed: {label}, {status}")
        if np.any(np.triu(output, 1)) or np.any(np.diag(output) < 0):
            raise ValueError("Generated covariance root is not lower triangular")
        if full_rank and np.any(np.diag(output) <= 0):
            raise ValueError("Full-rank factor lost a positive pivot")
        expected = reference(columns)
        covariance = columns.astype(float) @ columns.astype(float).T
        scale = np.sqrt(np.diag(covariance))
        scale = np.where(scale > 0, scale, 1.0)
        reconstructed = output.astype(float) @ output.astype(float).T
        covariance_error = float(
            np.max(np.abs(reconstructed - covariance) / np.outer(scale, scale))
        )
        root_error = float(np.max(np.abs(output - expected) / scale[:, None]))
        worst_nees_error = None
        if full_rank:
            relative_whitening = np.linalg.solve(output.astype(float), expected)
            singular_values = np.linalg.svd(relative_whitening, compute_uv=False)
            worst_nees_error = float(np.max(np.abs(singular_values**2 - 1)))
            if not np.isfinite(worst_nees_error) or worst_nees_error > 1e-3:
                raise ValueError(f"Inaccurate covariance-root whitening: {label}")
            maxima["worst_relative_nees_error"] = max(
                maxima["worst_relative_nees_error"], worst_nees_error
            )
        if (
            not np.isfinite([covariance_error, root_error]).all()
            or covariance_error > 3e-5
            or root_error > 3e-5
        ):
            raise ValueError(f"Inaccurate covariance root: {label}, {covariance_error}")
        maxima["normalized_covariance_error"] = max(
            maxima["normalized_covariance_error"], covariance_error
        )
        maxima["normalized_root_error"] = max(
            maxima["normalized_root_error"], root_error
        )
        cases.append(
            dict(
                label=label,
                full_rank=full_rank,
                normalized_covariance_error=covariance_error,
                normalized_root_error=root_error,
                worst_relative_nees_error=worst_nees_error,
            )
        )
        return output

    evaluate("zero", np.zeros((15, 30)), False)
    evaluate("negative diagonal", np.c_[-np.eye(15), np.zeros((15, 15))], True)
    rank_one = np.zeros((15, 30))
    rank_one[:, 0] = rng.normal(size=15)
    evaluate("rank one", rank_one, False)
    for trial in range(args.trials):
        scales = 10.0 ** rng.uniform(-5, 2, 15)
        factor = rng.normal(size=(15, 30)) * scales[:, None]
        evaluate(f"anisotropic {trial}", factor, True)
    for exponent in (-25, 20):
        evaluate(f"scaled {exponent}", rng.normal(size=(15, 30)) * 10.0**exponent, True)

    captured = json.loads(args.case.read_text())
    prior = np.array(captured["prior"])
    reset = np.array(captured["reset_jacobian"])
    prior_root = np.linalg.cholesky(prior).astype(np.float32)
    columns = np.c_[reset @ prior_root, np.zeros((15, 15))].astype(np.float32)
    root = evaluate("captured failing reset", columns, True)
    error = rng.normal(size=15)
    whitened = np.linalg.solve(root.astype(float), error)
    reference_whitened = np.linalg.solve(reference(columns), error)
    nees = float(whitened @ whitened)
    reference_nees = float(reference_whitened @ reference_whitened)
    nees_error = abs(nees / reference_nees - 1)
    if not np.isfinite(nees) or nees_error > 1e-4:
        raise ValueError("Covariance-root whitening failed on captured reset")
    result = dict(
        seed=args.seed,
        cases=cases,
        maxima=maxima,
        captured_reset_nees_relative_error=nees_error,
        captured_generated_covariance_min_eigenvalue=captured[
            "generated_min_eigenvalue"
        ],
        no_added_noise=True,
        note="Generated root primitive only; the current ESKF still stores a full covariance. This is not an end-to-end square-root ESKF or a fixed replay failure.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(cases=len(cases), **maxima, nees_relative_error=nees_error)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument(
        "--case",
        type=Path,
        default=Path(__file__).parent / "fixtures/ill_conditioned_reset.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20271007)
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("At least one numerical trial is required")
    check(args)
