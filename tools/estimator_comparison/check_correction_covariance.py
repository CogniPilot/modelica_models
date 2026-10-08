"""Challenge generated corrections with invalid joint error/noise covariances."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np


def check(libraries, output, allow_unsafe=False):
    nominal = np.zeros(16, dtype=np.float32)
    nominal[6] = 1
    covariance = np.eye(15, dtype=np.float32)
    observation = np.zeros((3, 15), dtype=np.float32)
    observation[:, :3] = np.eye(3)
    cases = []
    for name in (
        "indefinite_noise",
        "impossible_cross_covariance",
        "valid_correlated",
        "ill_scaled_valid_correlated",
    ):
        case_covariance = covariance.copy()
        noise = np.eye(3, dtype=np.float32)
        cross = np.zeros((15, 3), dtype=np.float32)
        if name == "indefinite_noise":
            noise[0, 1] = noise[1, 0] = 1.5
        elif name == "impossible_cross_covariance":
            cross[0, 0] = 2
        elif name == "valid_correlated":
            cross[0, 0] = 0.75
        else:
            case_covariance = np.diag(np.geomspace(1e-8, 100, 15)).astype(np.float32)
            cross[0, 0] = 0.1 * np.sqrt(case_covariance[0, 0])
        joint = np.block([[case_covariance, cross], [cross.T, noise]])
        innovation = observation @ case_covariance @ observation.T
        innovation += observation @ cross + cross.T @ observation.T + noise
        cases.append(
            (
                name,
                case_covariance,
                noise,
                cross,
                float(np.linalg.eigvalsh(joint)[0]),
                float(np.linalg.eigvalsh(innovation)[0]),
            )
        )
    results = []
    for label, path in libraries.items():
        function = ctypes.CDLL(str(path.resolve())).semi_direct_update
        array = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        function.argtypes = [array, array, array, ctypes.c_int, array]
        function.restype = ctypes.c_int
        for (
            name,
            case_covariance,
            noise,
            cross,
            joint_minimum,
            innovation_minimum,
        ) in cases:
            packed = np.r_[
                np.zeros(3),
                observation.ravel(),
                noise.ravel(),
                cross.ravel(),
                np.zeros(4),
            ].astype(np.float32)
            actual = np.empty(244, dtype=np.float32)
            status = function(nominal, case_covariance.ravel(), packed, 0, actual)
            if status or not np.isfinite(actual).all():
                raise ValueError(f"Invalid generated result: {label}/{name}")
            accepted = bool(actual[243])
            valid = joint_minimum > 0
            preserved = np.array_equal(actual[:16], nominal) and np.array_equal(
                actual[16:241].reshape(15, 15), case_covariance
            )
            result = dict(
                representation=label,
                case=name,
                accepted=accepted,
                expected_accepted=valid,
                reason=int(actual[242]),
                prior_preserved=preserved,
                minimum_joint_eigenvalue=joint_minimum,
                minimum_innovation_eigenvalue=innovation_minimum,
                minimum_posterior_eigenvalue=float(
                    np.linalg.eigvalsh(actual[16:241].reshape(15, 15).astype(float))[0]
                ),
                library_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            )
            result["passed"] = (
                accepted == valid
                and result["minimum_posterior_eigenvalue"] > 0
                and (valid or preserved)
                and result["reason"] == (1 if valid else 5 if label == "dense" else 4)
            )
            results.append(result)
    report = dict(
        cases=results,
        passed=all(row["passed"] for row in results),
        note="Every innovation is positive definite. Invalid cases violate the joint covariance requirement despite positive measurement diagonals.",
    )
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0 if report["passed"] or allow_unsafe else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dense", type=Path, required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-unsafe", action="store_true")
    args = parser.parse_args()
    raise SystemExit(
        check(dict(dense=args.dense, root=args.root), args.output, args.allow_unsafe)
    )
