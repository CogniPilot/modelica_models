"""Compare separately executed native coning probes with an attitude ODE."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np


def quaternion_product(left, right):
    return np.r_[
        left[0] * right[0] - left[1:] @ right[1:],
        left[0] * right[1:] + right[0] * left[1:] + np.cross(left[1:], right[1:]),
    ]


def reference_angle(duration, steps=2048):
    rate = np.array([0.3, -0.2, 0.1])
    acceleration = np.array([2.0, 3.0, -1.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    step = duration / steps

    def derivative(time, attitude):
        return 0.5 * quaternion_product(
            attitude, np.r_[0.0, rate + acceleration * time]
        )

    for interval in range(steps):
        time = interval * step
        first = derivative(time, quaternion)
        second = derivative(time + step / 2, quaternion + step / 2 * first)
        third = derivative(time + step / 2, quaternion + step / 2 * second)
        fourth = derivative(time + step, quaternion + step * third)
        quaternion += step / 6 * (first + 2 * second + 2 * third + fourth)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion[1:] * (
        2
        * np.arctan2(np.linalg.norm(quaternion[1:]), quaternion[0])
        / np.linalg.norm(quaternion[1:])
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--integrator", type=Path, required=True)
    parser.add_argument("--ardupilot-executable", type=Path)
    parser.add_argument("--ardupilot-provenance", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if bool(args.ardupilot_executable) != bool(args.ardupilot_provenance):
        parser.error("Supply both ArduPilot executable and provenance")
    rows = np.loadtxt(
        subprocess.check_output(
            [str(args.executable.resolve())], text=True
        ).splitlines(),
        delimiter=",",
    )
    if rows.shape != (12, 5) or not np.isfinite(rows).all():
        raise ValueError("Expected twelve finite PX4 probe rows with five fields")
    ardupilot = None
    provenance = None
    if args.ardupilot_executable:
        ardupilot = np.loadtxt(
            subprocess.check_output(
                [str(args.ardupilot_executable.resolve())], text=True
            ).splitlines(),
            delimiter=",",
        )
        if (
            ardupilot.shape != rows.shape
            or not np.isfinite(ardupilot).all()
            or not np.array_equal(ardupilot[:, :2], rows[:, :2])
        ):
            raise ValueError("Native coning probes use different sample schedules")
        provenance = json.loads(args.ardupilot_provenance.read_text())
        actual = hashlib.sha256(args.ardupilot_executable.read_bytes()).hexdigest()
        if not provenance["complete"] or provenance["executable_sha256"] != actual:
            raise ValueError("ArduPilot executable differs from its build provenance")
    cases = []
    for number, row in enumerate(rows):
        dt, intervals = row[:2]
        duration = dt * intervals
        reference = reference_angle(duration)
        refined = reference_angle(duration, steps=4096)
        difference = float(np.linalg.norm(refined - reference))
        assert difference < 1e-12
        px4_error = float(np.linalg.norm(row[2:5] - refined))
        case = dict(
            dt_s=float(dt),
            intervals=int(intervals),
            px4_angle_rad=row[2:5].tolist(),
            reference_angle_rad=refined.tolist(),
            px4_error_rad=px4_error,
            reference_refinement_difference_rad=difference,
        )
        if ardupilot is not None:
            angle = ardupilot[number, 2:5]
            error = float(np.linalg.norm(angle - refined))
            case.update(
                ardupilot_native_angle_rad=angle.tolist(),
                ardupilot_native_error_rad=error,
                ardupilot_native_closer=error < px4_error,
            )
        cases.append(case)
    result = dict(
        integrator_sha256=hashlib.sha256(args.integrator.read_bytes()).hexdigest(),
        executable_sha256=hashlib.sha256(args.executable.read_bytes()).hexdigest(),
        reference="Double-precision RK4 quaternion ODE for a linear angular-rate ramp",
        px4="Compiled supplied upstream IntegratorConing header with upstream matrix types",
        ardupilot=provenance,
        ardupilot_comparison_performed=ardupilot is not None,
        priming="One warm-up interval then packet reset, preserving previous increment",
        limitations=[
            "Finite linear-rate cases; no sensor noise, filtering, clipping, or timestamp jitter",
            "This probes sensor integration, not the complete EKF2/EKF3 estimators",
            "Published frozen EKF2 flight replays bypass this sensor integrator",
            "Does not establish a sculling indexing defect",
        ],
        cases=cases,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {
                "cases": len(cases),
                "ardupilot_comparison_performed": ardupilot is not None,
            }
        )
    )


if __name__ == "__main__":
    main()
