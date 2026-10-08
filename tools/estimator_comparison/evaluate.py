#!/usr/bin/env python3
"""Score a generated replay in memory, keeping durable metrics instead of raw CSVs."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np

from compare_delay import PROFILES
from consistency import consistency
from score import metrics, read, transition


def evaluate(args):
    if args.output.exists():
        raise ValueError("Use a new score filename to preserve previous evidence")
    if args.diagnostics and (args.gyro_bias is None or args.accel_bias is None):
        raise ValueError("Covariance scoring requires explicit truth IMU biases")
    if not np.isfinite(args.stationary_until) or not 0 <= args.stationary_until <= 60:
        raise ValueError("Stationary interval must end between 0 and 60 seconds")
    if args.imu_noise_density and not all(
        np.isfinite(value) and 1e-9 <= value <= 100 for value in args.imu_noise_density
    ):
        raise ValueError("IMU noise densities must be finite and between 1e-9 and 100")
    results = []
    input_hashes = {}
    for lower_rates in (False, True):
        profile = "25/25/25 Hz" if lower_rates else "100/50/50 Hz"
        for frequency in args.frequencies:
            for seed in args.seeds:
                tag = ("lower_" if lower_rates else "") + f"{frequency}_{seed}"
                data = args.captures / tag
                truth = read(data / "truth.csv")
                expected_times = np.loadtxt(
                    data / "modelica_input.csv", delimiter=",", skiprows=1, usecols=0
                )
                input_hashes[tag] = {
                    name: hashlib.sha256((data / name).read_bytes()).hexdigest()
                    for name in ("imu.csv", "modelica_input.csv", "truth.csv")
                }
                for case in args.scenarios:
                    command = [
                        str(args.replay.resolve()),
                        str(data / "imu.csv"),
                        str(data / "modelica_input.csv"),
                        "-",
                        case,
                        "foh",
                        *(["-"] if args.diagnostics else []),
                    ]
                    if args.stationary_until:
                        command += ["--stationary-until", str(args.stationary_until)]
                    if args.imu_noise_density:
                        command += [
                            "--imu-noise-density",
                            *map(str, args.imu_noise_density),
                        ]
                    if args.delay_profile:
                        command += [
                            "--delays",
                            *map(str, PROFILES[args.delay_profile]),
                            str(seed),
                        ]
                    completed = subprocess.run(
                        command, capture_output=True, text=True, check=True
                    )
                    estimate = read(io.StringIO(completed.stdout))
                    if len(estimate) != len(expected_times) or not np.allclose(
                        estimate["t_s"], expected_times, rtol=0, atol=5.0e-7
                    ):
                        raise ValueError(
                            f"Missing or mistimed replay rows: {tag} {case}"
                        )
                    finite = all(
                        np.isfinite(estimate[name]).all()
                        for name in estimate.dtype.names
                    )
                    if not finite or np.any(estimate["step_status"]):
                        raise ValueError(f"Invalid generated replay: {tag} {case}")
                    result = dict(
                        name="modelica",
                        profile=profile,
                        speed=frequency,
                        seed=seed,
                        case=case,
                        output_sha256=hashlib.sha256(
                            completed.stdout.encode()
                        ).hexdigest(),
                    )
                    for window, start, end in (
                        ("flight", 13, 60),
                        ("before_outage", 18, 25),
                        ("outage_window", 25, 40),
                        ("after_return", 40, 60),
                    ):
                        result[window] = metrics(estimate, truth, start, end)
                        if result[window]["finite_fraction"] != 1:
                            raise ValueError(f"Nonfinite scored error: {tag} {case}")
                    if case == "transition":
                        result["transition"] = transition(estimate, truth)
                    if args.diagnostics:
                        covariance = read(io.StringIO(completed.stderr))
                        result["consistency"] = consistency(
                            estimate,
                            covariance,
                            truth,
                            args.gyro_bias,
                            args.accel_bias,
                        )
                    elif completed.stderr:
                        raise ValueError(
                            f"Unexpected replay diagnostic: {completed.stderr}"
                        )
                    results.append(result)
        print(f"Scored {profile}", flush=True)
    evidence = dict(
        label=args.label,
        stationary_until_s=args.stationary_until,
        delay_profile=args.delay_profile,
        replay_sha256=hashlib.sha256(args.replay.read_bytes()).hexdigest(),
        input_sha256=input_hashes,
        scores=results,
    )
    if args.imu_noise_density:
        evidence["imu_noise_density"] = args.imu_noise_density
    args.output.write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")
    print(f"Saved {len(results)} finite generated-code scores.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("replay", "captures", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--label", default="ESKF")
    parser.add_argument("--frequencies", type=float, nargs="+", default=[0.25, 0.6])
    parser.add_argument("--seeds", type=int, nargs="+", default=[7, 19, 41])
    parser.add_argument("--diagnostics", action="store_true")
    parser.add_argument("--stationary-until", type=float, default=0)
    parser.add_argument(
        "--imu-noise-density", type=float, nargs=2, metavar=("GYRO", "ACCEL")
    )
    parser.add_argument("--delay-profile", choices=PROFILES)
    parser.add_argument(
        "--scenarios",
        nargs="+",
        choices=("gps", "denied", "transition"),
        default=["gps", "denied", "transition"],
    )
    parser.add_argument("--gyro-bias", type=float, nargs=3)
    parser.add_argument("--accel-bias", type=float, nargs=3)
    evaluate(parser.parse_args())
