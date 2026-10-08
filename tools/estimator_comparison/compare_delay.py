#!/usr/bin/env python3
"""Compare ESKF retrodiction and a buffered horizon on identical delayed packets."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import platform
import subprocess

import numpy as np

from score import POSITION, QUATERNION, VELOCITY, metrics, read, transition


PROFILES = {
    "zero": [0, 0, 0, 0, 0],
    "nominal": [110, 30, 20, 20, 10],
    "stress": [160, 60, 50, 70, 20],
    "exposure": [110, 30, 60, 20, 0],
}


def horizon_state(estimate):
    result = estimate.copy()
    result["t_s"] = estimate["fusion_t_s"]
    for field in (*POSITION, *VELOCITY, *QUATERNION):
        result[field] = estimate["horizon_" + field]
    return result[result["t_s"] >= 0]


def compare(args):
    if args.output.exists():
        raise ValueError("Use a new output filename to preserve previous evidence")
    if args.repetitions < 1:
        raise ValueError("At least one timing repetition is required")
    if not np.isfinite(args.stationary_until) or not 0 <= args.stationary_until <= 60:
        raise ValueError("Stationary interval must end between 0 and 60 seconds")
    if args.imu_noise_density and not all(
        np.isfinite(value) and 1e-9 <= value <= 100 for value in args.imu_noise_density
    ):
        raise ValueError("IMU noise densities must be finite and between 1e-9 and 100")
    records, capture_hashes = [], {}
    for lower_rates in (False, True):
        for frequency in args.frequencies:
            for seed in args.seeds:
                tag = ("lower_" if lower_rates else "") + f"{frequency}_{seed}"
                data = args.captures / tag
                truth = read(data / "truth.csv")
                expected_times = np.loadtxt(
                    data / "modelica_input.csv", delimiter=",", skiprows=1, usecols=0
                )
                capture_hashes[tag] = {
                    name: hashlib.sha256((data / name).read_bytes()).hexdigest()
                    for name in ("imu.csv", "modelica_input.csv", "truth.csv")
                }
                for delay_profile in args.delay_profiles:
                    delays = PROFILES[delay_profile]
                    for scenario in ("gps", "denied", "transition"):
                        shared_arrivals = None
                        for name, executable in (
                            ("retrodiction", args.retrodiction),
                            ("horizon", args.horizon),
                        ):
                            output_digest, timings = None, []
                            for _ in range(args.repetitions):
                                command = [
                                    str(executable.resolve()),
                                    str(data / "imu.csv"),
                                    str(data / "modelica_input.csv"),
                                    "-",
                                    scenario,
                                    "foh",
                                    "--delays",
                                    *map(str, delays),
                                    str(seed),
                                    "--timing",
                                    "-",
                                ]
                                if args.stationary_until:
                                    command += [
                                        "--stationary-until",
                                        str(args.stationary_until),
                                    ]
                                if args.imu_noise_density:
                                    command += [
                                        "--imu-noise-density",
                                        *map(str, args.imu_noise_density),
                                    ]
                                completed = subprocess.run(
                                    command, capture_output=True, text=True, check=True
                                )
                                digest = hashlib.sha256(
                                    completed.stdout.encode()
                                ).hexdigest()
                                if (
                                    output_digest is not None
                                    and digest != output_digest
                                ):
                                    raise ValueError(
                                        "Replay outputs changed between timing repetitions"
                                    )
                                output_digest = digest
                                timings.append(json.loads(completed.stderr))
                            estimate = read(io.StringIO(completed.stdout))
                            if len(estimate) != len(expected_times) or not np.allclose(
                                estimate["t_s"], expected_times, rtol=0, atol=5e-7
                            ):
                                raise ValueError(
                                    f"Missing or mistimed outputs: {tag} {name}"
                                )
                            if not all(
                                np.isfinite(estimate[field]).all()
                                for field in estimate.dtype.names
                            ):
                                raise ValueError(f"Nonfinite replay: {tag} {name}")
                            if np.any(estimate["step_status"]):
                                raise ValueError(
                                    f"Generated code reported an error: {tag} {name}"
                                )
                            arrivals = (
                                timings[0]["transport_delivered"],
                                timings[0]["transport_digest"],
                            )
                            if (
                                shared_arrivals is not None
                                and arrivals != shared_arrivals
                            ):
                                raise ValueError(
                                    "The two estimators received different sensor packets"
                                )
                            shared_arrivals = arrivals
                            record = dict(
                                name=name,
                                tag=tag,
                                frequency=frequency,
                                seed=seed,
                                profile="25/25/25 Hz"
                                if lower_rates
                                else "100/50/50 Hz",
                                scenario=scenario,
                                delay_profile=delay_profile,
                                delays_ms=delays,
                                output_sha256=output_digest,
                                timing=timings,
                            )
                            for window, start, end in (
                                ("flight", 13, 60),
                                ("before_outage", 18, 25),
                                ("outage_window", 25, 40),
                                ("after_return", 40, 60),
                            ):
                                record[window] = metrics(estimate, truth, start, end)
                            if scenario == "transition":
                                record["transition"] = transition(estimate, truth)
                            if name == "horizon":
                                if any(
                                    timing["late"]
                                    or timing["overflow"]
                                    or timing["stale"]
                                    for timing in timings
                                ):
                                    raise ValueError(
                                        f"Horizon dropped a measurement: {tag} {scenario} {timings[0]}"
                                    )
                                record["fusion_state"] = metrics(
                                    horizon_state(estimate), truth, 13, 59
                                )
                                released = estimate["t_s"] >= 1
                                ages = (
                                    estimate["t_s"][released]
                                    - estimate["fusion_t_s"][released]
                                )
                                record["fusion_lag_s"] = dict(
                                    minimum=float(ages.min()), maximum=float(ages.max())
                                )
                                if ages.min() < 0.2 - 1e-5 or ages.max() > 0.211:
                                    raise ValueError(
                                        "Fusion-state epochs disagree with the configured horizon"
                                    )
                            records.append(record)
                    print(f"Scored {tag} {delay_profile}", flush=True)
    result = dict(
        label=args.label,
        stationary_until_s=args.stationary_until,
        host=platform.platform(),
        timing_note="Per-component thread CPU time includes timer overhead; excludes CSV, transport simulation, and adapter copies. Host measurements are not flight-target WCET.",
        horizon_note="Generated components are scheduled explicitly; the composed Modelica block remains blocked by the Rumoca 0.10.2 sampled-read diagnostic. Queue work runs on sensor arrival or fusion release, rather than every IMU tick. Horizon rebases publish one IMU tick after a filter correction.",
        replay_sha256={
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in (
                ("retrodiction", args.retrodiction),
                ("horizon", args.horizon),
            )
        },
        input_sha256=capture_hashes,
        scores=records,
    )
    if args.imu_noise_density:
        result["imu_noise_density"] = args.imu_noise_density
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Saved {len(records)} paired estimator scores.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("retrodiction", "horizon", "captures", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--label", default="ESKF delayed fusion comparison")
    parser.add_argument("--frequencies", type=float, nargs="+", default=[0.25, 0.6])
    parser.add_argument("--seeds", type=int, nargs="+", default=[7, 19, 41])
    parser.add_argument(
        "--delay-profiles", nargs="+", choices=PROFILES, default=list(PROFILES)
    )
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--stationary-until", type=float, default=0)
    parser.add_argument(
        "--imu-noise-density", type=float, nargs=2, metavar=("GYRO", "ACCEL")
    )
    compare(parser.parse_args())
