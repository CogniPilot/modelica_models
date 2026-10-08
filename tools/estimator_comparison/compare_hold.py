#!/usr/bin/env python3
"""Measure FOH versus ZOH separately from the optional stationary IMU model."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np

from compare_delay import PROFILES
from consistency import consistency
from score import metrics, read, transition


WINDOWS = {
    "early_rest": (1, 5),
    "before_takeoff": (10, 13),
    "flight": (13, 59.7),
    "outage": (25, 40),
    "after_return": (40, 59.7),
}


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def replay(job):
    capture, binary, work, settings, reference, density = job
    work.mkdir()
    covariance_path = work / "covariance.csv"
    command = [
        str(binary.resolve()),
        str(capture / "imu.csv"),
        str(capture / "modelica_input.csv"),
        "-",
        settings["scenario"],
        settings["hold"],
        str(covariance_path),
        "--stationary-until",
        "13",
        "--delays",
        *map(str, PROFILES["nominal"]),
        str(settings["seed"]),
        "--imu-noise-density",
        *map(str, density),
        "--timing",
        "-",
    ]
    completed = subprocess.run(command, capture_output=True, check=True)
    estimate = read(io.StringIO(completed.stdout.decode()))
    expected_times = np.loadtxt(
        capture / "modelica_input.csv", delimiter=",", skiprows=1, usecols=0
    )
    if not np.array_equal(estimate["t_s"], expected_times):
        raise ValueError(f"Missing or mistimed output: {settings}")
    if any(not np.isfinite(estimate[name]).all() for name in estimate.dtype.names):
        raise ValueError(f"Nonfinite output: {settings}")
    if np.any(estimate["step_status"]):
        raise ValueError(f"Nonzero generated step status: {settings}")
    timing = json.loads(completed.stderr)
    for name, value in reference["transport"].items():
        if timing[name] != value:
            raise ValueError(f"Different sensor delivery: {settings}")
    if settings["method"] == "horizon" and any(
        timing[key] for key in ("late", "overflow", "stale")
    ):
        raise ValueError(f"Lost horizon packet: {settings}")
    output_hash = hashlib.sha256(completed.stdout).hexdigest()
    control = settings["hold"] == "foh" and not settings["stationary_imu"]
    if control and output_hash != reference["output_sha256"]:
        raise ValueError(f"FOH control changed from frozen reference: {settings}")
    truth = read(capture / "truth.csv")
    diagnostics = read(covariance_path)
    result = dict(
        **settings,
        output_sha256=output_hash,
        covariance_sha256=digest(covariance_path),
        frozen_control_identical=True if control else None,
        timing=timing,
        accuracy={
            window: metrics(estimate, truth, *interval)
            for window, interval in WINDOWS.items()
        },
        consistency={},
    )
    for window in ("before_takeoff", "flight"):
        result["consistency"][window] = consistency(
            estimate,
            diagnostics,
            truth,
            [0.0008, -0.0005, 0.0004],
            [0.02, -0.01, 0.015],
            *WINDOWS[window],
            fusion_horizon=settings["method"] == "horizon",
        )
    if settings["scenario"] == "transition":
        result["transition"] = transition(estimate, truth)
    (work / "score.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    covariance_path.unlink()
    print("Scored " + work.name, flush=True)
    return result


def compare(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new work and evidence paths")
    references = [json.loads(path.read_text()) for path in args.references]
    density = references[0]["noise_density"]
    if any(
        ref["noise_density"] != density or ref["delay_profile"] != "nominal"
        for ref in references
    ):
        raise ValueError("References must share noise tuning and nominal delays")
    args.work.mkdir(parents=True)
    binaries = {
        ("horizon", False): args.horizon,
        ("horizon", True): args.stationary_horizon,
        ("retrodiction", False): args.retrodiction,
        ("retrodiction", True): args.stationary_retrodiction,
    }
    inputs, jobs = {}, []
    for reference, captures, higher_captures in zip(
        references, args.captures, args.higher_captures, strict=True
    ):
        for seed, height, scenario in itertools.product(
            reference["seeds"], (2, 4), ("gps", "denied", "transition")
        ):
            tag = f"{reference['frequency']}_{seed}"
            capture = captures / tag if height == 2 else higher_captures / (tag + "_4m")
            key = f"{tag}_{height}m"
            hashes = reference["input_sha256"][key]
            if any(digest(capture / name) != value for name, value in hashes.items()):
                raise ValueError("Frozen capture changed")
            inputs[key] = hashes
            for method, hold, stationary in itertools.product(
                ("horizon", "retrodiction"), ("foh", "zoh"), (False, True)
            ):
                baseline = next(
                    row
                    for row in reference["scores"]
                    if (
                        row["seed"],
                        row["climb_height_m"],
                        row["scenario"],
                        row["name"],
                    )
                    == (seed, height, scenario, method)
                )
                settings = dict(
                    frequency=reference["frequency"],
                    seed=seed,
                    climb_height_m=height,
                    scenario=scenario,
                    method=method,
                    hold=hold,
                    stationary_imu=stationary,
                )
                work = args.work / (
                    f"{key}_{scenario}_{method}_{hold}_stationary{int(stationary)}"
                )
                jobs.append(
                    (
                        capture,
                        binaries[method, stationary],
                        work,
                        settings,
                        baseline,
                        density,
                    )
                )
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        scores = list(pool.map(replay, jobs))
    evidence = dict(
        experiment="Matched FOH/ZOH x stationary IMU off/on",
        stationary_until_s=13,
        delay_profile="nominal",
        imu_noise_density=density,
        imu_rate_hz=800,
        filter_rate_hz=100,
        aiding_rates_hz=[100, 50, 50],
        reference_sha256={path.name: digest(path) for path in args.references},
        binary_sha256={
            f"{method}_stationary{int(rest)}": digest(binary)
            for (method, rest), binary in binaries.items()
        },
        source_sha256={
            str(path): digest(path)
            for root in (
                "Estimation",
                "LieGroups",
                "LinearAlgebra",
                "Vehicles",
                "Avionics",
                "Tests",
                "tools/estimator_comparison",
            )
            for path in sorted(Path(root).rglob("*"))
            if path.is_file() and path.suffix in (".mo", ".order", ".c", ".h", ".py")
        },
        input_sha256=inputs,
        limitations=[
            "Synthetic frozen captures; nominal delays and high aiding rates only.",
            "Known rest until 13 s; this does not validate a rest detector.",
            "Covariance scored at fusion epoch for horizon, publication epoch for retrodiction.",
            "FOH shared-endpoint noise covariance remains approximate.",
            "NEES samples are correlated; descriptive statistics, not independent Monte Carlo evidence.",
            "Thread CPU timings on this host are not target WCET measurements.",
            "This ablation does not rerun or rank the native PX4/ArduPilot filters.",
        ],
        scores=scores,
    )
    args.output.write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")
    print(f"Saved {len(scores)} paired scores", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "horizon",
        "retrodiction",
        "stationary-horizon",
        "stationary-retrodiction",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    for name in ("references", "captures", "higher-captures"):
        parser.add_argument("--" + name, type=Path, nargs="+", required=True)
    parser.add_argument("--workers", type=int, default=2)
    compare(parser.parse_args())
