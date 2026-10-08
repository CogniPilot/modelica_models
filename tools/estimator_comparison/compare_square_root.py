#!/usr/bin/env python3
"""Compare the square-root ESKF with frozen full-covariance exposure replays."""

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


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new work and evidence paths")
    reference = json.loads(args.reference.read_text())
    root_reference = None
    if args.root_reference:
        root_reference = json.loads(args.root_reference.read_text())
        if root_reference["reference_sha256"] != digest(args.reference):
            raise ValueError("Previous root replay used a different frozen reference")
    for label, hashes in reference["input_sha256"].items():
        if any(
            digest(args.captures / label / name) != value
            for name, value in hashes.items()
        ):
            raise ValueError("Frozen exposure capture changed")
    args.work.mkdir(parents=True)
    result = dict(
        reference_sha256=digest(args.reference),
        binary_sha256={
            name: digest(getattr(args, name))
            for name in (
                "horizon_control",
                "retrodiction_control",
                "horizon_root",
                "retrodiction_root",
            )
        },
        controls=[],
        scores=[],
    )
    if root_reference:
        result["root_reference_sha256"] = digest(args.root_reference)
    baseline_rows = [
        r for r in reference["scores"] if r["name"] in ("horizon", "retrodiction")
    ]
    if len(baseline_rows) != 48:
        raise ValueError("Expected the complete 48-case ESKF reference")
    for baseline in baseline_rows:
        keys = {
            key: baseline[key]
            for key in (
                "name",
                "seed",
                "frequency",
                "climb_height_m",
                "scenario",
                "explicit_exposure",
            )
        }
        label = f"{keys['frequency']}_{keys['seed']}_{keys['climb_height_m']}m"
        capture = args.captures / label
        case = (
            args.work
            / f"{label}_{keys['scenario']}_{keys['explicit_exposure']}_{keys['name']}"
        )
        case.mkdir()
        options = [
            "--stationary-until",
            "13",
            "--delays",
            *map(str, PROFILES["exposure"]),
            str(keys["seed"]),
            "--imu-noise-density",
            *map(str, reference["imu_noise_density"]),
            "--timing",
            "-",
        ]
        if keys["explicit_exposure"]:
            options += ["--flow-packets", str(capture / "flow.csv")]
        control_timing = None
        for mode in ("control", "root"):
            binary = getattr(args, keys["name"] + "_" + mode)
            covariance = case / "covariance.csv"
            command = [
                str(binary),
                str(capture / "imu.csv"),
                str(capture / "modelica_input.csv"),
                "-",
                keys["scenario"],
                "foh",
            ]
            if mode == "root":
                command.append(str(covariance))
            completed = subprocess.run(
                command + options, capture_output=True, check=True
            )
            checksum = hashlib.sha256(completed.stdout).hexdigest()
            timing = json.loads(completed.stderr)
            if any(
                timing[key] != value for key, value in baseline["transport"].items()
            ):
                raise ValueError("Sensor delivery changed")
            if keys["name"] == "horizon" and any(
                timing[key] for key in ("late", "stale", "overflow")
            ):
                raise ValueError("Horizon packet lost")
            if mode == "control":
                if checksum != baseline["output_sha256"]:
                    raise ValueError(
                        f"Changed frozen full-covariance control: {case.name}"
                    )
                control_timing = timing
                result["controls"].append(
                    dict(**keys, identical=True, output_sha256=checksum, timing=timing)
                )
                continue
            estimate = read(io.StringIO(completed.stdout.decode()))
            expected = np.loadtxt(
                capture / "modelica_input.csv", delimiter=",", skiprows=1, usecols=0
            )
            if (
                not np.array_equal(estimate["t_s"], expected)
                or any(
                    not np.isfinite(estimate[key]).all() for key in estimate.dtype.names
                )
                or np.any(estimate["step_status"])
            ):
                raise ValueError("Invalid square-root replay output")
            diagnostics = read(covariance)
            truth = read(capture / "truth.csv")
            scored = dict(
                **keys,
                output_sha256=checksum,
                timing=timing,
                control_timing=control_timing,
                flight=metrics(estimate, truth, 13, 60),
                outage_window=metrics(estimate, truth, 25, 40),
                after_return=metrics(estimate, truth, 40, 60),
                consistency={
                    window: consistency(
                        estimate,
                        diagnostics,
                        truth,
                        [0.0008, -0.0005, 0.0004],
                        [0.02, -0.01, 0.015],
                        start,
                        end,
                        fusion_horizon=keys["name"] == "horizon",
                    )
                    for window, start, end in (
                        ("before_takeoff", 10, 13),
                        ("flight", 13, 59.7),
                    )
                },
                covariance_sha256=digest(covariance),
            )
            if root_reference:
                matches = [
                    row
                    for row in root_reference["scores"]
                    if all(row[key] == value for key, value in keys.items())
                ]
                if len(matches) != 1 or matches[0]["output_sha256"] != checksum:
                    raise ValueError(f"Changed previous root output: {case.name}")
                scored["root_control_identical"] = True
                scored["root_covariance_control_identical"] = (
                    matches[0]["covariance_sha256"] == scored["covariance_sha256"]
                )
            if keys["scenario"] == "transition":
                scored["transition"] = transition(estimate, truth)
            (case / "scores.json").write_text(
                json.dumps(scored, indent=2, allow_nan=False) + "\n"
            )
            result["scores"].append(scored)
            covariance.unlink()
        print("Scored " + case.name, flush=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "captures",
        "reference",
        "horizon-control",
        "retrodiction-control",
        "horizon-root",
        "retrodiction-root",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--root-reference", type=Path)
    run(parser.parse_args())
