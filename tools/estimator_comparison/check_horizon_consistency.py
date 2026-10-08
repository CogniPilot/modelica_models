#!/usr/bin/env python3
"""Check the delayed ESKF covariance against truth at the same fusion epoch."""

import argparse
import hashlib
import io
import itertools
import json
from pathlib import Path
import subprocess

from compare_delay import PROFILES
from consistency import consistency, fusion_horizon_pair
from score import metrics, read


WINDOWS = (
    (1, 5),
    (5, 10),
    (10, 13),
    (13, 18),
    (18, 25),
    (25, 40),
    (40, 59.7),
    (13, 59.7),
)


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def check(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    reference = json.loads(args.reference.read_text())
    expected = set(
        itertools.product(reference["seeds"], (2, 4), ("gps", "denied", "transition"))
    )
    previous = [row for row in reference["scores"] if row["name"] == "horizon"]
    actual = {(row["seed"], row["climb_height_m"], row["scenario"]) for row in previous}
    if expected != actual or len(previous) != len(expected):
        raise ValueError("Incomplete horizon reference matrix")
    if reference["delay_profile"] != "nominal":
        raise ValueError("Expected the nominal-delay altitude experiment")
    args.work.mkdir(parents=True)
    result = dict(
        source_sha256={
            name: digest(Path(__file__).parent / name)
            for name in (
                "check_horizon_consistency.py",
                "consistency.py",
                "replay.c",
                "score.py",
                "horizon.h",
            )
        },
        reference_sha256={args.reference.name: digest(args.reference)},
        replay_sha256=digest(args.horizon),
        imu_noise_density=reference["noise_density"],
        state_epoch="fusion",
        note="Covariance belongs to the delayed filter, not the predicted output at publication time. Correlated samples make NEES descriptive, not independent Monte Carlo evidence.",
        input_sha256={},
        scores=[],
    )
    for seed, height, scenario in sorted(expected):
        tag = f"{reference['frequency']}_{seed}"
        capture = (
            args.captures / tag if height == 2 else args.higher_captures / (tag + "_4m")
        )
        hashes = reference["input_sha256"][f"{tag}_{height}m"]
        if any(digest(capture / name) != value for name, value in hashes.items()):
            raise ValueError("Reference capture changed")
        result["input_sha256"][f"{tag}_{height}m"] = hashes
        baseline = next(
            row
            for row in previous
            if (row["seed"], row["climb_height_m"], row["scenario"])
            == (seed, height, scenario)
        )
        covariance_path = args.work / "covariance.csv"
        completed = subprocess.run(
            [
                str(args.horizon.resolve()),
                str(capture / "imu.csv"),
                str(capture / "modelica_input.csv"),
                "-",
                scenario,
                "foh",
                str(covariance_path),
                "--stationary-until",
                "13",
                "--delays",
                *map(str, PROFILES[reference["delay_profile"]]),
                str(seed),
                "--imu-noise-density",
                *map(str, reference["noise_density"]),
                "--timing",
                "-",
            ],
            capture_output=True,
            check=True,
        )
        output_hash = hashlib.sha256(completed.stdout).hexdigest()
        if output_hash != baseline["output_sha256"]:
            raise ValueError("Covariance-enabled driver changed the primary output")
        timing = json.loads(completed.stderr)
        if any(timing[key] != value for key, value in baseline["transport"].items()):
            raise ValueError("Reference packet delivery changed")
        if any(timing[key] for key in ("late", "overflow", "stale")):
            raise ValueError("Aiding packet lost in horizon queues")
        estimate = read(io.StringIO(completed.stdout.decode()))
        diagnostics = read(covariance_path)
        delayed, _ = fusion_horizon_pair(estimate, diagnostics)
        truth = read(capture / "truth.csv")
        if not (delayed["t_s"][0] < 1 and delayed["t_s"][-1] >= WINDOWS[-1][1]):
            raise ValueError("Delayed output does not cover every declared window")
        row = dict(
            seed=seed,
            tag=tag,
            climb_height_m=height,
            scenario=scenario,
            output_sha256=output_hash,
            primary_output_identical=True,
            covariance_sha256=digest(covariance_path),
            fusion_epoch_range_s=[float(delayed["t_s"][0]), float(delayed["t_s"][-1])],
            timing=timing,
            windows=[],
        )
        for start, end in WINDOWS:
            accuracy = metrics(delayed, truth, start, end)
            if accuracy["finite_fraction"] != 1:
                raise ValueError("Nonfinite delayed state")
            row["windows"].append(
                dict(
                    accuracy=accuracy,
                    consistency=consistency(
                        estimate,
                        diagnostics,
                        truth,
                        [0.0008, -0.0005, 0.0004],
                        [0.02, -0.01, 0.015],
                        start,
                        end,
                        fusion_horizon=True,
                    ),
                )
            )
        result["scores"].append(row)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(
            tag,
            height,
            scenario,
            "flight fusion NEES",
            row["windows"][-1]["consistency"]["mean_nees_15d"],
            flush=True,
        )
    covariance_path.unlink()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "captures",
        "higher-captures",
        "reference",
        "horizon",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    check(parser.parse_args())
