"""Retain every paired flight result while challenging covariance validation."""

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from compare_exposure import eskf


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new work and evidence paths")
    reference = json.loads(args.reference.read_text())
    root_reference = json.loads(args.root_reference.read_text())
    for label, hashes in reference["input_sha256"].items():
        if any(
            digest(args.captures / label / name) != value
            for name, value in hashes.items()
        ):
            raise ValueError("Frozen sensor capture changed")
    args.work.mkdir(parents=True)
    result = dict(
        reference_sha256=digest(args.reference),
        root_reference_sha256=digest(args.root_reference),
        scores=[],
        binary_sha256={
            p.name: digest(p)
            for p in args.binaries.glob("*-*")
            if p.name
            in (
                "horizon-control",
                "retrodiction-control",
                "horizon-root",
                "retrodiction-root",
            )
        },
    )
    for prior in reference["scores"]:
        if prior["name"] not in ("horizon", "retrodiction"):
            continue
        keys = {
            key: prior[key]
            for key in (
                "name",
                "seed",
                "frequency",
                "climb_height_m",
                "scenario",
                "explicit_exposure",
            )
        }
        capture = (
            args.captures
            / f"{keys['frequency']}_{keys['seed']}_{keys['climb_height_m']}m"
        )
        for representation in ("dense", "root"):
            mode = "control" if representation == "dense" else "root"
            work = (
                args.work
                / f"{capture.name}-{keys['scenario']}-{keys['explicit_exposure']}-{keys['name']}-{representation}"
            )
            work.mkdir()
            options = SimpleNamespace(
                work=work,
                delay_profile="exposure",
                seed=keys["seed"],
                imu_noise_density=reference["imu_noise_density"],
            )
            scored = eskf(
                args.binaries / (keys["name"] + "-" + mode),
                capture,
                keys["scenario"],
                options,
                keys["explicit_exposure"],
                prior["transport"],
            )
            previous = (
                prior
                if representation == "dense"
                else next(
                    row
                    for row in root_reference["scores"]
                    if all(row[key] == value for key, value in keys.items())
                )
            )
            scored.update(
                keys,
                representation=representation,
                previous_output_sha256=previous["output_sha256"],
                previous_timing=previous["timing"],
                output_identical=scored["output_sha256"] == previous["output_sha256"],
                previous_flight=previous["flight"],
            )
            result["scores"].append(scored)
            args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            print(
                work.name,
                "identical" if scored["output_identical"] else "CHANGED",
                flush=True,
            )
    assert len(result["scores"]) == 96
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "reference",
        "root-reference",
        "captures",
        "binaries",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    raise SystemExit(run(parser.parse_args()))
