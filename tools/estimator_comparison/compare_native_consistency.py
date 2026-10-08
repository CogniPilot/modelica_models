"""Check native observer output parity and common-state NEES on frozen cases."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np

from audit_native_configuration import prepare_px4, probe, probe_px4
from compare_delay import PROFILES
from compare_exposure import digest
from native_consistency import check
from native_innovations import check as check_innovations
from native_release import SOURCE_PINS


def verify_observers(args):
    observers = {}
    for name in ("px4", "ardupilot"):
        source = getattr(args, name + "_source")
        evidence = json.loads(getattr(args, name + "_observer").read_text())
        revision = subprocess.check_output(
            ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
        ).strip()
        changed = subprocess.check_output(
            ["git", "-C", str(source), "diff", "--name-only"], text=True
        ).splitlines()
        if revision != SOURCE_PINS[name] or evidence["native_revision"] != revision:
            raise ValueError("Native observer source revision changed")
        records = evidence.get("files", [evidence] if "file" in evidence else [])
        if not records or changed != sorted(record["file"] for record in records):
            raise ValueError("Only the recorded read-only native observer may differ")
        header = (
            "native_innovation_dump.h"
            if getattr(args, "observation", "covariance") == "innovations"
            else "native_covariance_dump.h"
        )
        for record in records:
            path = source / record["file"]
            if (
                digest(path) != record["after_sha256"]
                or digest(path.with_name(header)) != evidence["observer_sha256"]
            ):
                raise ValueError("Native observer source changed after instrumentation")
            original = subprocess.check_output(
                ["git", "-C", str(source), "show", "HEAD:" + record["file"]]
            )
            if hashlib.sha256(original).hexdigest() != record["before_sha256"]:
                raise ValueError(
                    "Native observer baseline differs from its pinned source"
                )
        observers[name] = evidence
    return observers


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    reference = json.loads(args.reference.read_text())
    if reference.get("native_campaign") != "stable-releases":
        raise ValueError("Use the completed stable release comparison")
    if digest(args.transport_trace) != reference["binary_sha256"]["transport_trace"]:
        raise ValueError("Frozen packet transport executable changed")
    observers = verify_observers(args)
    innovations = getattr(args, "observation", "covariance") == "innovations"
    args.work.mkdir(parents=True)
    base_work = args.work
    args.imu_noise_density = reference["imu_noise_density"]
    args.px4_release = "v1.17.0"
    args.exposure_flow = True
    args.delay_profile = "exposure"
    args.takeoff_clear_s = 18
    args.work = base_work / "px4-adapter"
    args.work.mkdir()
    px4 = prepare_px4(args)
    result = dict(
        observer_manifests=observers,
        reference_sha256=digest(args.reference),
        binary_sha256=dict(px4=digest(px4), ekf3=digest(args.ap_replay)),
        native_core_sha256=dict(
            px4=digest(args.px4_library), ekf3=digest(args.ap_replay)
        ),
        scores=[],
        scope="Native full covariance observer, frozen published-state parity, common 15D marginal NEES at native fusion epochs. No NIS or matched effective R/Q claim.",
    )
    if innovations:
        result["scope"] = (
            "Actual native scalar correction inputs and selected separate gate inputs; "
            "frozen published-state parity. Accepted-update NIS is conditional on selection. "
            "No full joint NIS, complete rejection coverage or matched effective R/Q claim."
        )
    for previous in reference["scores"]:
        if previous["name"] not in ("px4", "ekf3") or not previous["explicit_exposure"]:
            continue
        name = previous["name"]
        label = f"{previous['frequency']}_{previous['seed']}_{previous['climb_height_m']}m_{previous['scenario']}_explicit"
        original = args.cases / label / "capture"
        source = (
            f"{previous['frequency']}_{previous['seed']}_{previous['climb_height_m']}m"
        )
        for path, expected in reference["input_sha256"][source].items():
            if digest(original / path) != expected:
                raise ValueError("Frozen physical capture changed")
        args.work = base_work / (label + "-" + name)
        args.work.mkdir()
        capture = args.work / "capture"
        capture.mkdir()
        for path in original.iterdir():
            if path.is_file() and path.name != "arrivals.csv":
                (capture / path.name).symlink_to(path.resolve())
        trace = subprocess.run(
            [
                str(args.transport_trace),
                str(capture / "modelica_input.csv"),
                previous["scenario"],
                *map(str, PROFILES["exposure"]),
                str(previous["seed"]),
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        (capture / "arrivals.csv").write_text(trace.stdout)
        if (
            digest(capture / "arrivals.csv") != previous["arrival_trace_sha256"]
            or json.loads(trace.stderr) != previous["transport"]
        ):
            raise ValueError("Frozen packet delivery differs")
        args.capture = capture
        args.seed = previous["seed"]
        observation = args.work / (
            "native-innovations.csv" if innovations else "native-covariance.csv"
        )
        environment_key = (
            "NATIVE_INNOVATION_PATH" if innovations else "NATIVE_COVARIANCE_PATH"
        )
        old_environment = os.environ.get(environment_key)
        os.environ[environment_key] = str(observation.resolve())
        try:
            if name == "px4":
                score = probe_px4(args, px4, capture, previous["scenario"])
            else:
                arrivals = np.genfromtxt(
                    capture / "arrivals.csv", delimiter=",", names=True
                )
                score = probe(args, capture, arrivals, previous["scenario"], "baseline")
        finally:
            if old_environment is None:
                del os.environ[environment_key]
            else:
                os.environ[environment_key] = old_environment
        identical = score["output_sha256"] == previous["output_sha256"]
        row = dict(
            **{
                key: previous[key]
                for key in ("name", "seed", "frequency", "climb_height_m", "scenario")
            },
            output_sha256=score["output_sha256"],
            previous_output_sha256=previous["output_sha256"],
            output_identical=identical,
        )
        result["scores"].append(row)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        if not identical:
            raise ValueError(
                "Native instrumentation changed published outputs; retain the failure"
            )
        evidence = args.work / (
            "innovations.json" if innovations else "consistency.json"
        )
        (check_innovations if innovations else check)(
            SimpleNamespace(
                filter=name,
                covariance=observation,
                innovations=observation,
                truth=capture / "truth.csv",
                output=evidence,
            )
        )
        row["innovations" if innovations else "consistency"] = json.loads(
            evidence.read_text()
        )
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(label, name, "published output identical", flush=True)
    if len(result["scores"]) != 24:
        raise ValueError("Incomplete native covariance campaign")
    if innovations:
        result["complete"] = True
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "reference",
        "cases",
        "harness",
        "px4-source",
        "px4-library",
        "ardupilot-source",
        "px4-observer",
        "ardupilot-observer",
        "ap-replay",
        "transport-trace",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", default="c++")
    parser.add_argument(
        "--observation", choices=("covariance", "innovations"), default="covariance"
    )
    run(parser.parse_args())
