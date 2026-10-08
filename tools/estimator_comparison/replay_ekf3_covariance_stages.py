"""Require frozen capture, published-state and full-covariance parity for EKF3 traces."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np

from compare_exposure import digest
from diagnose_ekf3_covariance_stages import diagnose
from mission import Mission
from native_delay import run_ardupilot
from native_release import SOURCE_PINS
from sensor_noise import COMMON_NOISE_PROFILE


def verify_observer(source, evidence):
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    if (
        revision != SOURCE_PINS["ardupilot"]
        or revision != evidence["logging"]["native_revision"]
    ):
        raise ValueError("Use the pinned native stable release")
    records = {**evidence["files"], evidence["logging"]["file"]: evidence["logging"]}
    changed = subprocess.check_output(
        ["git", "-C", str(source), "diff", "HEAD", "--name-only"], text=True
    ).splitlines()
    if sorted(changed) != sorted(records):
        raise ValueError("Only recorded read-only observer source may differ")
    for name, record in records.items():
        original = subprocess.check_output(
            ["git", "-C", str(source), "show", "HEAD:" + name]
        )
        if (
            hashlib.sha256(original).hexdigest() != record["before_sha256"]
            or digest(source / name) != record["after_sha256"]
        ):
            raise ValueError("Observer source differs from its recorded transformation")
    headers = {
        "native_covariance_dump.h": evidence["logging"]["observer_sha256"],
        "native_covariance_stage_dump.h": evidence["observer_sha256"],
    }
    for name, expected in headers.items():
        if digest(source / "libraries/AP_NavEKF3" / name) != expected:
            raise ValueError("Observer header changed")


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned replay and evidence paths")
    if not 0 <= args.start_us <= args.end_us:
        raise ValueError("Require ordered nonnegative publication bounds")
    pilot = json.loads(args.pilot.read_text())
    baseline = json.loads(args.baseline.read_text())
    observer = json.loads(args.observer.read_text())
    verify_observer(args.source, observer)
    if (
        not pilot["complete"]
        or not baseline["complete"]
        or digest(args.pilot) != baseline["pilot_sha256"]
    ):
        raise ValueError("Require the complete frozen pilot and its covariance replay")
    (score,) = [
        s
        for s in baseline["scores"]
        if s["name"] == "ekf3" and s["scenario"] == args.scenario
    ]
    for filename, expected in pilot["input_sha256"].items():
        if digest(args.capture / filename) != expected:
            raise ValueError("Frozen physical capture changed")
    if digest(args.capture / "arrivals.csv") != score["arrival_trace_sha256"]:
        raise ValueError("Frozen packet arrival trace changed")
    if digest(args.reference) != pilot["reference_sha256"]:
        raise ValueError("Frozen IMU noise reference changed")
    mission = Mission(pilot["mission"]["warmup_s"])
    args.arm_after_s, args.takeoff_clear_s = mission.warmup_s, mission.warmup_s + 5
    args.imu_noise_density = json.loads(args.reference.read_text())["imu_noise_density"]
    args.noise_profile = pilot["noise_profile"]
    if args.noise_profile not in (COMMON_NOISE_PROFILE, "native-default-aiding"):
        raise ValueError("Unknown physical sensor profile")
    args.sensor_informed_noise = args.noise_profile == COMMON_NOISE_PROFILE
    args.exposure_flow = True
    args.work.mkdir(parents=True)
    trace, covariance = args.work / "stages.csv", args.work / "native-covariance.csv"
    output = args.work / "estimate.csv"
    variables = {
        "NATIVE_COVARIANCE_PATH": str(covariance.resolve()),
        "NATIVE_COVARIANCE_STAGE_PATH": str(trace.resolve()),
        "NATIVE_COVARIANCE_STAGE_START_US": str(args.start_us),
        "NATIVE_COVARIANCE_STAGE_END_US": str(args.end_us),
    }
    previous = {key: os.environ.get(key) for key in variables}
    os.environ.update(variables)
    try:
        arrivals = np.genfromtxt(
            args.capture / "arrivals.csv", delimiter=",", names=True
        )
        details = run_ardupilot(
            args, args.capture, arrivals, "exposure", args.scenario, output
        )
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    verify_observer(args.source, observer)
    if (
        digest(output) != score["output_sha256"]
        or digest(covariance) != score["consistency"]["covariance_sha256"]
    ):
        raise ValueError(
            "Observer changed native published states or covariance snapshots"
        )
    evidence = {
        "scenario": args.scenario,
        "state_parity": True,
        "full_covariance_parity": True,
        "state_sha256": digest(output),
        "covariance_sha256": digest(covariance),
        "binary_sha256": digest(args.ap_replay),
        "observer": observer,
        "pilot_sha256": digest(args.pilot),
        "baseline_sha256": digest(args.baseline),
        "input_sha256": {
            name: digest(args.capture / name)
            for name in (*pilot["input_sha256"], "arrivals.csv")
        },
        "native_details": details,
        "analysis": diagnose(trace),
    }
    args.output.write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")
    print("Published states and all native covariance snapshots are byte-identical.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", choices=("gps", "denied", "transition"), required=True
    )
    parser.add_argument("--start-us", type=int, required=True)
    parser.add_argument("--end-us", type=int, required=True)
    for name in (
        "source",
        "observer",
        "pilot",
        "baseline",
        "reference",
        "capture",
        "harness",
        "ap-replay",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
