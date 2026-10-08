"""Join native NEES only after covariance observers reproduce pilot state bytes."""

import argparse
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np

from compare_exposure import digest
from compare_native_consistency import verify_observers
from mission import Mission
from native_consistency import check
from native_delay import prepare_px4, run_ardupilot
from sensor_noise import COMMON_NOISE_PROFILE


def validate_pilot(pilot, allow_incomplete=False):
    names = (
        "horizon",
        "retrodiction",
        "horizon_joint",
        "retrodiction_joint",
        "px4",
        "ekf3",
    )
    scenarios = pilot.get("scenarios", ("gps", "denied", "transition"))
    if (
        not scenarios
        or len(set(scenarios)) != len(scenarios)
        or not set(scenarios) <= {"gps", "denied", "transition"}
    ):
        raise ValueError("Require distinct declared pilot scenarios")
    expected = {(name, scenario) for name in names for scenario in scenarios}
    keys = [(score["name"], score["scenario"]) for score in pilot["scores"]]
    if len(set(keys)) != len(keys) or not set(keys) <= expected:
        raise ValueError("Duplicated or unknown pilot condition")
    if (not pilot["complete"] or set(keys) != expected) and not allow_incomplete:
        raise ValueError("Require the complete declared readiness campaign")
    native = [score for score in pilot["scores"] if score["name"] in ("px4", "ekf3")]
    if not native:
        raise ValueError("Require at least one actual native replay")
    return native


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    pilot = json.loads(args.pilot.read_text())
    native_scores = validate_pilot(pilot, getattr(args, "allow_incomplete", False))
    mission = Mission(pilot["mission"]["warmup_s"])
    args.noise_profile = pilot.get("noise_profile", "native-default-aiding")
    args.sensor_informed_noise = args.noise_profile == COMMON_NOISE_PROFILE
    observers = verify_observers(args)
    args.work.mkdir(parents=True)
    root = args.work
    args.arm_after_s = mission.warmup_s
    args.takeoff_clear_s = mission.warmup_s + 5
    args.imu_noise_density = json.loads(args.reference.read_text())["imu_noise_density"]
    if digest(args.reference) != pilot["reference_sha256"]:
        raise ValueError("Pilot noise reference changed")
    args.px4_release = "v1.17.0"
    args.exposure_flow = True
    px4 = prepare_px4(args)
    result = dict(
        complete=False,
        pilot_complete=pilot["complete"],
        pilot_sha256=digest(args.pilot),
        observer_manifests=observers,
        binary_sha256=dict(px4=digest(args.px4_library), ekf3=digest(args.ap_replay)),
        scores=[],
        scope="Native 15D marginal NEES at physical fusion epochs. Every published state CSV must match the separate innovation-observer replay byte for byte. Native covariance and priors remain stack-specific.",
    )
    previous = os.environ.get("NATIVE_COVARIANCE_PATH")
    try:
        for score in native_scores:
            name, scenario = score["name"], score["scenario"]
            capture = args.cases / scenario / "capture"
            for filename, expected in pilot["input_sha256"].items():
                if (
                    filename != "arrivals.csv"
                    and digest(capture / filename) != expected
                ):
                    raise ValueError("Pilot physical capture changed")
            args.work = root / f"{scenario}-{name}"
            args.work.mkdir()
            observation = args.work / "native-covariance.csv"
            output = args.work / "estimate.csv"
            os.environ["NATIVE_COVARIANCE_PATH"] = str(observation.resolve())
            if name == "px4":
                subprocess.run(
                    [
                        str(px4),
                        "--input",
                        str(capture),
                        "--output",
                        str(output),
                        "--arm-after",
                        str(mission.warmup_s),
                        *(["--no-gps"] if scenario == "denied" else []),
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
            else:
                arrivals = np.genfromtxt(
                    capture / "arrivals.csv", delimiter=",", names=True
                )
                run_ardupilot(args, capture, arrivals, "exposure", scenario, output)
            if digest(output) != score["output_sha256"]:
                raise ValueError(
                    "Native covariance observer changed published state bytes"
                )
            evidence = args.work / "consistency.json"
            try:
                check(
                    SimpleNamespace(
                        filter=name,
                        covariance=observation,
                        truth=capture / "truth.csv",
                        output=evidence,
                        windows=mission.windows,
                    )
                )
            except np.linalg.LinAlgError as error:
                consistency = dict(
                    valid=False,
                    reason=str(error),
                    covariance_sha256=digest(observation),
                )
            else:
                consistency = dict(valid=True, **json.loads(evidence.read_text()))
            result["scores"].append(
                dict(
                    name=name,
                    scenario=scenario,
                    output_identical=True,
                    output_sha256=digest(output),
                    arrival_trace_sha256=digest(capture / "arrivals.csv"),
                    consistency=consistency,
                )
            )
            args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            print(
                f"Verified state parity: {scenario} {name}; covariance valid={consistency['valid']}",
                flush=True,
            )
    finally:
        if previous is None:
            os.environ.pop("NATIVE_COVARIANCE_PATH", None)
        else:
            os.environ["NATIVE_COVARIANCE_PATH"] = previous
    result["complete"] = True
    result["covariance_valid"] = all(
        row["consistency"]["valid"] for row in result["scores"]
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "pilot",
        "cases",
        "reference",
        "harness",
        "px4-source",
        "ardupilot-source",
        "px4-observer",
        "ardupilot-observer",
        "px4-library",
        "ap-replay",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", required=True)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="Diagnose available native rows of a failed pilot without declaring its comparison complete",
    )
    run(parser.parse_args())
