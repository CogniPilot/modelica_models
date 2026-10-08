"""Replay a common warmup with shifted outage, phase hints and scoring windows."""

import argparse
import io
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np

from compare_delay import PROFILES
from compare_exposure import digest, eskf
from compare_native_consistency import verify_observers
from mission import Mission
from native_delay import common_epochs, prepare_px4, run_ardupilot
from native_innovations import check as innovation_statistics
from native_readiness import check as readiness
from native_aiding_noise import configured_noise
from sensor_noise import COMMON_NOISE_PROFILE, COMMON_SENSOR_NOISE
from score import metrics, read, transition


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    origin = json.loads((args.capture / "origin.json").read_text())
    mission = Mission(origin["arm_after_s"])
    args.noise_profile = origin.get(
        "measurement_noise_profile", "native-default-aiding"
    )
    if args.noise_profile not in ("native-default-aiding", COMMON_NOISE_PROFILE):
        raise ValueError("Unknown capture measurement noise profile")
    args.sensor_informed_noise = args.noise_profile == COMMON_NOISE_PROFILE
    if args.sensor_informed_noise:
        args.measurement_noise = (
            COMMON_SENSOR_NOISE.magnetic_T,
            COMMON_SENSOR_NOISE.gps_velocity_m_s[2],
        )
    args.observation = "innovations"
    observers = verify_observers(args)
    native_reference = json.loads(args.native_reference.read_text())
    for name, path in (("px4", args.px4_library), ("ekf3", args.ap_replay)):
        if digest(path) != native_reference["native_core_sha256"][name]:
            raise ValueError("Audited native innovation core changed")
    args.work.mkdir(parents=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    root = args.work
    args.seed = origin["seed"]
    args.arm_after_s = mission.warmup_s
    args.takeoff_clear_s = mission.warmup_s + 5
    args.mission_offset_s = mission.offset_s
    args.consistency_windows = mission.windows
    args.accuracy_end_s = mission.end_s - 0.3
    args.delay_profile = "exposure"
    args.exposure_flow = True
    args.px4_release = "v1.17.0"
    args.imu_noise_density = json.loads(args.reference.read_text())["imu_noise_density"]
    px4_binary = prepare_px4(args)
    result = dict(
        complete=False,
        mission=dict(
            warmup_s=mission.warmup_s,
            loss_s=mission.loss_s,
            return_s=mission.return_s,
            end_s=mission.end_s,
            windows=mission.windows,
        ),
        origin=origin,
        noise_profile=args.noise_profile,
        native_noise_configuration=configured_noise(args)
        if args.sensor_informed_noise
        else None,
        observer_manifests=observers,
        native_reference_sha256=digest(args.native_reference),
        input_sha256={
            p.name: digest(p)
            for p in sorted(args.capture.iterdir())
            if p.is_file() and p.name != "arrivals.csv"
        },
        reference_sha256=digest(args.reference),
        binary_sha256={
            name: digest(getattr(args, name))
            for name in (
                "horizon",
                "retrodiction",
                "horizon_joint",
                "retrodiction_joint",
                "transport_trace",
                "px4_library",
                "ap_replay",
            )
        },
        px4_adapter_sha256=digest(root / "px4-arrivals.cpp"),
        scores=[],
        scenarios=getattr(args, "scenarios", ("gps", "denied", "transition")),
        limitations=[
            "Single-capture readiness pilot, not a held-out ranking or universal superiority result.",
            "Native innovation observers measure accepted scalar updates; incomplete candidate and sensor coverage. Their logging affects CPU cost.",
            "Native full-covariance NEES requires a separate observer replay with identical published states and is not included here.",
            "Native magnetic and height policies, priors and effective aiding covariance remain stack-specific.",
            "Same physical capture and delivered sensor packets; flow exposure, range filtering and internal averaging follow native policies.",
            "ESKF uses FOH, geometric startup alignment and vector magnetic fusion; barometer calibration is declared rest only. Joint pressure bias is optional.",
        ],
    )
    previous_observer = os.environ.get("NATIVE_INNOVATION_PATH")
    try:
        for scenario in result["scenarios"]:
            args.work = root / scenario
            args.work.mkdir()
            capture = args.work / "capture"
            capture.mkdir()
            for path in args.capture.iterdir():
                if path.is_file() and path.name != "arrivals.csv":
                    (capture / path.name).symlink_to(path.resolve())
            trace = subprocess.run(
                [
                    str(args.transport_trace),
                    str(capture / "modelica_input.csv"),
                    scenario,
                    *map(str, PROFILES[args.delay_profile]),
                    str(args.seed),
                    str(mission.offset_s),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            (capture / "arrivals.csv").write_text(trace.stdout)
            transport = json.loads(trace.stderr)
            arrivals = np.genfromtxt(
                io.StringIO(trace.stdout), delimiter=",", names=True
            )
            receipt = dict(
                arrival_trace_sha256=digest(capture / "arrivals.csv"),
                transport=transport,
            )
            for name in (
                "horizon",
                "retrodiction",
                "horizon_joint",
                "retrodiction_joint",
            ):
                score = eskf(
                    getattr(args, name), capture, scenario, args, True, transport
                )
                result["scores"].append(
                    dict(name=name, scenario=scenario, **receipt, **score)
                )
                args.output.write_text(
                    json.dumps(result, indent=2, allow_nan=False) + "\n"
                )
                print(f"Scored {scenario} {name}", flush=True)
            for name in ("px4", "ekf3"):
                observer = args.work / f"{name}-innovations.csv"
                output = args.work / f"{name}.csv"
                os.environ["NATIVE_INNOVATION_PATH"] = str(observer.resolve())
                if name == "px4":
                    completed = subprocess.run(
                        [
                            str(px4_binary),
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
                    details = dict(replay_log=completed.stdout + completed.stderr)
                else:
                    details = run_ardupilot(
                        args, capture, arrivals, args.delay_profile, scenario, output
                    )
                times = (
                    np.arange(
                        round(mission.warmup_s * 100), round(args.accuracy_end_s * 100)
                    )
                    / 100
                )
                estimate = common_epochs(read(output), times)
                truth = read(capture / "truth.csv")
                score = dict(
                    name=name,
                    scenario=scenario,
                    **receipt,
                    output_sha256=digest(output),
                    native_details=details,
                    flight=metrics(
                        estimate, truth, mission.warmup_s, args.accuracy_end_s
                    ),
                    outage_window=metrics(
                        estimate, truth, mission.loss_s, mission.return_s
                    ),
                    after_return=metrics(
                        estimate, truth, mission.return_s, args.accuracy_end_s
                    ),
                    readiness=readiness(observer, name, scenario, mission),
                    innovations=innovation_statistics(
                        SimpleNamespace(
                            filter=name,
                            innovations=observer,
                            output=args.work / f"{name}-nis.json",
                            windows=mission.windows,
                        )
                    ),
                )
                if scenario == "transition":
                    score["transition"] = transition(estimate, truth, mission.offset_s)
                result["scores"].append(score)
                args.output.write_text(
                    json.dumps(result, indent=2, allow_nan=False) + "\n"
                )
                if any(
                    score[window]["finite_fraction"] != 1
                    for window in ("flight", "outage_window", "after_return")
                ):
                    raise ValueError("Nonfinite native state in a scoring window")
                print(
                    f"Scored {scenario} {name}: readiness qualified={score['readiness']['qualified']}",
                    flush=True,
                )
    finally:
        if previous_observer is None:
            os.environ.pop("NATIVE_INNOVATION_PATH", None)
        else:
            os.environ["NATIVE_INNOVATION_PATH"] = previous_observer
    result["complete"] = True
    result["readiness_qualified"] = all(
        score["readiness"]["qualified"]
        for score in result["scores"]
        if "readiness" in score
    )
    result["source_sha256"] = {
        p.name: digest(p)
        for p in sorted(Path(__file__).parent.iterdir())
        if p.is_file() and p.suffix in (".py", ".c", ".h")
    }
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "capture",
        "reference",
        "horizon",
        "retrodiction",
        "horizon-joint",
        "retrodiction-joint",
        "transport-trace",
        "harness",
        "px4-source",
        "ardupilot-source",
        "px4-observer",
        "ardupilot-observer",
        "native-reference",
        "px4-library",
        "ap-replay",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", required=True)
    parser.add_argument(
        "--scenarios",
        choices=("gps", "denied", "transition"),
        nargs="+",
        default=("gps", "denied", "transition"),
    )
    run(parser.parse_args())
