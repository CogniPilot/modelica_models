#!/usr/bin/env python3
"""Compare native magnetic activation and ESKF at two declared climb heights."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np

from audit_native_configuration import digest, prepare_px4, probe, probe_px4
from compare_delay import PROFILES
from generate import generate
from score import metrics, read, transition


def eskf(args, binary, capture, scenario, seed):
    completed = subprocess.run(
        [
            str(binary),
            str(capture / "imu.csv"),
            str(capture / "modelica_input.csv"),
            "-",
            scenario,
            "foh",
            "--delays",
            *map(str, PROFILES[args.delay_profile]),
            str(seed),
            "--timing",
            "-",
            "--stationary-until",
            "13",
            "--imu-noise-density",
            *map(str, args.noise_density),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    estimate = read(io.StringIO(completed.stdout))
    if (
        len(estimate) != 6001
        or any(not np.isfinite(estimate[field]).all() for field in estimate.dtype.names)
        or np.any(estimate["step_status"])
    ):
        raise ValueError("Missing, invalid or nonfinite ESKF outputs")
    truth = read(capture / "truth.csv")
    result = dict(
        output_sha256=hashlib.sha256(completed.stdout.encode()).hexdigest(),
        timing=json.loads(completed.stderr),
        flight=metrics(estimate, truth, 13, 60),
        outage_window=metrics(estimate, truth, 25, 40),
        after_return=metrics(estimate, truth, 40, 60),
    )
    if scenario == "transition":
        result["transition"] = transition(estimate, truth)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "captures",
        "harness",
        "ap-source",
        "ap-replay",
        "px4-source",
        "px4-library",
        "transport-trace",
        "retrodiction",
        "horizon",
        "calibration",
        "native-reference",
        "eskf-reference",
        "audit-reference",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--frequency", type=float, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--delay-profile", choices=PROFILES, default="nominal")
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and output paths")
    native = json.loads(args.native_reference.read_text())
    old_eskf = json.loads(args.eskf_reference.read_text())
    old_audit = json.loads(args.audit_reference.read_text())
    calibration = json.loads(args.calibration.read_text())
    args.noise_density = calibration["imu_noise_density"]
    args.takeoff_clear_s = 18.0
    for name, path in (("px4", args.px4_source), ("ardupilot", args.ap_source)):
        pin = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
        if pin != native["native_source_pins"][name]:
            raise ValueError("Native source pin changed")
    for name, expected in native["external_adapter_sha256"].items():
        if digest(args.harness / name) != expected:
            raise ValueError("External adapter changed")
    for key, path in (
        ("ekf3", args.ap_replay),
        ("px4_core_library", args.px4_library),
        ("transport_trace", args.transport_trace),
    ):
        if digest(path) != native["binary_sha256"][key]:
            raise ValueError("Native binary or transport changed")
    args.work.mkdir(parents=True)
    px4 = prepare_px4(args)
    case_input = args.work / "capture"
    case_input.mkdir()
    evidence = dict(
        source_sha256={
            name: digest(Path(__file__).parent / name)
            for name in (
                "compare_altitude.py",
                "generate.py",
                "native_delay.py",
                "native_phase.py",
                "audit_native_configuration.py",
                "transport.h",
                "score.py",
            )
        },
        native_source_pins=native["native_source_pins"],
        binary_sha256={
            name: digest(path)
            for name, path in (
                ("px4_adapter", px4),
                ("px4_core", args.px4_library),
                ("ekf3", args.ap_replay),
                ("transport_trace", args.transport_trace),
                ("retrodiction", args.retrodiction),
                ("horizon", args.horizon),
            )
        },
        reference_sha256={
            path.name: digest(path)
            for path in (
                args.native_reference,
                args.eskf_reference,
                args.audit_reference,
                args.calibration,
            )
        },
        noise_density=args.noise_density,
        frequency=args.frequency,
        seeds=args.seeds,
        delay_profile=args.delay_profile,
        climb_heights_m=[2, 4],
        aiding_profile="100/50/50 Hz",
        takeoff_expectation_clear_s=args.takeoff_clear_s,
        limitations=[
            "Native tuning, measurement averaging/time semantics and internal sensor rates remain unequal",
            "Declared five-second takeoff phase, not the full Copter detector",
            "Height changes vertical acceleration and angular-flow geometry as well as magnetic activation",
            "Nominal-delay high-rate comparison; not the full rate/delay or disturbance matrix",
            "Native selection/last-fuse diagnostics do not count every accepted scalar observation",
        ],
        input_sha256={},
        controls=[],
        scores=[],
    )
    for seed in args.seeds:
        tag = f"{args.frequency}_{seed}"
        original_capture = args.captures / tag
        for height in (2, 4):
            capture = original_capture
            if height == 4:
                capture = args.work / (tag + "_4m")
                generate(capture, seed, args.frequency, climb_height_m=height)
            args.capture = capture
            evidence["input_sha256"][f"{tag}_{height}m"] = {
                path.name: digest(path)
                for path in sorted(capture.glob("*"))
                if path.is_file()
            }
            for name in (
                "imu.csv",
                "gps.csv",
                "flow.csv",
                "mag.csv",
                "baro.csv",
                "origin.json",
            ):
                link = case_input / name
                if link.is_symlink():
                    link.unlink()
                link.symlink_to((capture / name).resolve())
            for scenario in ("gps", "denied", "transition"):
                print(tag, height, scenario, flush=True)
                trace = subprocess.run(
                    [
                        str(args.transport_trace),
                        str(capture / "modelica_input.csv"),
                        scenario,
                        *map(str, PROFILES[args.delay_profile]),
                        str(seed),
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                arrivals = np.atleast_1d(
                    np.genfromtxt(io.StringIO(trace.stdout), names=True, delimiter=",")
                )
                (case_input / "arrivals.csv").write_text(trace.stdout)
                transport = json.loads(trace.stderr)
                results = {
                    "retrodiction": eskf(
                        args, args.retrodiction, capture, scenario, seed
                    ),
                    "horizon": eskf(args, args.horizon, capture, scenario, seed),
                    "px4": probe_px4(args, px4, case_input, scenario),
                    "ekf3": probe(args, case_input, arrivals, scenario, "baseline"),
                }
                for name, result in results.items():
                    if name in ("retrodiction", "horizon"):
                        if any(
                            result["timing"][key] != value
                            for key, value in transport.items()
                        ):
                            raise ValueError("ESKF and native packet delivery differs")
                        if name == "horizon" and any(
                            result["timing"][key]
                            for key in ("late", "stale", "overflow")
                        ):
                            raise ValueError("Horizon dropped measurements")
                    if height == 2 and name != "ekf3":
                        reference = native if name == "px4" else old_eskf
                        old = next(
                            row
                            for row in reference["scores"]
                            if row["name"] == name
                            and row["tag"] == tag
                            and row["scenario"] == scenario
                            and row["delay_profile"] == args.delay_profile
                        )
                        if result["output_sha256"] != old["output_sha256"]:
                            raise ValueError("Low-altitude control changed")
                        evidence["controls"].append(
                            dict(name=name, tag=tag, scenario=scenario, identical=True)
                        )
                    if (
                        height == 2
                        and name == "ekf3"
                        and tag == old_audit["capture_tag"]
                    ):
                        old = next(
                            row
                            for row in old_audit["scores"]
                            if row["scenario"] == scenario
                            and row["variant"] == "finite_takeoff"
                        )
                        if result["output_sha256"] != old["output_sha256"]:
                            raise ValueError(
                                "Takeoff correction differs from its audited ablation"
                            )
                        evidence["controls"].append(
                            dict(
                                name="ekf3_finite_takeoff",
                                tag=tag,
                                scenario=scenario,
                                identical=True,
                            )
                        )
                    result.update(
                        name=name,
                        tag=tag,
                        seed=seed,
                        climb_height_m=height,
                        scenario=scenario,
                        delay_profile=args.delay_profile,
                        transport=transport,
                    )
                    evidence["scores"].append(result)
                args.output.write_text(
                    json.dumps(evidence, indent=2, allow_nan=False) + "\n"
                )


if __name__ == "__main__":
    main()
