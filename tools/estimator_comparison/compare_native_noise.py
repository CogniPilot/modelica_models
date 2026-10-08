#!/usr/bin/env python3
"""Apply one frozen IMU calibration to pinned native replay configurations."""

import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np

from audit_native_configuration import digest, prepare_px4, probe, probe_px4
from compare_delay import PROFILES
from native_noise import native_noise, PREDICTION_PERIOD_S


def check_settings(result, name, expected):
    if name == "px4":
        if not np.allclose(
            result["rate_noise_std_range"], [expected, expected], rtol=2e-7, atol=1e-12
        ):
            raise ValueError(
                "PX4 runtime noise settings differ from the requested mapping"
            )
        interval = result["prediction_dt_s"]
    else:
        actual = [
            result["parameters"][key] for key in ("EK3_GYRO_P_NSE", "EK3_ACC_P_NSE")
        ]
        if not np.allclose(actual, expected, rtol=2e-7, atol=1e-12):
            raise ValueError(
                "EKF3 logged noise settings differ from the requested mapping"
            )
        timing = [row for row in result["timing"] if row["TimeUS"] >= 13_000_000]
        if not timing:
            raise ValueError("Missing native prediction timing diagnostics")
        interval = [
            min(row["AngMin"] for row in timing),
            max(row["AngMax"] for row in timing),
            min(row["VMin"] for row in timing),
            max(row["VMax"] for row in timing),
        ]
    if not np.allclose(interval, PREDICTION_PERIOD_S[name], rtol=1e-4, atol=0):
        raise ValueError(
            "Prediction interval changed; frozen mapping is no longer applicable"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "captures",
        "higher-captures",
        "reference",
        "calibration",
        "native-reference",
        "harness",
        "ap-source",
        "ap-replay",
        "px4-source",
        "px4-library",
        "transport-trace",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    reference = json.loads(args.reference.read_text())
    historical = json.loads(args.native_reference.read_text())
    calibration = json.loads(args.calibration.read_text())
    densities = calibration["imu_noise_density"]
    rates = native_noise(densities)
    if densities != reference["noise_density"]:
        raise ValueError("Native and ESKF calibration differ")
    for name, path in (("px4", args.px4_source), ("ardupilot", args.ap_source)):
        pin = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
        if pin != reference["native_source_pins"][name]:
            raise ValueError("Native source pin changed")
    for name, expected in historical["external_adapter_sha256"].items():
        if digest(args.harness / name) != expected:
            raise ValueError("External adapter changed")
    for key, path in (
        ("ekf3", args.ap_replay),
        ("px4_core", args.px4_library),
        ("transport_trace", args.transport_trace),
    ):
        if digest(path) != reference["binary_sha256"][key]:
            raise ValueError("Native binary or transport changed")
    args.delay_profile = reference["delay_profile"]
    args.takeoff_clear_s = reference["takeoff_expectation_clear_s"]
    args.work.mkdir(parents=True)
    profiles = {}
    for profile, density in (("default_control", None), ("calibrated", densities)):
        options = argparse.Namespace(**vars(args))
        options.work = args.work / profile
        options.work.mkdir()
        options.imu_noise_density = density
        profiles[profile] = (options, prepare_px4(options))
    capture = args.work / "capture"
    capture.mkdir()
    evidence = dict(
        source_sha256={
            name: digest(Path(__file__).parent / name)
            for name in (
                "compare_native_noise.py",
                "native_delay.py",
                "native_noise.py",
                "native_phase.py",
                "audit_native_configuration.py",
                "transport.h",
                "score.py",
            )
        },
        reference_sha256={
            path.name: digest(path)
            for path in (args.reference, args.native_reference, args.calibration)
        },
        native_source_pins=reference["native_source_pins"],
        core_binary_sha256={
            "px4": digest(args.px4_library),
            "ekf3": digest(args.ap_replay),
        },
        adapter_binary_sha256={
            name: digest(binary) for name, (_, binary) in profiles.items()
        },
        patched_adapter_sha256={
            name: digest(options.work / "px4-arrivals.cpp")
            for name, (options, _) in profiles.items()
        },
        imu_noise_density=densities,
        rate_noise_std=rates,
        nominal_prediction_period_s=PREDICTION_PERIOD_S,
        takeoff_expectation_clear_s=args.takeoff_clear_s,
        delay_profile=args.delay_profile,
        frequency=reference["frequency"],
        seeds=reference["seeds"],
        limitations=[
            "Only IMU white-noise density is matched; bias drift, priors and measurement tuning remain unchanged",
            "Frozen rate-noise mapping uses observed settled prediction intervals; startup intervals may differ",
            "Native sensor averaging/delay semantics, source policies and ingestion rates remain unequal",
            "Synthetic high-rate nominal-delay experiment; not a general algorithm ranking",
        ],
        controls=[],
        scores=[],
    )
    for seed in reference["seeds"]:
        tag = f"{reference['frequency']}_{seed}"
        for height in (2, 4):
            actual = (
                args.captures / tag
                if height == 2
                else args.higher_captures / (tag + "_4m")
            )
            hashes = reference["input_sha256"][f"{tag}_{height}m"]
            if any(
                digest(actual / name) != expected for name, expected in hashes.items()
            ):
                raise ValueError("Altitude capture changed")
            for name in (
                "imu.csv",
                "gps.csv",
                "flow.csv",
                "mag.csv",
                "baro.csv",
                "origin.json",
            ):
                link = capture / name
                if link.is_symlink():
                    link.unlink()
                link.symlink_to((actual / name).resolve())
            for scenario in ("gps", "denied", "transition"):
                trace = subprocess.run(
                    [
                        str(args.transport_trace),
                        str(actual / "modelica_input.csv"),
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
                transport = json.loads(trace.stderr)
                (capture / "arrivals.csv").write_text(trace.stdout)
                for profile, (options, px4) in profiles.items():
                    if profile == "default_control" and seed != reference["seeds"][0]:
                        continue
                    options.capture = actual
                    print(tag, height, scenario, profile, flush=True)
                    for name in ("px4", "ekf3"):
                        previous = next(
                            row
                            for row in reference["scores"]
                            if row["tag"] == tag
                            and row["climb_height_m"] == height
                            and row["scenario"] == scenario
                            and row["name"] == name
                        )
                        if transport != previous["transport"]:
                            raise ValueError(
                                "Packet delivery differs from the ESKF/native reference"
                            )
                        result = (
                            probe_px4(options, px4, capture, scenario)
                            if name == "px4"
                            else probe(options, capture, arrivals, scenario, "baseline")
                        )
                        if profile == "default_control":
                            if result["output_sha256"] != previous["output_sha256"]:
                                raise ValueError(
                                    "Omitted native calibration changed baseline output"
                                )
                            evidence["controls"].append(
                                dict(
                                    name=name,
                                    tag=tag,
                                    climb_height_m=height,
                                    scenario=scenario,
                                    identical=True,
                                    output_sha256=result["output_sha256"],
                                )
                            )
                        else:
                            check_settings(result, name, rates[name])
                            if any(
                                result[window]["finite_fraction"] != 1
                                for window in (
                                    "flight",
                                    "outage_window",
                                    "after_return",
                                )
                            ):
                                raise ValueError("Nonfinite calibrated output")
                            result.update(
                                name=name,
                                tag=tag,
                                seed=seed,
                                climb_height_m=height,
                                scenario=scenario,
                                transport=transport,
                                arrival_trace_sha256=hashlib.sha256(
                                    trace.stdout.encode()
                                ).hexdigest(),
                            )
                            evidence["scores"].append(result)
                        args.output.write_text(
                            json.dumps(evidence, indent=2, allow_nan=False) + "\n"
                        )


if __name__ == "__main__":
    main()
