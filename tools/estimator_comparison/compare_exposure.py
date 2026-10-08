#!/usr/bin/env python3
"""Compare explicit flow exposure packets with legacy reconstruction at common rates."""

import argparse
import hashlib
import io
import itertools
import json
from pathlib import Path
import subprocess

import numpy as np

from audit_native_configuration import probe, probe_px4
from compare_delay import PROFILES
from consistency import consistency, fusion_horizon_pair
from flow_native import verify_px4
from score import metrics, read, transition


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def eskf(binary, capture, scenario, options, exposure, transport):
    covariance = options.work / "covariance.csv"
    command = [
        str(binary),
        str(capture / "imu.csv"),
        str(capture / "modelica_input.csv"),
        "-",
        scenario,
        "foh",
        str(covariance),
        "--stationary-until",
        "13",
        "--delays",
        *map(str, PROFILES[options.delay_profile]),
        str(options.seed),
        "--imu-noise-density",
        *map(str, options.imu_noise_density),
        "--timing",
        "-",
    ]
    if exposure:
        command += ["--flow-packets", str(capture / "flow.csv")]
    completed = subprocess.run(command, capture_output=True, check=True)
    estimate = read(io.StringIO(completed.stdout.decode()))
    expected = np.loadtxt(
        capture / "modelica_input.csv", delimiter=",", skiprows=1, usecols=0
    )
    if (
        not np.array_equal(estimate["t_s"], expected)
        or any(not np.isfinite(estimate[k]).all() for k in estimate.dtype.names)
        or np.any(estimate["step_status"])
    ):
        raise ValueError("Invalid ESKF production replay")
    timing = json.loads(completed.stderr)
    if any(timing[key] != value for key, value in transport.items()):
        raise ValueError("ESKF and native packet delivery differs")
    horizon = "fusion_t_s" in estimate.dtype.names
    if horizon and any(timing[key] for key in ("late", "stale", "overflow")):
        raise ValueError("Lost horizon packet")
    if (
        exposure
        and timing["flow_exposure_packets"] != transport["transport_delivered"][1]
    ):
        raise ValueError("ESKF lost a canonical exposure packet")
    truth = read(capture / "truth.csv")
    result = dict(
        output_sha256=hashlib.sha256(completed.stdout).hexdigest(),
        timing=timing,
        flight=metrics(estimate, truth, 13, 60),
        outage_window=metrics(estimate, truth, 25, 40),
        after_return=metrics(estimate, truth, 40, 60),
    )
    diagnostics = read(covariance)
    result["consistency"] = {}
    for window, start, end in (("before_takeoff", 10, 13), ("flight", 13, 59.7)):
        try:
            result["consistency"][window] = dict(
                valid=True,
                **consistency(
                    estimate,
                    diagnostics,
                    truth,
                    [0.0008, -0.0005, 0.0004],
                    [0.02, -0.01, 0.015],
                    start,
                    end,
                    fusion_horizon=horizon,
                ),
            )
        except np.linalg.LinAlgError:
            _, covariance_rows = (
                fusion_horizon_pair(estimate, diagnostics)
                if horizon
                else (estimate, diagnostics)
            )
            covariance_rows = covariance_rows[
                (covariance_rows["t_s"] >= start) & (covariance_rows["t_s"] < end)
            ]
            matrices = np.column_stack(
                [
                    covariance_rows[f"p{row}_{column}"]
                    for row in range(15)
                    for column in range(15)
                ]
            ).reshape(-1, 15, 15)
            failed = []
            for index, matrix in enumerate(matrices):
                try:
                    np.linalg.cholesky((matrix + matrix.T) / 2)
                except np.linalg.LinAlgError:
                    failed.append(index)
            if not failed:
                raise
            first = failed[0]
            result["consistency"][window] = dict(
                valid=False,
                reason="Covariance is not positive definite; NEES is undefined and no samples were dropped",
                start_s=start,
                end_s=end,
                rows=len(matrices),
                failed_samples=len(failed),
                first_failed_epoch_s=float(covariance_rows["t_s"][first]),
                first_failed_covariance=matrices[first].tolist(),
                first_failed_eigenvalues=np.linalg.eigvalsh(matrices[first]).tolist(),
                covariance_sha256=digest(covariance),
            )
    if scenario == "transition":
        result["transition"] = transition(estimate, truth)
    covariance.unlink()
    return result


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    review = args.references
    calibration = json.loads((review / "eskf-noise-calibration.json").read_text())
    historical = {
        seed: json.loads(
            (
                review
                / (
                    "native-noise-original.json"
                    if seed == 7
                    else "native-noise-held-out.json"
                )
            ).read_text()
        )
        for seed in (7, 101)
    }
    source_pins = getattr(
        args, "native_source_pins", historical[7]["native_source_pins"]
    )
    for name, path in (("px4", args.px4_source), ("ardupilot", args.ap_source)):
        pin = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
        ).strip()
        if pin != source_pins[name]:
            raise ValueError("Pinned native source changed")
    binary_pins = getattr(
        args, "native_core_sha256", historical[7]["core_binary_sha256"]
    )
    for name, path in (("px4", args.px4_library), ("ekf3", args.ap_replay)):
        if digest(path) != binary_pins[name]:
            raise ValueError("Audited native binary changed")
    args.work.mkdir(parents=True)
    base_work = args.work
    args.imu_noise_density = calibration["imu_noise_density"]
    args.takeoff_clear_s = 18
    result = dict(
        native_campaign=getattr(args, "native_campaign", "historical-pins"),
        imu_noise_density=args.imu_noise_density,
        native_source_pins=source_pins,
        native_core_sha256={
            "px4": digest(args.px4_library),
            "ekf3": digest(args.ap_replay),
        },
        binary_sha256={
            name: digest(getattr(args, name))
            for name in (
                "horizon",
                "retrodiction",
                "px4_explicit",
                "px4_legacy",
                "transport_trace",
            )
        },
        controls=[],
        scores=[],
        input_sha256={},
        limitations=[
            "Two seeds and two heights; an ingestion/measurement-contract probe, not a full held-out campaign.",
            "Flow exposure, gyro independence, packet covariance, aiding rate and magnetic transport delay differ from historical captures; do not attribute the entire historical change to one factor.",
            "Legacy versus explicit mappings below share the same new capture and delivery trace.",
            "Camera gyro is an independent unbiased calibrated synthetic sensor; not navigation-IMU truth subtraction.",
            "No image texture, rolling shutter, magnetic disturbances or vibration model.",
            "EKF3 range processing still assumes a 25 ms delay and applies its own median filter; the supplied range is co-timed with the flow midpoint.",
            "Native height/magnetic policies, priors and measurement noise floors remain stack-specific.",
            "Serialization/API checks establish delivered inputs; they do not count every accepted EKF3 measurement.",
            "FOH shared-endpoint noise and inter-packet image/gyro correlations remain approximate.",
            "Core and adapter hashes identify the actual supplied native binaries; this replay is not a complete firmware or real-flight qualification.",
        ],
    )
    for frequency, seed in ((0.6, 7), (0.85, 101)):
        args.seed = seed
        for height, scenario, exposure in itertools.product(
            (2, 4), ("gps", "denied", "transition"), (False, True)
        ):
            source = args.captures / f"{frequency}_{seed}_{height}m"
            label = f"{frequency}_{seed}_{height}m_{scenario}_{'explicit' if exposure else 'legacy'}"
            args.work = base_work / label
            args.work.mkdir()
            capture = args.work / "capture"
            capture.mkdir()
            for path in source.glob("*"):
                if path.is_file():
                    (capture / path.name).symlink_to(path.resolve())
            result["input_sha256"][source.name] = {
                path.name: digest(path)
                for path in sorted(source.glob("*"))
                if path.suffix in (".csv", ".json") and path.name != "arrivals.csv"
            }
            args.capture = capture
            args.delay_profile = "exposure"
            args.exposure_flow = exposure
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
            (capture / "arrivals.csv").write_text(trace.stdout)
            arrivals = np.genfromtxt(
                io.StringIO(trace.stdout), delimiter=",", names=True
            )
            transport = json.loads(trace.stderr)
            scores = []
            for name, binary in (
                ("horizon", args.horizon),
                ("retrodiction", args.retrodiction),
            ):
                scores.append(
                    dict(
                        name=name,
                        **eskf(binary, capture, scenario, args, exposure, transport),
                    )
                )
            px4 = probe_px4(
                args,
                args.px4_explicit if exposure else args.px4_legacy,
                capture,
                scenario,
            )
            if exposure:
                flow = read(capture / "flow.csv")
                selected = np.searchsorted(
                    flow["t_s"],
                    arrivals[arrivals["source"] == 1]["measurement_t_s"] - 1e-8,
                )
                px4["flow_api"] = verify_px4(px4["replay_log"], flow[selected])
                px4["replay_log"] = "\n".join(
                    line
                    for line in px4["replay_log"].splitlines()
                    if not line.startswith("FLOW_PACKET ")
                )
            scores.append(dict(name="px4", **px4))
            scores.append(
                dict(
                    name="ekf3", **probe(args, capture, arrivals, scenario, "baseline")
                )
            )
            for row in scores:
                if any(
                    row[window]["finite_fraction"] != 1
                    for window in ("flight", "outage_window", "after_return")
                ):
                    raise ValueError("Nonfinite scored native or ESKF state")
                row.update(
                    frequency=frequency,
                    seed=seed,
                    climb_height_m=height,
                    scenario=scenario,
                    explicit_exposure=exposure,
                    transport=transport,
                    arrival_trace_sha256=digest(capture / "arrivals.csv"),
                )
            (args.work / "scores.json").write_text(
                json.dumps(scores, indent=2, allow_nan=False) + "\n"
            )
            result["scores"].extend(scores)
            args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            print(f"Scored {label}", flush=True)
    result["source_sha256"] = {
        path.name: digest(path)
        for path in sorted(Path(__file__).parent.glob("*"))
        if path.is_file() and path.suffix in (".py", ".c", ".h")
    }
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "captures",
        "references",
        "harness",
        "horizon",
        "retrodiction",
        "px4-explicit",
        "px4-legacy",
        "px4-source",
        "px4-library",
        "ap-source",
        "ap-replay",
        "transport-trace",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
