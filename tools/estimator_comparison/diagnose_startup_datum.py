"""Separate startup translation from subsequent GPS-denied displacement error."""

import argparse
import json
from pathlib import Path
import subprocess

import numpy as np

from compare_delay import PROFILES
from compare_exposure import digest
from score import errors, read


BINARIES = {
    "horizon_control": "horizon_replay",
    "horizon_candidate": "horizon-datum",
    "retrodiction_control": "modelica_replay",
    "retrodiction_candidate": "retrodiction-datum",
}


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose fresh owned work and evidence paths")
    study = json.loads(args.study.read_text())
    reference = json.loads(args.reference.read_text())
    if not study.get("complete") or study["reference_sha256"] != digest(args.reference):
        raise ValueError("Require the completed frozen pressure calibration study")
    for name, filename in BINARIES.items():
        if digest(args.build / filename) != study["binary_sha256"][name]:
            raise ValueError("Frozen generated-C replay binary changed")
    args.work.mkdir(parents=True)
    result = dict(
        study_sha256=digest(args.study),
        reference_sha256=digest(args.reference),
        scope="Diagnostic decomposition only. Raw scores are unchanged. Translation-aligned displacement is not absolute position accuracy or a replacement ranking.",
        scores=[],
    )
    for previous in study["scores"]:
        if previous["scenario"] != "denied":
            continue
        source = (
            f"{previous['frequency']}_{previous['seed']}_{previous['climb_height_m']}m"
        )
        label = source + "_denied_explicit"
        capture = args.cases / label / "capture"
        for filename, expected in study["input_sha256"][source].items():
            if digest(capture / filename) != expected:
                raise ValueError("Frozen physical capture changed")
        if digest(capture / "arrivals.csv") != previous["arrival_trace_sha256"]:
            raise ValueError("Frozen packet arrivals changed")
        method = previous["name"] + "_" + previous["variant"]
        output = args.work / (label + "_" + method + ".csv")
        command = [
            str(args.build / BINARIES[method]),
            str(capture / "imu.csv"),
            str(capture / "modelica_input.csv"),
            str(output),
            "denied",
            "foh",
            "--stationary-until",
            "13",
            "--delays",
            *map(str, PROFILES["exposure"]),
            str(previous["seed"]),
            "--imu-noise-density",
            *map(str, reference["imu_noise_density"]),
            "--flow-packets",
            str(capture / "flow.csv"),
        ]
        subprocess.run(command, check=True, capture_output=True)
        if digest(output) != previous["output_sha256"]:
            raise ValueError("Diagnostic replay changed the frozen estimator output")
        estimate, truth = read(output), read(capture / "truth.csv")
        selected = (estimate["t_s"] >= 13) & (estimate["t_s"] < 60)
        position, velocity, attitude, yaw = errors(estimate[selected], truth)
        if not np.isfinite(np.column_stack((position, velocity, attitude, yaw))).all():
            raise ValueError("Diagnostic error contains a nonfinite row")
        start = position[0, :2]
        displacement = position[:, :2] - start
        raw_mse = float(np.mean(np.sum(position[:, :2] ** 2, axis=1)))
        relative_mse = float(np.mean(np.sum(displacement**2, axis=1)))
        offset_mse = float(start @ start)
        cross_term = float(2 * start @ np.mean(displacement, axis=0))
        if not np.isclose(raw_mse, offset_mse + relative_mse + cross_term):
            raise ValueError(
                "Startup/displacement decomposition does not reconstruct raw MSE"
            )
        if not np.isclose(
            np.sqrt(raw_mse), previous["flight"]["horizontal_position_rmse_m"]
        ):
            raise ValueError("Raw horizontal RMS changed")
        result["scores"].append(
            dict(
                **{
                    key: previous[key]
                    for key in (
                        "name",
                        "variant",
                        "seed",
                        "frequency",
                        "climb_height_m",
                    )
                },
                output_sha256=digest(output),
                first_flight_epoch_s=float(estimate["t_s"][selected][0]),
                initial_position_error_m=position[0].tolist(),
                initial_velocity_error_m_s=velocity[0].tolist(),
                initial_attitude_error_deg=float(attitude[0]),
                initial_yaw_error_deg=float(yaw[0]),
                horizontal_rmse_m=float(np.sqrt(raw_mse)),
                horizontal_displacement_rmse_m=float(np.sqrt(relative_mse)),
                startup_offset_squared_m2=offset_mse,
                displacement_mean_squared_m2=relative_mse,
                offset_displacement_cross_term_m2=cross_term,
            )
        )
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(label, method, np.sqrt(raw_mse), np.sqrt(relative_mse), flush=True)
    if len(result["scores"]) != 16:
        raise ValueError("Require all sixteen frozen GPS-denied replays")
    result["complete"] = True
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("study", "reference", "cases", "build", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
