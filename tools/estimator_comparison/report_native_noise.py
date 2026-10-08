#!/usr/bin/env python3
"""Keep default and calibrated native results beside the frozen ESKF candidate."""

import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
from statistics import median

from native_noise import native_noise, PREDICTION_PERIOD_S


METRICS = (
    "horizontal_position_rmse_m",
    "vertical_position_rmse_m",
    "velocity_3d_rmse_m_s",
    "yaw_rmse_deg",
)
NAMES = {
    "horizon": "ESKF horizon",
    "retrodiction": "ESKF retrodiction",
    "px4_default": "PX4 default IMU noise",
    "px4_calibrated": "PX4 calibrated IMU noise",
    "ekf3_default": "EKF3 default IMU noise",
    "ekf3_calibrated": "EKF3 calibrated IMU noise",
}


def report(args):
    paths = [args.output.with_suffix(suffix) for suffix in (".txt", ".csv")]
    if any(path.exists() for path in paths):
        raise ValueError("Choose new report paths")
    points = []
    configuration = None
    for dataset, prior_path, new_path in (
        ("original", args.original_reference, args.original),
        ("held-out", args.held_out_reference, args.held_out),
    ):
        prior = json.loads(prior_path.read_text())
        calibrated = json.loads(new_path.read_text())
        if (
            calibrated["reference_sha256"][prior_path.name]
            != hashlib.sha256(prior_path.read_bytes()).hexdigest()
            or calibrated["native_source_pins"] != prior["native_source_pins"]
            or calibrated["seeds"] != prior["seeds"]
            or calibrated["frequency"] != prior["frequency"]
        ):
            raise ValueError("Native results do not match the supplied reference")
        profile = (
            calibrated["imu_noise_density"],
            calibrated["rate_noise_std"],
            calibrated["nominal_prediction_period_s"],
            calibrated["native_source_pins"],
        )
        if configuration is not None and configuration != profile:
            raise ValueError("Native configuration differs between motion sets")
        configuration = profile
        if calibrated["imu_noise_density"] != prior["noise_density"]:
            raise ValueError("ESKF and native white-noise calibrations differ")
        if (
            calibrated["rate_noise_std"] != native_noise(prior["noise_density"])
            or calibrated["nominal_prediction_period_s"] != PREDICTION_PERIOD_S
        ):
            raise ValueError("Native rate mapping differs from the frozen calibration")
        if (
            calibrated["delay_profile"] != "nominal"
            or calibrated["takeoff_expectation_clear_s"] != 18
            or calibrated["delay_profile"] != prior["delay_profile"]
            or calibrated["takeoff_expectation_clear_s"]
            != prior["takeoff_expectation_clear_s"]
            or calibrated["core_binary_sha256"]
            != {
                "px4": prior["binary_sha256"]["px4_core"],
                "ekf3": prior["binary_sha256"]["ekf3"],
            }
        ):
            raise ValueError("Unexpected replay configuration")
        if len(set(prior["seeds"])) != 3:
            raise ValueError("Expected three distinct seeds per motion set")
        for rows, names, seeds in (
            (
                prior["scores"],
                ("horizon", "retrodiction", "px4", "ekf3"),
                prior["seeds"],
            ),
            (calibrated["scores"], ("px4", "ekf3"), prior["seeds"]),
            (calibrated["controls"], ("px4", "ekf3"), prior["seeds"][:1]),
        ):
            expected = set(
                itertools.product(seeds, (2, 4), ("gps", "denied", "transition"), names)
            )
            actual = {
                (
                    int(row["tag"].rsplit("_", 1)[1]),
                    row["climb_height_m"],
                    row["scenario"],
                    row["name"],
                )
                for row in rows
            }
            if expected != actual or len(rows) != len(expected):
                raise ValueError("Incomplete comparison matrix")
            if any(
                row["tag"] != f"{prior['frequency']}_{row['seed']}"
                for row in rows
                if "seed" in row
            ):
                raise ValueError("Capture tags and declared seeds differ")
        if not all(r["identical"] for r in calibrated["controls"]):
            raise ValueError("Native default controls are incomplete or changed")
        reference_rows = {
            (row["tag"], row["climb_height_m"], row["scenario"], row["name"]): row
            for row in prior["scores"]
        }
        for row in calibrated["controls"]:
            reference = reference_rows[
                row["tag"], row["climb_height_m"], row["scenario"], row["name"]
            ]
            if row["output_sha256"] != reference["output_sha256"]:
                raise ValueError("Native default control hash differs from reference")
        for row in calibrated["scores"]:
            reference = reference_rows[
                row["tag"], row["climb_height_m"], row["scenario"], row["name"]
            ]
            if row["transport"] != reference["transport"]:
                raise ValueError("Native and ESKF packet delivery differs")
        for source, rows in (
            ("default", prior["scores"]),
            ("calibrated", calibrated["scores"]),
        ):
            for row in rows:
                name = (
                    row["name"] + "_" + source
                    if row["name"] in ("px4", "ekf3")
                    else row["name"]
                )
                for window in ("flight", "outage_window", "after_return"):
                    if row[window]["finite_fraction"] != 1 or not all(
                        math.isfinite(row[window][key]) and row[window][key] >= 0
                        for key in METRICS
                    ):
                        raise ValueError("Nonfinite result")
                    points.append(
                        dict(
                            dataset=dataset,
                            seed=row["seed"],
                            height_m=row["climb_height_m"],
                            scenario=row["scenario"],
                            estimator=name,
                            window=window,
                            **{key: row[window][key] for key in METRICS},
                        )
                    )
    with paths[1].open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(points[0]))
        writer.writeheader()
        writer.writerows(points)
    text = """Frozen IMU calibration applied to the native estimator cores
===========================================================

This ablation closes the earlier white-IMU-noise tuning asymmetry. It applies
the same stationary-only seed-7 calibration already used by the ESKF candidate
to native PX4 EKF2 and ArduPilot EKF3. Native bias random walks, initial priors,
measurement noise floors, source policies and output dynamics are unchanged.
It therefore matches one process-noise component, not every uncertainty or
algorithm detail. Sensor averaging/delay and internal ingestion-rate fidelity
gaps from native-configuration-audit remain unresolved.

The density-to-rate mapping is sigma_rate = density / sqrt(prediction_dt),
because the native covariance prediction adds sigma_rate^2 * dt^2 while ESKF
integrates density^2 * dt. Periods are frozen at the observed settled values:
PX4 10 ms and EKF3 12.5 ms. Runtime PX4 parameter getters and EKF3 logged PARM
values confirm the settings. Every new replay checks flight prediction timing
against the mapped interval; startup intervals may differ. This is an equivalent
leading white-noise variance mapping, not an exact match of complete discrete
FOH/coning/sculling noise propagation or fault-triggered variance inflation.

The actual generated PX4 covariance helper independently verifies PSD * dt at
5/10/12.5/20 ms. EKF3's explicit dt-squared formula and runtime clamping are
checked in pinned source, but its isolated covariance helper is not executed.
No native core is modified. EKF3's mapped accelerometer value (about 0.00949)
is slightly below its documented recommended minimum (0.01), although it is
inside the actual runtime clamp and the logged parameter confirms acceptance.
These controlled synthetic low-noise parameters are not a flight tuning
recommendation.

"""
    text += f"Frozen densities (gyro, accel): {configuration[0]}\n"
    text += f"Native rate standard deviations: {configuration[1]}\n\n"
    text += """72 new calibrated native replays cover six seeds, two heights and all three
GPS scenarios, with high-rate aiding and nominal delays. Another 24 native
replays verify omitted calibration preserves the earlier output hashes for
the two first-seed captures across both heights/scenarios. The 72 earlier
ESKF scores and 72 default-native scores are reused, not recomputed or retuned.
The 2 m mission keeps EKF3 heading-only; the 4 m mission permits native 3-axis
activation. Takeoff expectation clears at 18 s for both heights.

The sibling CSV includes all 648 seed/window rows. Tables use medians of three
seeds, with horizontal m / vertical m / 3D velocity m/s / yaw degrees. GPS and
denied use the full flight window; transition uses 25 <= t < 40 s. Invalid
finite outputs remain scored. No per-scenario best tuning is selected, and
these results do not establish intrinsic or universal estimator superiority.

"""
    for dataset, height, scenario in itertools.product(
        ("original", "held-out"), (2, 4), ("gps", "denied", "transition")
    ):
        window = "outage_window" if scenario == "transition" else "flight"
        text += f"{dataset}, climb {height} m, {scenario} ({window})\n"
        for name, label in NAMES.items():
            rows = [
                r
                for r in points
                if r["dataset"] == dataset
                and r["height_m"] == height
                and r["scenario"] == scenario
                and r["window"] == window
                and r["estimator"] == name
            ]
            if len(rows) != 3:
                raise ValueError("Missing seed group")
            text += (
                f"  {label:27s}"
                + " ".join(f"{median(r[k] for r in rows):10.5f}" for k in METRICS)
                + "\n"
            )
        text += "\n"
    text += """Remaining consistency and fidelity work
---------------------------------------

The seed-41 ESKF retrodiction failure remains. The separate startup timeline
places its strongest overconfidence before takeoff: attitude/bias errors grow
at rest and then recover during maneuvering. This suggests investigating the
stationary tilt/accelerometer-bias ambiguity and its covariance treatment; it
does not yet prove a root cause. There is no validated Modelica algorithm fix
in this stage. Horizon/native covariance and target CPU/memory comparisons,
production sensor/replay validation and the full delay/rate/disturbance matrix
remain open. Correcting native white-noise tuning alone cannot resolve them.

Reproduce with compare_native_noise.py --help, the frozen calibration and
eskf-altitude reference JSONs, unchanged pinned cores and audited external
adapters. Supply original/higher capture directories with hashes matching the
reference; compare_altitude.py can regenerate the higher captures. Use new
owned work/output paths under HOME/scratch when writable. report_native_noise.py
regenerates this text and CSV from both reference/current motion sets. The
manifest retains source, binary, input-reference and output/control hashes.
"""
    paths[0].write_text(text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "original-reference",
        "held-out-reference",
        "original",
        "held-out",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
