#!/usr/bin/env python3
"""Report square-root accuracy, numerical validity and deployment costs."""

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np

from compare_square_root import digest


KEYS = ("name", "frequency", "seed", "climb_height_m", "scenario", "explicit_exposure")
METRICS = (
    "horizontal_position_rmse_m",
    "vertical_position_rmse_m",
    "velocity_3d_rmse_m_s",
    "attitude_rmse_deg",
    "yaw_rmse_deg",
)
WINDOWS = {
    "flight": (13, 60, 4700),
    "outage_window": (25, 40, 1500),
    "after_return": (40, 60, 2000),
}


def key(row):
    return tuple(row[name] for name in KEYS)


def cpu(timing):
    return sum(
        timing[name]["total_ns"]
        for name in ("filter", "preintegration", "predictor", "queues")
    )


def validate(evidence, reference):
    expected = set(
        (method, frequency, seed, height, scenario, exposure)
        for method, (frequency, seed), height, scenario, exposure in itertools.product(
            ("horizon", "retrodiction"),
            ((0.6, 7), (0.85, 101)),
            (2, 4),
            ("gps", "denied", "transition"),
            (False, True),
        )
    )
    indexed = {key(row): row for row in evidence["scores"]}
    controls = {key(row): row for row in evidence["controls"]}
    baseline = {
        key(row): row
        for row in reference["scores"]
        if row["name"] in ("horizon", "retrodiction")
    }
    if any(
        set(group) != expected or len(rows) != 48
        for group, rows in (
            (indexed, evidence["scores"]),
            (controls, evidence["controls"]),
            (
                baseline,
                [
                    row
                    for row in reference["scores"]
                    if row["name"] in ("horizon", "retrodiction")
                ],
            ),
        )
    ):
        raise ValueError("Expected all 48 distinct controls and square-root cases")
    for identifier, row in indexed.items():
        control, prior = controls[identifier], baseline[identifier]
        if (
            control["identical"] is not True
            or control["output_sha256"] != prior["output_sha256"]
        ):
            raise ValueError("Frozen control output changed")
        if row["control_timing"] != control["timing"]:
            raise ValueError("Mismatched paired timing")
        for timing in (row["timing"], control["timing"]):
            if (
                timing["clock"] != "CLOCK_THREAD_CPUTIME_ID"
                or timing["imu_ticks"] != 48001
                or timing["stationary_until_s"] != 13
            ):
                raise ValueError("Unexpected replay timing")
            if timing["filter"]["total_ns"] <= 0 or cpu(timing) <= 0:
                raise ValueError("Invalid CPU timing")
            if any(
                timing[name] != prior["timing"][name]
                for name in ("buffer_bytes", "transport_bytes")
            ):
                raise ValueError("Buffer storage changed")
            if any(timing[name] != value for name, value in prior["transport"].items()):
                raise ValueError("Paired sensor delivery changed")
            if row["name"] == "horizon" and any(
                timing[name] for name in ("late", "stale", "overflow")
            ):
                raise ValueError("Horizon packet lost")
        for window, (start, end, rows) in WINDOWS.items():
            accuracy = row[window]
            if (
                (accuracy["start_s"], accuracy["end_s"], accuracy["rows"])
                != (start, end, rows)
                or any(
                    accuracy[name] != 1
                    for name in (
                        "finite_fraction",
                        "row_coverage",
                        "position_valid_fraction",
                        "attitude_valid_fraction",
                    )
                )
                or accuracy["nonzero_step_status_rows"]
                or any(
                    not np.isfinite(accuracy[name]) or accuracy[name] <= 0
                    for name in METRICS
                )
            ):
                raise ValueError("Invalid accuracy evidence")
        for window, rows in (("before_takeoff", 300), ("flight", 4670)):
            diagnostics = row["consistency"][window]
            if (
                diagnostics["covariance_representation"] != "square_root"
                or diagnostics["rows"] != rows
                or diagnostics["max_covariance_asymmetry"] != 0
                or not np.isfinite(diagnostics["mean_nees_15d"])
                or diagnostics["mean_nees_15d"] <= 0
                or not 0
                <= diagnostics["max_covariance_root_reconstruction_error"]
                <= 1e-5
                or not 0 <= diagnostics["dense_covariance_non_pd_samples"] <= rows
            ):
                raise ValueError("Invalid square-root diagnostics")
    return indexed, baseline


def report(args):
    evidence, reference = [
        json.loads(path.read_text()) for path in (args.input, args.reference)
    ]
    if evidence["reference_sha256"] != digest(args.reference):
        raise ValueError("Frozen reference changed")
    indexed, baseline = validate(evidence, reference)
    previous_root = None
    if "root_reference_sha256" in evidence:
        if (
            not args.root_reference
            or digest(args.root_reference) != evidence["root_reference_sha256"]
        ):
            raise ValueError("Previous root reference is missing or changed")
        previous = json.loads(args.root_reference.read_text())
        previous_root, _ = validate(previous, reference)
        for identifier, row in indexed.items():
            prior_root = previous_root[identifier]
            if (
                row.get("root_control_identical") is not True
                or row["output_sha256"] != prior_root["output_sha256"]
            ):
                raise ValueError("Previous root output changed")
            if row.get("root_covariance_control_identical") != (
                row["covariance_sha256"] == prior_root["covariance_sha256"]
            ):
                raise ValueError("Incorrect covariance preservation claim")
    pairs = []
    for identifier, row in indexed.items():
        prior = baseline[identifier]
        for window in WINDOWS:
            paired = dict(**{name: row[name] for name in KEYS}, window=window)
            for metric in METRICS:
                paired[metric + "_baseline"] = prior[window][metric]
                paired[metric + "_root"] = row[window][metric]
                paired[metric + "_change_percent"] = 100 * (
                    row[window][metric] / prior[window][metric] - 1
                )
            paired["filter_cpu_ratio"] = (
                row["timing"]["filter"]["total_ns"]
                / row["control_timing"]["filter"]["total_ns"]
            )
            paired["total_cpu_ratio"] = cpu(row["timing"]) / cpu(row["control_timing"])
            pairs.append(paired)
    lines = [
        "ESKF square-root prototype — 7 October 2026",
        "",
        "96 fresh replays: 48 exact frozen-output controls and 48 root candidates.",
        "Two seeds, two heights, GPS/denied/loss-and-return, legacy/explicit optical flow,",
        "fusion horizon and retrodiction; unchanged captures, tuning and sensor delivery.",
        "FOH, geometric alignment and vector magnetometer; stationary IMU disabled.",
        "Accuracy is scored at publication time; horizon covariance at fusion time.",
        "",
        "Median paired percentage changes in flight RMSE; negative is better.",
        "Each row contains 12 pairs. CPU ratios use the corresponding fresh controls.",
        "method         exposure  horiz%   vert%    vel%    att%    yaw% filterCPU totalCPU",
    ]
    for method, exposure in itertools.product(
        ("horizon", "retrodiction"), (False, True)
    ):
        group = [
            row
            for row in pairs
            if row["name"] == method
            and row["explicit_exposure"] == exposure
            and row["window"] == "flight"
        ]
        values = [
            np.median([row[metric + "_change_percent"] for row in group])
            for metric in METRICS
        ]
        ratios = [
            np.median([row[name] for row in group])
            for name in ("filter_cpu_ratio", "total_cpu_ratio")
        ]
        lines.append(
            f"{method:14} {str(exposure):8} "
            + " ".join(f"{value:7.2f}" for value in values)
            + " "
            + " ".join(f"{value:8.2f}" for value in ratios)
        )
    lines += [
        "",
        "Explicit exposure by GPS scenario (four pairs per row)",
        "method         scenario     horiz%   vert%    vel%    att%    yaw%",
    ]
    for method, scenario in itertools.product(
        ("horizon", "retrodiction"), ("gps", "denied", "transition")
    ):
        group = [
            row
            for row in pairs
            if row["name"] == method
            and row["explicit_exposure"]
            and row["scenario"] == scenario
            and row["window"] == "flight"
        ]
        values = [
            np.median([row[metric + "_change_percent"] for row in group])
            for metric in METRICS
        ]
        lines.append(
            f"{method:14} {scenario:10} "
            + " ".join(f"{value:7.2f}" for value in values)
        )
    lines += ["", "Covariance validity and calibration"]
    samples = sum(
        row["consistency"][window]["rows"]
        for row in indexed.values()
        for window in ("before_takeoff", "flight")
    )
    dense_failures = sum(
        row["consistency"][window]["dense_covariance_non_pd_samples"]
        for row in indexed.values()
        for window in ("before_takeoff", "flight")
    )
    lines += [
        f"All {samples:,} scored root factors were finite, triangular, positive on the diagonal,",
        f"and consistent with the dense diagnostic; dense rounding failures: {dense_failures}.",
        "NEES uses the authoritative root. No failed samples are removed or jittered.",
    ]
    for method, exposure in itertools.product(
        ("horizon", "retrodiction"), (False, True)
    ):
        group = [
            row
            for row in indexed.values()
            if row["name"] == method and row["explicit_exposure"] == exposure
        ]
        nees = np.median(
            [row["consistency"]["flight"]["mean_nees_15d"] for row in group]
        )
        lines.append(
            f"{method}, explicit={exposure}: median flight mean 15D NEES {nees:.3f}."
        )
    lines += ["", "Previously failing legacy denied retrodiction cases"]
    for row in indexed.values():
        prior = baseline[key(row)]
        if not prior["consistency"]["flight"].get("valid", True):
            lines.append(
                f"seed {row['seed']}, {row['climb_height_m']} m: prior {prior['consistency']['flight']['failed_samples']} non-PD samples; root valid throughout; horizontal RMSE {prior['flight'][METRICS[0]]:.3f} -> {row['flight'][METRICS[0]]:.3f} m."
            )
    worst = max(indexed.values(), key=lambda row: row["flight"][METRICS[0]])
    lines += [
        f"Worst root horizontal RMSE: {worst['flight'][METRICS[0]]:.3f} m ({key(worst)}).",
        "",
        "Generated state storage, including optional code when disabled",
    ]
    for method in ("horizon", "retrodiction"):
        old = {
            row["timing"]["state_bytes"]
            for row in baseline.values()
            if row["name"] == method
        }
        new = {
            row["timing"]["state_bytes"]
            for row in indexed.values()
            if row["name"] == method
        }
        if len(old) != 1 or len(new) != 1:
            raise ValueError("Inconsistent state storage")
        old, new = old.pop(), new.pop()
        lines.append(
            f"{method}: {old} -> {new} bytes (+{new - old}); buffers/transport unchanged."
        )
    native = {
        key(row): row for row in reference["scores"] if row["name"] in ("px4", "ekf3")
    }
    if len(native) != 48:
        raise ValueError("Missing frozen native comparisons")
    lines += [
        "",
        "Explicit-exposure root errors below BOTH frozen native estimators (12 cases per method)",
        "method         horizontal vertical velocity attitude yaw",
    ]
    for method in ("horizon", "retrodiction"):
        group = [
            row
            for row in indexed.values()
            if row["name"] == method and row["explicit_exposure"]
        ]
        wins = [
            sum(
                all(
                    row["flight"][metric]
                    < native[(name, *key(row)[1:])]["flight"][metric]
                    for name in ("px4", "ekf3")
                )
                for row in group
            )
            for metric in METRICS
        ]
        lines.append(f"{method:14} " + " ".join(f"{count:7}/12" for count in wins))
    lines += [
        "",
        "Limits and next work",
        "Opt-in useSquareRootCovariance=false remains the default. This is a prototype,",
        "with a large CPU cost and storage overhead even when the feature is disabled.",
        "Numerical stability does not remove legacy flow drift or prove calibrated errors.",
        "Dense P is retained for diagnostics and auxiliary terrain/barometer logic.",
        "State record constructors now require the root and representation flag.",
        "The same approximate Simpson process-noise model is preserved; not exact FOH noise.",
        "Extreme invalid/nonfinite inputs, recovery/reseed and IMU-dropout behavior still need",
        "dedicated numerical qualification beyond these normal-flight replays.",
        "Native results are frozen from the identical exposure captures, not rerun here.",
        "Native range/height/source policies and measurement-noise floors remain unequal.",
        "Synthetic captures, correlated NEES samples and host CPU timings do not establish",
        "flight robustness, independent Monte Carlo calibration, target WCET or universal superiority.",
        "The CSV retains every pair and scoring window, including regressions.",
    ]
    if previous_root:
        lines += [
            "",
            "QR optimization relative to the previous root prototype",
            "Published outputs are byte-identical in all 48 cases.",
            f"Covariance diagnostics are byte-identical in {sum(row['root_covariance_control_identical'] for row in indexed.values())}/48 cases.",
            "Median paired CPU changes; negative is faster. Historical/fresh host timings.",
            "method         exposure filterCPU% totalCPU%",
        ]
        for method, exposure in itertools.product(
            ("horizon", "retrodiction"), (False, True)
        ):
            group = [
                row
                for row in indexed.values()
                if row["name"] == method and row["explicit_exposure"] == exposure
            ]
            changes = [
                np.median(
                    [
                        100
                        * (
                            measure(row["timing"])
                            / measure(previous_root[key(row)]["timing"])
                            - 1
                        )
                        for row in group
                    ]
                )
                for measure in (lambda timing: timing["filter"]["total_ns"], cpu)
            ]
            lines.append(
                f"{method:14} {str(exposure):8} "
                + " ".join(f"{value:9.2f}" for value in changes)
            )
        lines.append(
            "Generated whole-estimator state storage is unchanged by this optimization."
        )
        lines += [
            "CPU increases apply to root-enabled runs; the storage overhead applies in both modes.",
            "",
            "Fusion horizon relative to retrodiction, explicit exposure (12 paired cases)",
            "Metric                           median change% horizon wins",
        ]
        horizon = [
            row
            for row in indexed.values()
            if row["name"] == "horizon" and row["explicit_exposure"]
        ]
        for metric in METRICS:
            changes = [
                100
                * (
                    row["flight"][metric]
                    / indexed[("retrodiction", *key(row)[1:])]["flight"][metric]
                    - 1
                )
                for row in horizon
            ]
            lines.append(
                f"{metric:32} {np.median(changes):12.2f} {sum(value < 0 for value in changes):5}/12"
            )
        cpu_changes = [
            100
            * (
                cpu(row["timing"])
                / cpu(indexed[("retrodiction", *key(row)[1:])]["timing"])
                - 1
            )
            for row in horizon
        ]
        lines.append(
            f"total_component_cpu              {np.median(cpu_changes):12.2f} {sum(value < 0 for value in cpu_changes):5}/12"
        )
    args.text.write_text("\n".join(lines) + "\n")
    with args.csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(pairs[0]))
        writer.writeheader()
        writer.writerows(pairs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input", "reference", "text", "csv"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--root-reference", type=Path)
    report(parser.parse_args())
