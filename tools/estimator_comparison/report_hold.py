#!/usr/bin/env python3
"""Report matched hold-order and stationary-model effects without retuning."""

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np

from compare_hold import WINDOWS, digest


METRICS = (
    "horizontal_position_rmse_m",
    "vertical_position_rmse_m",
    "velocity_3d_rmse_m_s",
    "attitude_rmse_deg",
    "yaw_rmse_deg",
)
KEYS = (
    "frequency",
    "seed",
    "climb_height_m",
    "scenario",
    "method",
    "hold",
    "stationary_imu",
)


def validate(evidence):
    expected = set(
        (*mission, height, scenario, method, hold, rest)
        for mission in (
            (0.6, 7),
            (0.6, 19),
            (0.6, 41),
            (0.85, 101),
            (0.85, 307),
            (0.85, 911),
        )
        for height, scenario, method, hold, rest in itertools.product(
            (2, 4),
            ("gps", "denied", "transition"),
            ("horizon", "retrodiction"),
            ("foh", "zoh"),
            (False, True),
        )
    )
    scores = evidence["scores"]
    indexed = {tuple(row[key] for key in KEYS): row for row in scores}
    if set(indexed) != expected or len(scores) != len(expected):
        raise ValueError("Expected the complete 288-run paired matrix")
    if evidence["stationary_until_s"] != 13 or evidence["delay_profile"] != "nominal":
        raise ValueError("Unexpected rest interval or delay profile")
    for row in scores:
        if row["hold"] == "foh" and not row["stationary_imu"]:
            if row["frozen_control_identical"] is not True:
                raise ValueError("Frozen FOH control must reproduce byte for byte")
        if set(row["accuracy"]) != set(WINDOWS):
            raise ValueError("Missing scoring window")
        for window, (start, end) in WINDOWS.items():
            accuracy = row["accuracy"][window]
            if (
                (accuracy["start_s"], accuracy["end_s"]) != (start, end)
                or accuracy["finite_fraction"] != 1
                or accuracy["nonzero_step_status_rows"]
                or any(
                    not np.isfinite(accuracy[k]) or accuracy[k] <= 0 for k in METRICS
                )
            ):
                raise ValueError("Invalid accuracy result")
        for window in ("before_takeoff", "flight"):
            consistency = row["consistency"][window]
            if (
                not np.isfinite(consistency["mean_nees_15d"])
                or consistency["mean_nees_15d"] <= 0
                or consistency["max_covariance_asymmetry"] != 0
            ):
                raise ValueError("Invalid covariance result")
        timing = row["timing"]
        if timing["clock"] != "CLOCK_THREAD_CPUTIME_ID" or timing["imu_ticks"] != 48001:
            raise ValueError("Unexpected timing clock or replay length")
        if row["method"] == "horizon" and any(
            timing[k] for k in ("late", "overflow", "stale")
        ):
            raise ValueError("Lost horizon packet")
        partners = [
            indexed[(*tuple(row[k] for k in KEYS[:-2]), hold, rest)]
            for hold, rest in itertools.product(("foh", "zoh"), (False, True))
        ]
        for partner in partners:
            if any(
                timing[k] != partner["timing"][k]
                for k in ("transport_delivered", "transport_digest")
            ):
                raise ValueError("Paired sensor delivery differs")
    return indexed


def cpu(row):
    return sum(
        row["timing"][name]["total_ns"]
        for name in ("filter", "preintegration", "predictor", "queues")
    )


def change(candidate, baseline):
    return 100 * (candidate / baseline - 1)


def report(args):
    evidence = json.loads(args.input.read_text())
    indexed = validate(evidence)
    lines = [
        "ESKF hold-order and stationary-model ablation — 7 October 2026",
        "",
        "288 fresh generated-code replays: six seeds, two climb heights, three GPS scenarios,",
        "two delay methods, two hold orders and stationary IMU disabled/enabled.",
        "72 FOH/stationary-disabled controls reproduce frozen outputs byte for byte.",
        "800 Hz IMU, 100 Hz filter, nominal delays, unchanged noise tuning and aiding samples.",
        "Declared-rest zero-velocity correction is active until 13 s in every run.",
        "Stationary enabled replaces inertial prediction during rest and adds IMU bias/gravity observations.",
        "Accuracy uses the published output; covariance uses the matching filter epoch.",
        "",
        "Values below are median paired percentage changes. Negative means smaller error or CPU time.",
        "Each row contains 36 pairs. Flight is 13–59.7 s; preflight is 10–13 s.",
        "",
        "FOH relative to ZOH",
        "method         stationary  horiz%   vert%    vel%    att%    yaw%    CPU%  horiz wins",
    ]
    pairs = []
    for effect, candidates in (
        ("foh_vs_zoh", itertools.product(("horizon", "retrodiction"), (False, True))),
        (
            "stationary_vs_normal",
            itertools.product(("horizon", "retrodiction"), ("foh", "zoh")),
        ),
    ):
        if effect == "stationary_vs_normal":
            lines += [
                "",
                "Stationary IMU relative to normal inertial prediction",
                "method         hold        horiz%   vert%    vel%    att%    yaw%    CPU%  pre-att% pre-NEES%",
            ]
        for method, setting in candidates:
            group = []
            for key, candidate in indexed.items():
                if candidate["method"] != method:
                    continue
                if effect == "foh_vs_zoh":
                    if (
                        candidate["hold"] != "foh"
                        or candidate["stationary_imu"] != setting
                    ):
                        continue
                    baseline_key = (*key[:-2], "zoh", key[-1])
                else:
                    if not candidate["stationary_imu"] or candidate["hold"] != setting:
                        continue
                    baseline_key = (*key[:-1], False)
                baseline = indexed[baseline_key]
                values = [
                    change(
                        candidate["accuracy"]["flight"][k],
                        baseline["accuracy"]["flight"][k],
                    )
                    for k in METRICS
                ]
                values += [change(cpu(candidate), cpu(baseline))]
                values += [
                    change(
                        candidate["accuracy"]["before_takeoff"]["attitude_rmse_deg"],
                        baseline["accuracy"]["before_takeoff"]["attitude_rmse_deg"],
                    )
                ]
                values += [
                    change(
                        candidate["consistency"]["before_takeoff"]["mean_nees_15d"],
                        baseline["consistency"]["before_takeoff"]["mean_nees_15d"],
                    )
                ]
                group.append(values)
                for window in WINDOWS:
                    row = dict(
                        effect=effect, **{k: candidate[k] for k in KEYS}, window=window
                    )
                    for metric in METRICS:
                        row[metric + "_candidate"] = candidate["accuracy"][window][
                            metric
                        ]
                        row[metric + "_baseline"] = baseline["accuracy"][window][metric]
                        row[metric + "_change_percent"] = change(
                            row[metric + "_candidate"], row[metric + "_baseline"]
                        )
                    row["cpu_change_percent"] = values[5]
                    if window in ("before_takeoff", "flight"):
                        for label, source in (
                            ("candidate", candidate),
                            ("baseline", baseline),
                        ):
                            row["mean_nees_15d_" + label] = source["consistency"][
                                window
                            ]["mean_nees_15d"]
                    pairs.append(row)
            medians = np.median(group, axis=0)
            prefix = f"{method:14} {str(setting):10}"
            line = prefix + " ".join(f"{value:7.2f}" for value in medians[:6])
            line += (
                f"  {np.count_nonzero(np.array(group)[:, 0] < 0):2d}/36"
                if effect == "foh_vs_zoh"
                else " " + " ".join(f"{value:8.2f}" for value in medians[6:])
            )
            lines.append(line)
    lines += [
        "",
        "FOH relative to ZOH by GPS scenario, stationary IMU disabled",
        "method         scenario     horiz%   vert%    vel%    att%    yaw%    CPU%",
    ]
    for method, scenario in itertools.product(
        ("horizon", "retrodiction"), ("gps", "denied", "transition")
    ):
        selected = [
            row
            for row in pairs
            if row["effect"] == "foh_vs_zoh"
            and row["method"] == method
            and row["scenario"] == scenario
            and not row["stationary_imu"]
            and row["window"] == "flight"
        ]
        medians = [
            np.median([row[metric + "_change_percent"] for row in selected])
            for metric in METRICS
        ] + [np.median([row["cpu_change_percent"] for row in selected])]
        lines.append(
            f"{method:14} {scenario:10}"
            + " ".join(f"{value:7.2f}" for value in medians)
        )
    lines += [
        "",
        "Preflight mean 15D NEES, median across 36 missions (smaller alone does not prove calibration)",
        "method         hold        stationary off   stationary on",
    ]
    for method, hold in itertools.product(("horizon", "retrodiction"), ("foh", "zoh")):
        values = [
            np.median(
                [
                    row["consistency"]["before_takeoff"]["mean_nees_15d"]
                    for row in indexed.values()
                    if row["method"] == method
                    and row["hold"] == hold
                    and row["stationary_imu"] == rest
                ]
            )
            for rest in (False, True)
        ]
        lines.append(f"{method:14} {hold:10} {values[0]:14.3f} {values[1]:15.3f}")
    reference_states = {method: set() for method in ("horizon", "retrodiction")}
    for name, checksum in evidence["reference_sha256"].items():
        path = args.input.parent / name
        if digest(path) != checksum:
            raise ValueError("Historical memory reference changed")
        for row in json.loads(path.read_text())["scores"]:
            if row["name"] in reference_states:
                reference_states[row["name"]].add(row["timing"]["state_bytes"])
    lines += [
        "",
        "Generated state RAM, including the optional stationary code even when disabled",
    ]
    for method, previous in reference_states.items():
        current = {
            row["timing"]["state_bytes"]
            for row in indexed.values()
            if row["method"] == method
        }
        if len(previous) != 1 or len(current) != 1:
            raise ValueError("Mixed generated state sizes")
        old, new = next(iter(previous)), next(iter(current))
        lines.append(
            f"{method}: {old} -> {new} bytes ({new - old:+d}); aiding/predictor buffers and transport unchanged."
        )
    lines += [
        "",
        "Limits",
        *evidence["limitations"],
        "",
        "CPU includes filter, preintegration, predictor and aiding queue calls; CSV I/O is excluded.",
        "One timing observation per run; medians are descriptive and do not establish target speedups.",
        "The CSV includes all paired cases and windows, including regressions.",
    ]
    args.output.with_suffix(".txt").write_text("\n".join(lines) + "\n")
    with args.output.with_suffix(".csv").open("w") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(dict.fromkeys(k for row in pairs for k in row))
        )
        writer.writeheader()
        writer.writerows(pairs)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    report(parser.parse_args())
