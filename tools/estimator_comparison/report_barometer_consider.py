"""Report frozen pressure-consider ablations without hiding regressions."""

import argparse
import csv
import json
from pathlib import Path
from statistics import median

from compare_exposure import digest
from report_estimator_theory import METRICS


FIELDS = ("seed", "frequency", "climb_height_m", "scenario")


def case(row):
    return tuple(row[field] for field in FIELDS)


def report(args):
    native = json.loads(args.native.read_text())
    covariance = json.loads(args.native_covariance.read_text())
    if (
        covariance["reference_sha256"] != digest(args.native)
        or len(covariance["scores"]) != 24
    ):
        raise ValueError("Native covariance reference is incomplete or changed")
    observed = {(row["name"], case(row)): row for row in covariance["scores"]}
    groups = {}
    for row in native["scores"]:
        if row["name"] in ("px4", "ekf3") and row["explicit_exposure"]:
            observer = observed[row["name"], case(row)]
            if (
                not observer["output_identical"]
                or observer["output_sha256"] != row["output_sha256"]
            ):
                raise ValueError("Native covariance observation changed the output")
            groups.setdefault(row["name"], {})[case(row)] = dict(
                **row, consistency=observer["consistency"]["windows"]
            )
    source_hashes = dict(
        native=digest(args.native), native_covariance=digest(args.native_covariance)
    )
    for startup in ("default", "rest"):
        path = getattr(args, startup + "_study")
        study = json.loads(path.read_text())
        source_hashes[startup] = digest(path)
        if (
            not study.get("complete")
            or len(study["scores"]) != 48
            or study["reference_sha256"] != digest(args.native)
        ):
            raise ValueError("Pressure-consider study is incomplete or changed")
        for row in study["scores"]:
            if row["variant"] == "control" and not row["control_output_identical"]:
                raise ValueError("Default behavior changed")
            name = "_".join((row["name"], startup, row["variant"]))
            group = groups.setdefault(name, {})
            if case(row) in group:
                raise ValueError("Duplicate pressure-consider condition")
            group[case(row)] = row
    conditions = set(groups["px4"])
    if (
        len(groups) != 10
        or len(conditions) != 12
        or any(set(group) != conditions for group in groups.values())
    ):
        raise ValueError("Require all ten methods on the same twelve conditions")
    rows, paired, wins, changes = [], [], [], []
    for name, group in groups.items():
        for window in ("flight", "outage_window", "after_return"):
            consistency_window = "outage" if window == "outage_window" else window
            counts = dict.fromkeys(METRICS, 0)
            for condition in sorted(conditions):
                row = group[condition]
                statistics = row["consistency"][consistency_window]
                counts = {
                    metric: counts[metric]
                    + int(
                        row[window][metric]
                        < min(
                            groups[baseline][condition][window][metric]
                            for baseline in ("px4", "ekf3")
                        )
                    )
                    for metric in METRICS
                }
                rows.append(
                    dict(
                        estimator=name,
                        **dict(zip(FIELDS, condition, strict=True)),
                        window=window,
                        covariance_valid=statistics.get("valid", True),
                        mean_nees_15d=statistics.get("mean_nees_15d"),
                        **{metric: row[window][metric] for metric in METRICS},
                    )
                )
            if name not in ("px4", "ekf3"):
                wins.append(dict(estimator=name, window=window, **counts))
    for startup in ("default", "rest"):
        for method in ("horizon", "retrodiction"):
            name = method + "_" + startup
            for metric in METRICS:
                ratios = []
                for condition in sorted(conditions):
                    control = groups[name + "_control"][condition]["flight"][metric]
                    candidate = groups[name + "_candidate"][condition]["flight"][metric]
                    ratios.append(candidate / control)
                    paired.append(
                        dict(
                            estimator=name,
                            **dict(zip(FIELDS, condition, strict=True)),
                            metric=metric,
                            control=control,
                            candidate=candidate,
                            change_percent=100 * (candidate / control - 1),
                        )
                    )
                changes.append(
                    dict(
                        estimator=name,
                        metric=metric,
                        improved=sum(ratio < 1 for ratio in ratios),
                        conditions=12,
                        minimum_change_percent=100 * (min(ratios) - 1),
                        maximum_change_percent=100 * (max(ratios) - 1),
                    )
                )
    args.output.mkdir(parents=True, exist_ok=True)
    for filename, records in (("comparison.csv", rows), ("paired-flight.csv", paired)):
        with (args.output / filename).open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=records[0], lineterminator="\n")
            writer.writeheader()
            writer.writerows(records)
    summary = dict(
        controls_byte_identical=48,
        candidate_replays=48,
        conditions_per_method=12,
        source_sha256=source_hashes,
        paired_changes=changes,
        lower_rmse_than_both_native=wins,
        undefined_nees_windows=[
            {field: row[field] for field in ("estimator", *FIELDS, "window")}
            for row in rows
            if not row["covariance_valid"]
        ],
    )
    costs = []
    for name, group in groups.items():
        if name in ("px4", "ekf3"):
            continue
        timings = [row["timing"] for row in group.values()]
        costs.append(
            dict(
                estimator=name,
                median_filter_cpu_us_per_update=median(
                    row["filter"]["total_ns"] / row["filter"]["calls"] / 1000
                    for row in timings
                ),
                median_pipeline_cpu_us_per_imu_packet=median(
                    sum(
                        row[stage]["total_ns"]
                        for stage in ("filter", "preintegration", "predictor", "queues")
                    )
                    / row["imu_ticks"]
                    / 1000
                    for row in timings
                ),
                generated_state_bytes=sorted({row["state_bytes"] for row in timings}),
            )
        )
    summary["cpu_cost"] = costs
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    text = [
        "Pressure datum consider state: frozen matched-data ablation",
        "",
        "48 fresh controls reproduce frozen state CSVs byte for byte; 48 candidates.",
        "Each method covers GPS, GPS-denied and loss/return, two coupled seed/motion choices and two heights.",
        "Default/rest indicate the startup-calibration policy; candidate enables shared-datum cross covariance.",
        "The consider datum mean is held during navigation corrections. This is not a fully estimated bias state.",
        "Effective native R/Q, priors and height/magnetic policies remain unequal.",
        "Native 15D NEES comes from full 24D covariance; ESKF is evaluated at its fusion epoch.",
        "Correlated samples are descriptive, not independent Monte Carlo consistency tests.",
        "Native per-sensor NIS, rich held-out captures and complete Modelica ports remain incomplete.",
        f"Undefined NEES condition/windows retained: {len(summary['undefined_nees_windows'])}.",
        "The option stays disabled; vertical improvements do not offset horizontal regressions.",
        "",
        "Paired flight RMS changes (negative is better):",
    ]
    for row in changes:
        text.append(
            f"{row['estimator']} {row['metric']}: {row['improved']}/12 improved; "
            f"{row['minimum_change_percent']:.2f}% to {row['maximum_change_percent']:+.2f}%"
        )
    text += ["", "Flight RMS wins versus BOTH frozen native cores (out of twelve):"]
    for row in wins:
        if row["window"] == "flight":
            text.append(
                row["estimator"]
                + ": "
                + ", ".join(metric + "=" + str(row[metric]) for metric in METRICS)
            )
    text += [
        "",
        "Thread CPU medians over the full replay, including startup; not embedded worst-case timing:",
    ]
    for row in costs:
        text.append(
            f"{row['estimator']}: filter {row['median_filter_cpu_us_per_update']:.3f} us/update, "
            f"pipeline {row['median_pipeline_cpu_us_per_imu_packet']:.3f} us/IMU packet, "
            f"generated state {row['generated_state_bytes']} bytes"
        )
    (args.output / "comparison.txt").write_text("\n".join(text) + "\n")
    print("\n".join(text))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "default-study",
        "rest-study",
        "native",
        "native-covariance",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
