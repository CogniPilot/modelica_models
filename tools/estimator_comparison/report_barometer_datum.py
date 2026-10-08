"""Report pressure-datum accuracy and common-coordinate native/ESKF NEES."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from compare_exposure import digest
from report_estimator_theory import METRICS


METHODS = {
    "horizon_control": "ESKF horizon control",
    "horizon_candidate": "ESKF horizon declared rest",
    "retrodiction_control": "ESKF retrodiction control",
    "retrodiction_candidate": "ESKF retrodiction declared rest",
    "px4": "PX4 EKF2",
    "ekf3": "ArduPilot EKF3",
}
CASE_FIELDS = ("seed", "frequency", "climb_height_m", "scenario")
WINDOWS = ("before_takeoff", "flight", "outage", "after_return")


def case(row):
    return tuple(row[name] for name in CASE_FIELDS)


def report(args):
    study, native, covariance = [
        json.loads(path.read_text())
        for path in (args.study, args.native, args.native_covariance)
    ]
    if (
        not study.get("complete")
        or len(study["scores"]) != 48
        or study["reference_sha256"] != digest(args.native)
        or covariance["reference_sha256"] != digest(args.native)
        or len(covariance["scores"]) != 24
    ):
        raise ValueError("Require complete campaigns on the same frozen reference")
    groups = {name: {} for name in METHODS}
    for row in study["scores"]:
        if row["variant"] == "control" and not row["control_output_identical"]:
            raise ValueError("Fresh ESKF controls differ from the frozen outputs")
        if any(not row["consistency"][window]["valid"] for window in WINDOWS):
            raise ValueError("Do not rank undefined NEES or hide covariance failures")
        groups[row["name"] + "_" + row["variant"]][case(row)] = row
    observed = {(row["name"], case(row)): row for row in covariance["scores"]}
    for row in native["scores"]:
        if row["name"] not in ("px4", "ekf3") or not row["explicit_exposure"]:
            continue
        observation = observed[row["name"], case(row)]
        if (
            not observation["output_identical"]
            or observation["output_sha256"] != row["output_sha256"]
        ):
            raise ValueError("Native covariance observer failed published-state parity")
        groups[row["name"]][case(row)] = dict(
            **row, consistency=observation["consistency"]["windows"]
        )
    conditions = set(groups["horizon_control"])
    if len(conditions) != 12 or any(
        set(group) != conditions for group in groups.values()
    ):
        raise ValueError("Every method must cover all twelve frozen conditions")
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, group in groups.items():
        for condition, score in sorted(group.items()):
            for window in WINDOWS:
                statistics = score["consistency"][window]
                metric_window = "outage_window" if window == "outage" else window
                metrics = score.get(metric_window, {})
                rows.append(
                    dict(
                        estimator=METHODS[name],
                        **dict(zip(CASE_FIELDS, condition, strict=True)),
                        window=window,
                        mean_nees_15d=statistics["mean_nees_15d"],
                        nees_rows=statistics["rows"],
                        **{metric: metrics.get(metric) for metric in METRICS},
                    )
                )
    with (args.output / "comparison.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    wins = []
    for name in tuple(METHODS)[:4]:
        for window in ("flight", "outage_window", "after_return"):
            counts = {
                metric: sum(
                    groups[name][condition][window][metric]
                    < min(
                        groups[native][condition][window][metric]
                        for native in ("px4", "ekf3")
                    )
                    for condition in conditions
                )
                for metric in METRICS
            }
            wins.append(dict(estimator=METHODS[name], window=window, **counts))
    ratios = [
        groups[method + "_candidate"][condition]["flight"]["vertical_position_rmse_m"]
        / groups[method + "_control"][condition]["flight"]["vertical_position_rmse_m"]
        for method in ("horizon", "retrodiction")
        for condition in conditions
    ]
    paired = []
    for method in ("horizon", "retrodiction"):
        for condition in sorted(conditions):
            candidate = groups[method + "_candidate"][condition]
            control = groups[method + "_control"][condition]
            for metric in METRICS:
                baseline = control["flight"][metric]
                measured = candidate["flight"][metric]
                paired.append(
                    dict(
                        estimator=METHODS[method + "_candidate"],
                        **dict(zip(CASE_FIELDS, condition, strict=True)),
                        metric=metric,
                        control=baseline,
                        candidate=measured,
                        change_percent=100 * (measured / baseline - 1),
                    )
                )
    with (args.output / "paired-flight.csv").open("w") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=paired[0].keys(), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(paired)
    horizontal = [
        row for row in paired if row["metric"] == "horizontal_position_rmse_m"
    ]
    summary = dict(
        cases_per_method=12,
        controls_byte_identical=24,
        vertical_rmse_reduction_percent_range=[
            float(100 * (1 - max(ratios))),
            float(100 * (1 - min(ratios))),
        ],
        lower_rmse_than_both_native=wins,
        horizontal_rmse_change_percent_range=[
            min(row["change_percent"] for row in horizontal),
            max(row["change_percent"] for row in horizontal),
        ],
        worst_horizontal_regression=max(
            horizontal, key=lambda row: row["change_percent"]
        ),
        source_sha256={
            "study": digest(args.study),
            "native": digest(args.native),
            "native_covariance": digest(args.native_covariance),
        },
    )
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    text = [
        "Declared startup rest pressure calibration: frozen matched-data follow-up",
        "",
        "24 fresh ESKF controls reproduce their frozen published outputs byte for byte.",
        "24 candidate runs use the same captures, arrivals, IMU density and tuning.",
        "Only startup pressure calibration policy changes; the option defaults off.",
        "The two seeds are coupled to motion scale; this is not independent Monte Carlo.",
        "Native effective R/Q, priors and height/magnetic policies remain unequal.",
        "The datum assumes declared startup rest at the configured initial altitude.",
        "Native NEES uses full 24x24 covariance mapped to the common 15D marginal.",
        "ESKF NEES uses its delayed state and covariance at the matching fusion epoch.",
        "All observed native rows are retained; repeated epochs remain correlated.",
        "These are descriptive NEES values; lower is not inherently better.",
        "Per-sensor native NIS and exact joint pressure/navigation covariance are unresolved.",
        "No universal superiority, complete native port, or deployment qualification claim.",
        "",
        "Cases with lower RMSE than BOTH native cores (twelve conditions per row):",
        "estimator                             window         horizontal vertical velocity attitude yaw",
    ]
    for row in wins:
        text.append(
            f"{row['estimator']:<37} {row['window']:<14} "
            + " ".join(f"{row[metric]:2}/12" for metric in METRICS)
        )
    text += [
        "",
        "Candidate flight vertical RMSE reduction versus its control: "
        + " to ".join(
            f"{value:.1f}%"
            for value in summary["vertical_rmse_reduction_percent_range"]
        ),
        "Candidate flight horizontal RMSE change versus its control: "
        + " to ".join(
            f"{value:+.1f}%"
            for value in summary["horizontal_rmse_change_percent_range"]
        ),
        "See paired-flight.csv for every candidate/control pair; the option stays off.",
        "",
        "Flight RMSE and NEES medians across seeds/heights:",
        "scenario   estimator                              vertical m  mean 15D NEES",
    ]
    figure, axes = plt.subplots(3, 2, figsize=(13, 11), layout="constrained")
    colors = ("#8a9ba8", "#007f73", "#b1a5bd", "#9354aa", "#cc7b13", "#2e5ca5")
    for scenario_index, scenario in enumerate(("gps", "denied", "transition")):
        for method_index, (name, group) in enumerate(groups.items()):
            selected = [
                row for key, row in sorted(group.items()) if key[-1] == scenario
            ]
            vertical = [row["flight"]["vertical_position_rmse_m"] for row in selected]
            nees = [row["consistency"]["flight"]["mean_nees_15d"] for row in selected]
            text.append(
                f"{scenario:<10} {METHODS[name]:<38} {np.median(vertical):10.5f} {np.median(nees):14.4f}"
            )
            for column, values in enumerate((vertical, nees)):
                x = method_index + np.linspace(-0.12, 0.12, len(values))
                axes[scenario_index, column].scatter(
                    x, values, color=colors[method_index], s=40
                )
                axes[scenario_index, column].plot(
                    [method_index - 0.2, method_index + 0.2],
                    [np.median(values)] * 2,
                    color=colors[method_index],
                    linewidth=2,
                )
        for column, label in enumerate(("Vertical RMSE (m)", "Mean common 15D NEES")):
            axis = axes[scenario_index, column]
            axis.set_title(scenario)
            axis.set_ylabel(label)
            axis.set_xticks(
                range(len(METHODS)),
                METHODS.values(),
                rotation=35,
                ha="right",
                fontsize=8,
            )
            axis.set_ylim(bottom=0)
            axis.grid(axis="y", alpha=0.25)
        axes[scenario_index, 1].axhline(
            15, color="#555555", linestyle="--", linewidth=1
        )
    figure.suptitle(
        "Frozen cases: pressure-datum accuracy and descriptive consistency\nUnequal native effective R/Q and priors; no universal ranking"
    )
    for suffix in ("svg", "png"):
        figure.savefig(args.output / ("comparison." + suffix), dpi=160)
    plt.close(figure)
    (args.output / "comparison.txt").write_text("\n".join(text) + "\n")
    print("\n".join(text[-20:]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("study", "native", "native-covariance", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
