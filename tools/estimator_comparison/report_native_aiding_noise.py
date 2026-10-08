"""Report native noise ablations without hiding failed covariance diagnostics."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from compare_exposure import digest
from report_barometer_datum import CASE_FIELDS, WINDOWS, case
from report_estimator_theory import METRICS


def report(args):
    study, reference, observed, eskf = [
        json.loads(path.read_text())
        for path in (args.study, args.reference, args.observers, args.eskf)
    ]
    if (
        not study.get("complete")
        or len(study["scores"]) != 24
        or study["reference_sha256"] != digest(args.reference)
        or study["observer_reference_sha256"] != digest(args.observers)
        or observed["reference_sha256"] != digest(args.reference)
        or not eskf.get("complete")
        or eskf["reference_sha256"] != digest(args.reference)
    ):
        raise ValueError("Require completed campaigns on the frozen stable reference")
    groups = {}

    def insert(name, condition, score):
        group = groups.setdefault(name, {})
        if condition in group:
            raise ValueError("Duplicated comparison condition")
        score = dict(score, invalid_rms_windows=[])
        for window in ("flight", "outage_window", "after_return"):
            metrics = score[window]
            if (
                metrics["finite_fraction"] != 1
                or metrics["row_coverage"] < 0.999
                or metrics["position_valid_fraction"] != 1
                or metrics["attitude_valid_fraction"] != 1
                or not all(np.isfinite(metrics[metric]) for metric in METRICS)
            ):
                score["invalid_rms_windows"].append(window)
        group[condition] = score

    for row in eskf["scores"]:
        if row["variant"] == "control" and not row["control_output_identical"]:
            raise ValueError("ESKF control changed")
        name = "eskf_" + row["name"] + "_" + row["variant"]
        insert(name, case(row), row)
    covariance = {(row["name"], case(row)): row for row in observed["scores"]}
    for row in reference["scores"]:
        if row["name"] not in ("px4", "ekf3") or not row["explicit_exposure"]:
            continue
        observer = covariance[row["name"], case(row)]
        if (
            not observer["output_identical"]
            or observer["output_sha256"] != row["output_sha256"]
        ):
            raise ValueError("Published native observer parity failed")
        insert(
            row["name"] + "_published",
            case(row),
            dict(**row, consistency=observer["consistency"]["windows"]),
        )
    for row in study["scores"]:
        if not row["observer_output_identical"]:
            raise ValueError("Sensor-informed native observer parity failed")
        consistency = row["consistency"]
        insert(
            row["name"] + "_sensor_informed",
            case(row),
            dict(
                **row["native"],
                consistency=consistency["windows"] if consistency["valid"] else None,
                covariance_failure=consistency.get("reason", ""),
            ),
        )
    conditions = set(groups["eskf_horizon_control"])
    if len(conditions) != 12 or any(
        set(group) != conditions for group in groups.values()
    ):
        raise ValueError("Missing or duplicated comparison conditions")
    rows = []
    failures = []
    state_failures = []
    for name, group in groups.items():
        for condition, score in sorted(group.items()):
            if score["invalid_rms_windows"]:
                state_failures.append(
                    dict(
                        estimator=name,
                        **dict(zip(CASE_FIELDS, condition, strict=True)),
                        invalid_windows=score["invalid_rms_windows"],
                    )
                )
            if score.get("covariance_failure"):
                failures.append(
                    dict(
                        estimator=name,
                        **dict(zip(CASE_FIELDS, condition, strict=True)),
                        reason=score["covariance_failure"],
                    )
                )
            for window in WINDOWS:
                statistics = (
                    score["consistency"][window] if score["consistency"] else None
                )
                metrics = score.get(
                    "outage_window" if window == "outage" else window, {}
                )
                rows.append(
                    dict(
                        estimator=name,
                        **dict(zip(CASE_FIELDS, condition, strict=True)),
                        window=window,
                        valid_rms=("outage_window" if window == "outage" else window)
                        not in score["invalid_rms_windows"],
                        **{metric: metrics.get(metric) for metric in METRICS},
                        mean_nees_15d=statistics["mean_nees_15d"]
                        if statistics
                        else None,
                        nees_rows=statistics["rows"] if statistics else None,
                        covariance_failure=score.get("covariance_failure", ""),
                    )
                )
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "comparison.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    wins = []
    for name in groups:
        if not name.startswith("eskf_"):
            continue
        for baseline in ("published", "sensor_informed"):
            for window in ("flight", "outage_window", "after_return"):
                eligible = [
                    condition
                    for condition in conditions
                    if all(
                        window not in groups[method][condition]["invalid_rms_windows"]
                        for method in (name, "px4_" + baseline, "ekf3_" + baseline)
                    )
                ]
                wins.append(
                    dict(
                        estimator=name,
                        native_configuration=baseline,
                        window=window,
                        eligible_conditions=len(eligible),
                        **{
                            metric: sum(
                                groups[name][condition][window][metric]
                                < min(
                                    groups[native + "_" + baseline][condition][window][
                                        metric
                                    ]
                                    for native in ("px4", "ekf3")
                                )
                                for condition in eligible
                            )
                            for metric in METRICS
                        },
                    )
                )
    summary = dict(
        cases_per_method=12,
        native_observer_pairs_identical=24,
        covariance_failures=failures,
        state_validity_failures=state_failures,
        lower_rmse_than_both_native=wins,
        source_sha256={
            name: digest(path) for name, path in vars(args).items() if name != "output"
        },
        scope=study["scope"],
        limitations=study["profile"]["limitations"],
    )
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    text = [
        "Sensor-informed native noise configuration: unchanged physical captures and delivery",
        "",
        study["scope"],
        "48 native runs: 24 control/observer pairs have byte-identical published states.",
        "ESKF results are unchanged frozen replays, not retuned for this experiment.",
        "Undefined full-covariance NEES is recorded as failure, never regularized or dropped.",
        "All failed conditions remain in raw RMS tables; invalid windows are excluded from paired win counts with an explicit denominator.",
        "Raw RMS medians include failed states and are descriptive, not acceptance results. A smaller NEES is not a quality ranking.",
        "Only two seeds, coupled to motion scale; this is not independent Monte Carlo.",
        "",
        "Full-covariance failures:",
        *(str(row) for row in failures),
        "",
        "State validity failures:",
        *(str(row) for row in state_failures),
        "",
        "Flight medians across four seed/height conditions (m, m, m/s, deg, deg):",
        "scenario    estimator                          horizontal  vertical  velocity  attitude  yaw      mean 15D NEES",
    ]
    for scenario in ("gps", "denied", "transition"):
        for name, group in groups.items():
            selected = [
                row
                for condition, row in sorted(group.items())
                if condition[-1] == scenario
            ]
            medians = [
                np.median([row["flight"][metric] for row in selected])
                for metric in METRICS
            ]
            nees = [
                row["consistency"]["flight"]["mean_nees_15d"]
                for row in selected
                if row["consistency"]
            ]
            label = (
                f"{np.median(nees):.3f}"
                if len(nees) == 4
                else f"undefined ({len(nees)}/4 valid)"
            )
            text.append(
                f"{scenario:<11} {name:<34} "
                + " ".join(f"{value:9.5f}" for value in medians)
                + " "
                + label
            )
    text += [
        "",
        "Flight cases with lower RMS than BOTH native cores, among valid conditions only:",
        "estimator                          native settings     horizontal vertical velocity attitude yaw",
    ]
    for row in wins:
        if row["window"] == "flight":
            text.append(
                f"{row['estimator']:<34} {row['native_configuration']:<19} "
                + " ".join(
                    f"{row[metric]:2}/{row['eligible_conditions']}"
                    for metric in METRICS
                )
            )
    text += [
        "",
        *study["profile"]["limitations"],
        "",
        "No fully matched R/Q, native NIS, universal superiority or deployment qualification claim.",
    ]
    (args.output / "comparison.txt").write_text("\n".join(text) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("study", "reference", "observers", "eskf", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
