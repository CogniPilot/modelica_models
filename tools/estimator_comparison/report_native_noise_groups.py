"""Report every declared noise-group diagnostic, including covariance failures."""

import argparse
import csv
import json
from pathlib import Path

from compare_exposure import digest
from report_estimator_theory import METRICS


def report(args):
    study = json.loads(args.study.read_text())
    variants = study["declared_variants"]
    names = {row["name"] for row in study["scores"]}
    expected = {(name, variant) for name in names for variant in variants}
    actual = [(row["name"], row["variant"]) for row in study["scores"]]
    controls = [
        row for row in study["scores"] if row["variant"] in ("published", "full")
    ]
    if (
        not study.get("complete")
        or set(actual) != expected
        or len(actual) != len(expected)
        or any(not row["observer_output_identical"] for row in study["scores"])
        or any(not row["frozen_control_identical"] for row in controls)
    ):
        raise ValueError(
            "Require all declared variants and verified frozen/observer parity"
        )
    rows = []
    text = [
        study["scope"],
        "",
        "Condition: " + study["condition"],
        "",
        "No variants are removed or selected as a new baseline.",
        "Covariance failures have undefined NEES; finite RMS does not override them.",
        "",
        "estimator variant                  H RMS m   V RMS m   vel RMS   att deg   yaw deg   mean 15D NEES",
    ]
    for score in study["scores"]:
        covariance = score["consistency"]
        for window in ("flight", "outage_window", "after_return"):
            metrics = score["native"][window]
            statistics = (
                covariance["windows"]["outage" if window == "outage_window" else window]
                if covariance["valid"]
                else None
            )
            row = dict(
                estimator=score["name"],
                variant=score["variant"],
                window=window,
                **{metric: metrics[metric] for metric in METRICS},
                position_valid_fraction=metrics["position_valid_fraction"],
                attitude_valid_fraction=metrics["attitude_valid_fraction"],
                row_coverage=metrics["row_coverage"],
                covariance_valid=covariance["valid"],
                covariance_failure=covariance.get("reason", ""),
                mean_nees_15d=statistics["mean_nees_15d"] if statistics else None,
                output_sha256=score["native"]["output_sha256"],
            )
            rows.append(row)
            if window == "flight":
                nees = f"{row['mean_nees_15d']:.3f}" if statistics else "undefined"
                text.append(
                    f"{row['estimator']:<9} {row['variant']:<24} "
                    + " ".join(f"{row[metric]:9.5f}" for metric in METRICS)
                    + " "
                    + nees
                )
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "comparison.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    (args.output / "comparison.txt").write_text("\n".join(text) + "\n")
    summary = dict(
        study_sha256=digest(args.study),
        native_conditions=len(study["scores"]),
        native_replays=2 * len(study["scores"]),
        frozen_controls_identical=len(controls),
        observer_pairs_identical=len(study["scores"]),
        covariance_failures=[
            dict(
                estimator=row["name"],
                variant=row["variant"],
                reason=row["consistency"]["reason"],
            )
            for row in study["scores"]
            if not row["consistency"]["valid"]
        ],
        scope=study["scope"],
    )
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("study", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
