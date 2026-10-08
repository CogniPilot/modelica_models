"""Check actual GPS fusion before outage and EKF3 gyro-bias readiness."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path

import numpy as np

from compare_exposure import digest


FIELDS = ("name", "frequency", "seed", "climb_height_m", "scenario")


def diagnose(args):
    study = json.loads(args.innovations.read_text())
    covariance = json.loads(args.native_covariance.read_text())
    if (
        not study.get("complete")
        or len(study["scores"]) != 24
        or len(covariance["scores"]) != 24
    ):
        raise ValueError("Require both complete native observer campaigns")
    if study["reference_sha256"] != covariance["reference_sha256"]:
        raise ValueError("Native observer references differ")
    observed = {
        tuple(row[field] for field in FIELDS): row for row in covariance["scores"]
    }
    if len(observed) != 24 or {
        tuple(row[field] for field in FIELDS) for row in study["scores"]
    } != set(observed):
        raise ValueError("Native observer conditions are duplicated or differ")
    records = []
    for score in study["scores"]:
        previous = observed[tuple(score[field] for field in FIELDS)]
        if (
            not score["output_identical"]
            or not previous["output_identical"]
            or score["output_sha256"] != previous["output_sha256"]
        ):
            raise ValueError(
                "Native innovation/covariance observer trajectories differ"
            )
        label = f"{score['frequency']}_{score['seed']}_{score['climb_height_m']}m_{score['scenario']}_explicit-{score['name']}"
        path = args.innovation_work / label / "native-innovations.csv"
        if digest(path) != score["innovations"]["observer_csv_sha256"]:
            raise ValueError("Native innovation trace changed")
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        gps = [
            row
            for row in rows
            if row["sensor"] == "gps_position"
            and row["stage"] == "fusion"
            and int(row["fused"]) == 1
            and int(row["axis"]) == 0
        ]
        epochs = np.array([int(row["fusion_us"]) / 1e6 for row in gps])
        samples = np.array([int(row["sample_us"]) for row in gps])
        before_loss = int(((epochs >= 13) & (epochs < 25)).sum())
        after_return = int(((epochs >= 40) & (epochs < 59.7)).sum())
        record = dict(
            **{field: score[field] for field in FIELDS},
            first_gps_position_fusion_s=float(epochs[0]) if len(epochs) else None,
            gps_position_fusions_before_loss=before_loss,
            gps_position_fusions_after_return=after_return,
            native_gps_sample_intervals_us={
                str(key): value
                for key, value in sorted(Counter(np.diff(samples).tolist()).items())
            },
            gps_active_before_and_after_requested_outage=bool(
                before_loss and after_return
            )
            if score["scenario"] == "transition"
            else None,
        )
        if score["name"] == "ekf3":
            path = args.covariance_work / label / "native-covariance.csv"
            if digest(path) != previous["consistency"]["covariance_sha256"]:
                raise ValueError("Native covariance trace changed")
            values = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
            fusion = values["fusion_us"] / 1e6
            ratio = (
                np.maximum.reduce([values[f"p{i}_{i}"] for i in (10, 11, 12)])
                / (np.deg2rad(0.15) * values["bias_period_s"]) ** 2
            )
            candidates = np.flatnonzero((fusion >= 10) & (ratio <= 1))
            takeoff = int(np.argmin(abs(fusion - 13)))
            record.update(
                first_logged_gyro_variance_ready_after_10s=float(fusion[candidates[0]])
                if len(candidates)
                else None,
                maximum_gyro_variance_over_readiness_threshold_at_13s=float(
                    ratio[takeoff]
                ),
                readiness_scope="Necessary native all-axis covariance condition with compass aiding; post-update snapshots, not all readiness flags or a sole-cause intervention.",
            )
        records.append(record)
    result = dict(
        records=records,
        complete=True,
        innovations_sha256=digest(args.innovations),
        native_covariance_sha256=digest(args.native_covariance),
        note="Same published trajectories under both observers. GPS fusion events are measured, not inferred from supplied packets. The existing 25-40s outage is a true GPS loss/return experiment only if GPS fusion was active before loss. Readiness variances do not establish that all other admission conditions passed.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    for row in records:
        print(
            row["name"],
            row["frequency"],
            row["seed"],
            row["climb_height_m"],
            row["scenario"],
            "first GPS",
            row["first_gps_position_fusion_s"],
            "GPS before outage",
            row["gps_position_fusions_before_loss"],
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "innovations",
        "native-covariance",
        "innovation-work",
        "covariance-work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    diagnose(parser.parse_args())
