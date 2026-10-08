#!/usr/bin/env python3
"""Report explicit exposure fidelity, paired mapping effects and remaining gaps."""

import argparse
import csv
import itertools
import json
from pathlib import Path

import numpy as np


METRICS = (
    "horizontal_position_rmse_m",
    "vertical_position_rmse_m",
    "velocity_3d_rmse_m_s",
    "attitude_rmse_deg",
    "yaw_rmse_deg",
)
NAMES = ("horizon", "retrodiction", "px4", "ekf3")
KEYS = ("seed", "climb_height_m", "scenario", "explicit_exposure", "name")


def validate(evidence, controls):
    expected = set(
        itertools.product(
            (7, 101), (2, 4), ("gps", "denied", "transition"), (False, True), NAMES
        )
    )
    indexed = {tuple(row[key] for key in KEYS): row for row in evidence["scores"]}
    if set(indexed) != expected or len(evidence["scores"]) != len(expected):
        raise ValueError("Expected 96 unique replay results")
    if (
        len(controls) != 8
        or {(r["seed"], r["name"]) for r in controls}
        != set(itertools.product((7, 101), NAMES))
        or any(r["frozen_output_identical"] is not True for r in controls)
    ):
        raise ValueError("Expected eight exact frozen-output controls")
    for row in indexed.values():
        shared = indexed[
            (row["seed"], row["climb_height_m"], row["scenario"], False, "horizon")
        ]
        if (
            row["transport"] != shared["transport"]
            or row["arrival_trace_sha256"] != shared["arrival_trace_sha256"]
        ):
            raise ValueError("Paired sensor delivery differs")
        for window, start, end in (
            ("flight", 13, 60),
            ("outage_window", 25, 40),
            ("after_return", 40, 60),
        ):
            accuracy = row[window]
            if (
                (accuracy["start_s"], accuracy["end_s"]) != (start, end)
                or accuracy["finite_fraction"] != 1
                or any(not np.isfinite(accuracy[k]) for k in METRICS)
            ):
                raise ValueError("Invalid output or scoring window")
        if row["name"] in ("horizon", "retrodiction"):
            if row["flight"]["nonzero_step_status_rows"]:
                raise ValueError("Generated ESKF error status")
            for window, start, end in (
                ("before_takeoff", 10, 13),
                ("flight", 13, 59.7),
            ):
                check = row["consistency"][window]
                if (check["start_s"], check["end_s"]) != (start, end):
                    raise ValueError("Invalid covariance scoring window")
                if not row["consistency"][window]["valid"]:
                    matrix = np.array(check["first_failed_covariance"])
                    if (
                        not 0 < check["failed_samples"] <= check["rows"]
                        or not start <= check["first_failed_epoch_s"] < end
                        or matrix.shape != (15, 15)
                        or not np.isfinite(matrix).all()
                        or not np.array_equal(matrix, matrix.T)
                        or not np.allclose(
                            np.linalg.eigvalsh(matrix),
                            check["first_failed_eigenvalues"],
                            rtol=1e-10,
                            atol=1e-12,
                        )
                    ):
                        raise ValueError("Unverified covariance failure")
                    try:
                        np.linalg.cholesky(matrix)
                    except np.linalg.LinAlgError:
                        continue
                    raise ValueError("Reported failed covariance is positive definite")
                if (
                    not np.isfinite(row["consistency"][window]["mean_nees_15d"])
                    or row["consistency"][window]["max_covariance_asymmetry"] != 0
                ):
                    raise ValueError("Invalid ESKF covariance")
            if row["name"] == "horizon" and any(
                row["timing"][k] for k in ("late", "stale", "overflow")
            ):
                raise ValueError("Lost horizon packet")
            if (
                row["explicit_exposure"]
                and row["timing"]["flow_exposure_packets"]
                != row["transport"]["transport_delivered"][1]
            ):
                raise ValueError("Missing canonical ESKF exposure")
        elif row["explicit_exposure"]:
            check = (
                row["flow_api"] if row["name"] == "px4" else row["flow_serialization"]
            )
            if check["rows"] != row["transport"]["transport_delivered"][1] or any(
                not np.isfinite(value) or value > 2e-6
                for key, value in check.items()
                if key.startswith("max_")
            ):
                raise ValueError("Native flow serialization or API mismatch")
    return indexed


def report(args):
    evidence = json.loads(args.input.read_text())
    controls = json.loads(args.controls.read_text())
    rows = list(validate(evidence, controls).values())
    lines = [
        "Native/ESKF explicit optical-flow exposure probe — 7 October 2026",
        "",
        "96 fresh replays: seeds 7/101, 2/4 m climbs, GPS/denied/loss-return, four estimators,",
        "legacy reconstruction versus explicit exposure mappings on the SAME new captures.",
        "Eight additional historical controls reproduce original outputs byte for byte.",
        "Pinned native cores are unchanged. This is a two-seed measurement-contract probe.",
        "",
        "Common physical packet",
        "100 ms exposure at 10 Hz; timestamp is its midpoint, availability starts at exposure end.",
        "Image motion includes true camera rotation plus frozen image translation noise.",
        "The camera gyro has independent unbiased calibrated noise; its noise is not constructed into the image and cancelled.",
        "Flow, magnetic and barometric delivery rates are 10 Hz, below EKF3's ingestion cap.",
        "Transport delays GPS/flow/mag/baro are 110/30/60/20 ms, with zero jitter.",
        "60 ms magnetic transport represents EKF3's fixed magnetic delay; each native core retains its half-step convention.",
        "ESKF receives integrated FLU image/gyro angles, exposure duration, midpoint and packet variances.",
        "PX4 receives rate-valued image/gyro components and midpoint timestamps.",
        "EKF3 receives rate-valued DAL components; its fixed 100 ms exposure and configured transport delay define the epoch.",
        "The canonical-to-native mappings are verified after actual serialization/API conversion.",
        "",
        "Explicit exposure results: median across two seeds, published-state flight RMSE 13–60 s",
        "height scenario   estimator        horiz m   vert m   vel m/s   att deg   yaw deg",
    ]
    for height, scenario, name in itertools.product(
        (2, 4), ("gps", "denied", "transition"), NAMES
    ):
        selected = [
            row
            for row in rows
            if row["climb_height_m"] == height
            and row["scenario"] == scenario
            and row["name"] == name
            and row["explicit_exposure"]
        ]
        values = [
            np.median([row["flight"][key] for row in selected]) for key in METRICS
        ]
        lines.append(
            f"{height:4} m {scenario:10} {name:12} "
            + " ".join(f"{value:9.4f}" for value in values)
        )
    lines += [
        "",
        "Explicit versus legacy mapping on identical new captures",
        "Median paired percent change across 12 conditions; negative means smaller error.",
        "estimator        horiz%    vert%     vel%     att%     yaw%",
    ]
    for name in NAMES:
        changes = []
        for row in rows:
            if row["name"] != name or not row["explicit_exposure"]:
                continue
            previous = next(
                r
                for r in rows
                if (
                    r["seed"],
                    r["climb_height_m"],
                    r["scenario"],
                    r["name"],
                    r["explicit_exposure"],
                )
                == (row["seed"], row["climb_height_m"], row["scenario"], name, False)
            )
            changes.append(
                [
                    100 * (row["flight"][key] / previous["flight"][key] - 1)
                    for key in METRICS
                ]
            )
        lines.append(
            f"{name:14} "
            + " ".join(f"{value:8.2f}" for value in np.median(changes, axis=0))
        )
    lines += [
        "",
        "ESKF mean 15D NEES, median across 12 conditions (descriptive correlated samples)",
        "method          mapping     before takeoff    flight",
    ]
    for name, exposure in itertools.product(("horizon", "retrodiction"), (False, True)):
        selected = [
            row
            for row in rows
            if row["name"] == name and row["explicit_exposure"] == exposure
        ]
        values = []
        for window in ("before_takeoff", "flight"):
            invalid = sum(not row["consistency"][window]["valid"] for row in selected)
            values.append(
                f"INVALID ({invalid}/12)"
                if invalid
                else f"{np.median([row['consistency'][window]['mean_nees_15d'] for row in selected]):.3f}"
            )
        lines.append(
            f"{name:14} {'explicit' if exposure else 'legacy':10} {values[0]:>15} {values[1]:>15}"
        )
    failures = [
        row
        for row in rows
        if row["name"] in ("horizon", "retrodiction")
        and any(not check["valid"] for check in row["consistency"].values())
    ]
    lines += [
        "",
        f"Covariance failures: {len(failures)} replay cases; all accuracy results remain included.",
    ]
    for row in failures:
        for window, check in row["consistency"].items():
            if not check["valid"]:
                lines.append(
                    f"seed {row['seed']}, {row['climb_height_m']} m, {row['scenario']}, {row['name']}, {'explicit' if row['explicit_exposure'] else 'legacy'}, {window}: {check['failed_samples']} non-PD samples; first epoch {check['first_failed_epoch_s']:.6f} s. NEES unavailable."
                )
    lines += [
        "",
        "Remaining comparison gaps",
        *evidence["limitations"],
        "",
        "Required next steps: separate range sampling/latency, actual accepted EKF3 epochs and supported source policies;",
        "then run the complete seed/rate/delay/disturbance matrix before another algorithm-superiority claim.",
    ]
    args.output.with_suffix(".txt").write_text("\n".join(lines) + "\n")
    with args.output.with_suffix(".csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=[*KEYS, "window", *METRICS])
        writer.writeheader()
        for row in rows:
            for window in ("flight", "outage_window", "after_return"):
                writer.writerow(
                    {
                        **{key: row[key] for key in KEYS},
                        "window": window,
                        **{key: row[window][key] for key in METRICS},
                    }
                )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input", "controls", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
