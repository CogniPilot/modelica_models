"""Join a declared matched campaign without discarding failed conditions."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
from statistics import median

from manifest import digest


METRICS = (
    "horizontal_position_rmse_m",
    "vertical_position_rmse_m",
    "velocity_3d_rmse_m_s",
    "attitude_rmse_deg",
    "yaw_rmse_deg",
)
WINDOWS = ("flight", "outage_window", "after_return")
NATIVE = ("px4", "ekf3")


def indexed(rows, fields):
    result = {tuple(row[field] for field in fields): row for row in rows}
    if len(result) != len(rows):
        raise ValueError("Duplicated comparison condition")
    return result


def join(pilot, covariance, pilot_sha256, capture, plan):
    expected = set(itertools.product(plan["filters"], plan["scenarios"]))
    scores = indexed(pilot["scores"], ("name", "scenario"))
    native = indexed(covariance["scores"], ("name", "scenario"))
    if (
        not pilot["complete"]
        or not covariance["complete"]
        or covariance["pilot_sha256"] != pilot_sha256
        or set(scores) != expected
        or set(native) != {key for key in expected if key[0] in NATIVE}
    ):
        raise ValueError("Require complete declared rows and a bound covariance replay")
    origin = pilot["origin"]
    if (
        (origin["seed"], origin["speed"]) != (capture["seed"], capture["speed"])
        or origin["gps_fix_after_s"] != plan["gps_fix_after_s"]
        or pilot["input_sha256"] != capture["audit"]["input_sha256"]
    ):
        raise ValueError("Pilot does not use the independently audited capture")
    joined = []
    for scenario in plan["scenarios"]:
        arrivals = {
            scores[name, scenario]["arrival_trace_sha256"] for name in plan["filters"]
        }
        if len(arrivals) != 1:
            raise ValueError("Estimators received different sensor arrival traces")
        for name in plan["filters"]:
            score = scores[name, scenario]
            if name in NATIVE:
                observer = native[name, scenario]
                if (
                    not observer["output_identical"]
                    or observer["output_sha256"] != score["output_sha256"]
                    or observer["arrival_trace_sha256"] not in arrivals
                ):
                    raise ValueError(
                        "Native covariance observer changed inputs or state"
                    )
                consistency = observer["consistency"]
            else:
                consistency = score["consistency"]
            for window in WINDOWS:
                statistics_window = "outage" if window == "outage_window" else window
                valid_covariance = (
                    consistency["valid"]
                    if name in NATIVE
                    else consistency[statistics_window]["valid"]
                )
                statistics = (
                    consistency["windows"][statistics_window]
                    if name in NATIVE and valid_covariance
                    else consistency.get(statistics_window, {})
                )
                accuracy = score[window]
                valid_state = (
                    accuracy["finite_fraction"] == 1
                    and accuracy["position_valid_fraction"] == 1
                    and accuracy["attitude_valid_fraction"] == 1
                    and accuracy["row_coverage"] >= 0.999
                )
                joined.append(
                    dict(
                        capture=capture["name"],
                        seed=capture["seed"],
                        frequency_rad_s=capture["speed"],
                        height_m=capture["height_m"],
                        estimator=name,
                        scenario=scenario,
                        window=window,
                        **{metric: accuracy.get(metric) for metric in METRICS},
                        state_valid=valid_state,
                        readiness_qualified=score.get("readiness", {}).get("qualified"),
                        covariance_valid=valid_covariance,
                        covariance_failure=consistency.get("reason", ""),
                        mean_nees_15d=statistics.get("mean_nees_15d")
                        if valid_covariance
                        else None,
                        **{
                            key: accuracy[key]
                            for key in (
                                "rows",
                                "row_coverage",
                                "finite_fraction",
                                "position_valid_fraction",
                                "attitude_valid_fraction",
                            )
                        },
                        output_sha256=score["output_sha256"],
                        arrival_trace_sha256=score["arrival_trace_sha256"],
                    )
                )
    return joined


def comparisons(rows, filters, scenarios, captures):
    groups = indexed(rows, ("capture", "estimator", "scenario", "window"))
    pairs = [
        (name, native) for name in filters if name not in NATIVE for native in NATIVE
    ]
    pairs += [
        ("horizon", "retrodiction"),
        ("horizon_joint", "retrodiction_joint"),
        ("horizon_joint", "horizon"),
        ("retrodiction_joint", "retrodiction"),
    ]
    result = []
    for left, right in pairs:
        for scenario, window, metric in itertools.product(scenarios, WINDOWS, METRICS):
            values, failures = [], []
            for capture in captures:
                a, b = (
                    groups.get((capture, name, scenario, window))
                    for name in (left, right)
                )
                if a is None or b is None:
                    failures.append(capture)
                    continue
                if any(
                    not row["state_valid"]
                    or not row["covariance_valid"]
                    or row["readiness_qualified"] is False
                    for row in (a, b)
                ):
                    failures.append(capture)
                    continue
                x, y = a[metric], b[metric]
                if x is None or y is None or not math.isfinite(x + y):
                    failures.append(capture)
                    continue
                values.append((x, y))
            result.append(
                dict(
                    left=left,
                    right=right,
                    scenario=scenario,
                    window=window,
                    metric=metric,
                    declared_pairs=len(captures),
                    valid_pairs=len(values),
                    invalid_pairs=failures,
                    left_lower=sum(x < y for x, y in values),
                    right_lower=sum(y < x for x, y in values),
                    ties=sum(x == y for x, y in values),
                    median_left_minus_right=median(x - y for x, y in values)
                    if values
                    else None,
                    worst_left_minus_right=max(x - y for x, y in values)
                    if values
                    else None,
                )
            )
    return result


def write_csv(path, rows):
    with path.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def innovation_summary(records):
    fields = (
        "estimator",
        "scenario",
        "sensor",
        "axis",
        "stage",
        "navigation",
        "window",
    )
    groups = {}
    for record in records:
        key = tuple(record[field] for field in fields)
        groups.setdefault(key, []).append(record)
    result = []
    for key, values in sorted(groups.items()):
        means = [row["mean_scalar_nis"] for row in values if row["valid"]]
        result.append(
            dict(
                **dict(zip(fields, key)),
                captures=len(values),
                invalid_captures=sum(not row["valid"] for row in values),
                median_capture_mean_scalar_nis=median(means) if means else None,
                minimum_capture_mean_scalar_nis=min(means) if means else None,
                maximum_capture_mean_scalar_nis=max(means) if means else None,
                observed_rows=sum(row["rows"] for row in values),
                invalid_rows=sum(row["invalid_rows"] for row in values),
                fused_rows=sum(row["fused_rows"] for row in values),
                rejected_rows=sum(row["rejected_fusion_rows"] for row in values),
            )
        )
    return result


def report(args):
    if args.output.exists():
        raise ValueError("Choose a new report directory")
    declaration = json.loads(args.declaration.read_text())
    plan = declaration["held_out_plan"]
    captures = json.loads(args.captures.read_text())
    campaign = json.loads(args.campaign.read_text())
    snapshot = json.loads(args.snapshot.read_text())
    cases = indexed(captures["cases"], ("name",))
    results = indexed(campaign["results"], ("name",))
    expected = set(itertools.product(plan["seeds"], plan["speeds"], plan["heights_m"]))
    actual = {
        (case["seed"], case["speed"], case["height_m"]) for case in cases.values()
    }
    if (
        not captures["complete"]
        or actual != expected
        or len(cases) != len(expected)
        or set(results) != set(cases)
        or campaign["capture_manifest_sha256"] != digest(args.captures)
        or campaign["source_snapshot_sha256"] != digest(args.snapshot)
    ):
        raise ValueError(
            "Require every declared condition and matching frozen manifests"
        )
    rows, innovations, timing, transitions, failures = [], [], [], [], []
    bindings = {}
    configuration = None
    for (name,), case in sorted(cases.items()):
        result = results[name,]
        if not result["complete"]:
            failures.append(dict(capture=name, result=result))
            continue
        work = args.root / name
        pilot, covariance = (
            json.loads((work / filename).read_text())
            for filename in ("pilot.json", "covariance.json")
        )
        hashes = {
            key: digest(work / filename)
            for key, filename in (
                ("pilot_sha256", "pilot.json"),
                ("covariance_sha256", "covariance.json"),
            )
        }
        if any(hashes[key] != result[key] for key in hashes):
            raise ValueError("Campaign result changed after completion")
        if any(
            snapshot.get(name) != value
            for name, value in pilot["source_sha256"].items()
        ):
            raise ValueError("Replay tools differ from the frozen campaign snapshot")
        current_configuration = {
            key: pilot[key]
            for key in (
                "binary_sha256",
                "noise_profile",
                "native_noise_configuration",
                "observer_manifests",
                "mission",
            )
        }
        current_configuration["native_covariance_binaries"] = covariance[
            "binary_sha256"
        ]
        if configuration is not None and current_configuration != configuration:
            raise ValueError("Estimator configuration changed between captures")
        configuration = current_configuration
        bindings[name] = hashes
        if pilot["mission"]["warmup_s"] != declaration["warmup_s"]:
            raise ValueError("Warmup differs from the declared campaign")
        truth = work / "capture/truth.csv"
        if digest(truth) != pilot["input_sha256"]["truth.csv"]:
            raise ValueError("Truth changed after the replay")
        with truth.open() as stream:
            height = max(float(row["u_m"]) for row in csv.DictReader(stream))
        if not math.isclose(height, case["height_m"], rel_tol=0, abs_tol=1e-8):
            raise ValueError("Physical climb differs from the declared condition")
        rows.extend(join(pilot, covariance, hashes["pilot_sha256"], case, plan))
        for score in pilot["scores"]:
            identity = dict(
                capture=name, estimator=score["name"], scenario=score["scenario"]
            )
            for record in score.get("innovations", {}).get("records", []):
                innovations.append(dict(**identity, **record))
            if "timing" in score:
                measured = score["timing"]
                total_ns = sum(
                    measured[part]["total_ns"]
                    for part in ("filter", "preintegration", "predictor", "queues")
                )
                timing.append(
                    dict(
                        **identity,
                        cpu_ms=total_ns / 1e6,
                        imu_ticks=measured["imu_ticks"],
                        cpu_us_per_imu_tick=total_ns / measured["imu_ticks"] / 1e3,
                        state_bytes=measured["state_bytes"],
                        buffer_bytes=measured.get("buffer_bytes", 0),
                        transport_bytes=measured["transport_bytes"],
                    )
                )
            if "transition" in score:
                transitions.append(dict(**identity, **score["transition"]))
    paired = comparisons(
        rows, plan["filters"], plan["scenarios"], sorted(name for (name,) in cases)
    )
    summary = dict(
        complete=len(bindings) == len(expected),
        declared_captures=len(expected),
        completed_captures=len(bindings),
        state_replays=len(rows) // len(WINDOWS),
        native_covariance_replays=sum(
            row["estimator"] in NATIVE and row["window"] == "flight" for row in rows
        ),
        failed_captures=failures,
        invalid_rows=[
            row
            for row in rows
            if not row["state_valid"]
            or not row["covariance_valid"]
            or row["readiness_qualified"] is False
        ],
        declaration_sha256=digest(args.declaration),
        captures_sha256=digest(args.captures),
        campaign_sha256=digest(args.campaign),
        snapshot_sha256=digest(args.snapshot),
        result_bindings=bindings,
        estimator_configuration=configuration,
        paired_comparisons=paired,
        limitations=[
            "Two independent noise seeds, two frequencies, two heights; finite synthetic domain.",
            "Same physical noise and delivered packets, with stack-specific effective Q/R, priors, height, terrain and magnetic policies.",
            "NEES is a common 15D marginal at each filter's own state epoch. Temporal samples are correlated; no independent chi-square confidence claim.",
            "Native scalar NIS is conditional on observed candidates/updates with incomplete coverage. ESKF NIS is unavailable in this campaign.",
            "Native CPU is not compared: innovation observers and CSV I/O change measured work. ESKF CPU is thread time for instrumented replay, not flight-only WCET.",
            "Capture failures stay in the summary and prevent a complete campaign claim; invalid state/covariance/readiness pairs are explicitly counted.",
            "Once inspected, this campaign is development evidence for future tuning, not an untouched validation set for later changes.",
        ],
    )
    lines = [
        "Matched ESKF / native EKF2 / native EKF3 campaign",
        "",
        f"Completed captures: {len(bindings)}/{len(expected)}; failed: {len(failures)}",
        f"State replays: {summary['state_replays']}; native covariance replays: {summary['native_covariance_replays']}",
        f"Invalid scoring rows: {len(summary['invalid_rows'])}",
        "",
        "Flight RMS: median across capture-level RMS, followed by the worst capture.",
        "scenario estimator               H median/max m    V median/max m   velocity median/max m/s   attitude median/max deg   yaw median/max deg",
    ]
    for scenario, name in itertools.product(plan["scenarios"], plan["filters"]):
        selected = [
            row
            for row in rows
            if (row["scenario"], row["estimator"], row["window"])
            == (scenario, name, "flight")
        ]
        values = [
            [row[metric] for row in selected if row[metric] is not None]
            for metric in METRICS
        ]
        cells = [f"{median(v):.5f}/{max(v):.5f}" if v else "undefined" for v in values]
        lines.append(
            f"{scenario:<10} {name:<21}" + " ".join(f"{cell:>22}" for cell in cells)
        )
    lines += [
        "",
        "Paired flight counts where the left estimator has lower RMS; all capture pairs retained.",
        "scenario left/right                         H     V   vel   att   yaw    valid/declared",
    ]
    for scenario in plan["scenarios"]:
        for left, right in dict.fromkeys((row["left"], row["right"]) for row in paired):
            selected = [
                row
                for row in paired
                if (row["left"], row["right"], row["scenario"], row["window"])
                == (left, right, scenario, "flight")
            ]
            lines.append(
                f"{scenario:<10} {left + '/' + right:<33}"
                + " ".join(f"{row['left_lower']:5}" for row in selected)
                + f"    {selected[0]['valid_pairs']}/{selected[0]['declared_pairs']}"
            )
    lines += ["", "Flight mean common 15D NEES: capture-level median / maximum."]
    for scenario, name in itertools.product(plan["scenarios"], plan["filters"]):
        values = [
            row["mean_nees_15d"]
            for row in rows
            if (row["scenario"], row["estimator"], row["window"])
            == (scenario, name, "flight")
            and row["mean_nees_15d"] is not None
        ]
        value = f"{median(values):.3f} / {max(values):.3f}" if values else "undefined"
        lines.append(f"{scenario:<10} {name:<22} {value}")
    lines += [
        "",
        "ESKF instrumented whole-capture CPU microseconds per IMU tick: median / maximum.",
    ]
    for name in plan["filters"]:
        values = [
            row["cpu_us_per_imu_tick"] for row in timing if row["estimator"] == name
        ]
        if values:
            lines.append(f"{name:<22} {median(values):.3f} / {max(values):.3f}")
    lines += ["", *summary["limitations"]]
    args.output.mkdir(parents=True)
    for filename, records in (
        ("comparison.csv", rows),
        ("native-nis.csv", innovations),
        ("native-nis-summary.csv", innovation_summary(innovations)),
        ("eskf-timing.csv", timing),
        ("transitions.csv", transitions),
    ):
        if records:
            write_csv(args.output / filename, records)
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )
    (args.output / "comparison.txt").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("declaration", "captures", "campaign", "snapshot", "root", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
