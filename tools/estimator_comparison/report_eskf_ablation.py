"""Compare every ESKF ablation condition with its frozen native/control rows."""

import argparse
import itertools
import json
import math
from pathlib import Path
import shutil
from statistics import median

from manifest import digest
from replay_eskf_innovations import VARIANTS
from report_readiness_campaign import METRICS, NATIVE, WINDOWS, indexed, join, write_csv


def validate_candidate(candidate, reference, reference_sha, capture):
    expected = set(itertools.product(VARIANTS, ("gps", "denied", "transition")))
    scores = indexed(candidate["scores"], ("name", "scenario"))
    if (
        not candidate["complete"]
        or candidate["reference_sha256"] != reference_sha
        or candidate["input_sha256"] != capture["audit"]["input_sha256"]
        or set(scores) != expected
    ):
        raise ValueError("Require every candidate condition and its frozen capture")
    originals = indexed(reference["scores"], ("name", "scenario"))
    for key, score in scores.items():
        if any(score["timing"][k] != v for k, v in originals[key]["transport"].items()):
            raise ValueError("Candidate sensor delivery differs from its reference")
        if set(score["consistency"]) != {w[0] for w in reference["mission"]["windows"]}:
            raise ValueError("Missing covariance window")
        if key[0].endswith("_joint") and "full16_covariance" not in score:
            raise ValueError("Missing complete pressure covariance qualification")
    return scores


def candidate_rows(candidate, original_rows, label):
    originals = indexed(original_rows, ("estimator", "scenario", "window"))
    rows = []
    for score in candidate["scores"]:
        for window in WINDOWS:
            accuracy = score[window]
            statistics = score["consistency"][
                "outage" if window == "outage_window" else window
            ]
            original = originals[score["name"], score["scenario"], window]
            rows.append(
                original
                | dict(
                    estimator=label + score["name"],
                    **{metric: accuracy.get(metric) for metric in METRICS},
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
                    state_valid=(
                        accuracy["finite_fraction"] == 1
                        and accuracy["position_valid_fraction"] == 1
                        and accuracy["attitude_valid_fraction"] == 1
                        and accuracy["row_coverage"] >= 0.999
                        and accuracy["nonzero_step_status_rows"] == 0
                        and accuracy["prediction_accepted_fraction"] == 1
                    ),
                    covariance_valid=statistics["valid"],
                    covariance_failure=statistics.get("reason", ""),
                    mean_nees_15d=statistics.get("mean_nees_15d")
                    if statistics["valid"]
                    else None,
                    output_sha256=score["output_sha256"],
                )
            )
    return rows


def paired(rows, pairs, captures):
    values = indexed(rows, ("capture", "estimator", "scenario", "window"))
    result = []
    for left, right in pairs:
        for scenario, window, metric in itertools.product(
            ("gps", "denied", "transition"), WINDOWS, METRICS
        ):
            deltas, percentages, invalid = [], [], []
            for capture in captures:
                pair = [
                    values.get((capture, name, scenario, window))
                    for name in (left, right)
                ]
                if any(
                    row is None
                    or not row["state_valid"]
                    or not row["covariance_valid"]
                    or row["readiness_qualified"] is False
                    or row[metric] is None
                    or not math.isfinite(row[metric])
                    for row in pair
                ):
                    invalid.append(capture)
                    continue
                a, b = (row[metric] for row in pair)
                deltas.append(a - b)
                if b > 0:
                    percentages.append(100 * (a / b - 1))
            result.append(
                dict(
                    left=left,
                    right=right,
                    scenario=scenario,
                    window=window,
                    metric=metric,
                    declared_pairs=len(captures),
                    valid_pairs=len(deltas),
                    invalid_pairs=invalid,
                    left_lower=sum(v < 0 for v in deltas),
                    right_lower=sum(v > 0 for v in deltas),
                    ties=sum(v == 0 for v in deltas),
                    median_left_minus_right=median(deltas) if deltas else None,
                    worst_left_minus_right=max(deltas) if deltas else None,
                    median_percent_change=median(percentages) if percentages else None,
                )
            )
    return result


def verify_build(build, baseline_build, flag):
    if set(build) != set(VARIANTS):
        raise ValueError("Missing ablation binary")
    for name, variant in VARIANTS.items():
        command = build[name]["command"]
        expected = baseline_build["variants"][variant]
        if [arg for arg in command if arg.startswith("-D")] != expected[
            "definitions"
        ] + [flag]:
            raise ValueError("Ablation changes more than the declared option")
        if digest(Path(command[-1])) != build[name]["binary_sha256"]:
            raise ValueError("Candidate binary changed")
        objects = {Path(arg).name: Path(arg) for arg in command if arg.endswith(".o")}
        if {name: digest(path) for name, path in objects.items()} != expected[
            "generated_object_sha256"
        ]:
            raise ValueError("Ablation changed generated Modelica objects")


def report(args):
    if args.output.exists():
        raise ValueError("Choose a fresh review directory")
    declaration_path = args.candidate / "campaign-declaration.json"
    declaration = json.loads(declaration_path.read_text())
    campaign_path = args.candidate / "campaign.json"
    campaign = json.loads(campaign_path.read_text())
    capture_path = args.references / "captures.json"
    captures = json.loads(capture_path.read_text())
    reference_summary = json.loads(args.reference_summary.read_text())
    plan = json.loads(args.reference_declaration.read_text())["held_out_plan"]
    conditions = {
        key[0]: value
        for key, value in indexed(campaign["conditions"], ("condition",)).items()
    }
    expected = {case["name"] for case in captures["cases"]}
    if (
        not campaign["complete"]
        or not captures["complete"]
        or not reference_summary["complete"]
        or set(conditions) != expected
        or not all(c["complete"] for c in conditions.values())
        or set(declaration["conditions"]) != expected
        or digest(capture_path) != declaration["capture_manifest_sha256"]
        or digest(capture_path) != reference_summary["captures_sha256"]
    ):
        raise ValueError("Require the complete declared campaign, retaining failures")
    for name, expected_sha in declaration["source_sha256"].items():
        if digest(args.candidate / "tools" / name) != expected_sha:
            raise ValueError("Frozen replay tool changed")
    build_path = args.candidate / "build.json"
    if digest(build_path) != declaration["build_sha256"]:
        raise ValueError("Candidate build manifest changed")
    verify_build(
        json.loads(build_path.read_text()),
        json.loads(args.baseline_build.read_text()),
        args.flag,
    )
    args.output.mkdir(parents=True)
    rows, bindings, full16, ancillary = [], [], [], []
    for capture in captures["cases"]:
        condition = capture["name"]
        reference_path = args.references / condition / "pilot.json"
        covariance_path = args.references / condition / "covariance.json"
        binding = reference_summary["result_bindings"][condition]
        if (
            digest(reference_path) != binding["pilot_sha256"]
            or digest(covariance_path) != binding["covariance_sha256"]
        ):
            raise ValueError("Frozen native comparison changed")
        reference = json.loads(reference_path.read_text())
        originals = join(
            reference,
            json.loads(covariance_path.read_text()),
            digest(reference_path),
            capture,
            plan,
        )
        candidate_path = args.candidate / "campaign" / condition / "scores.json"
        if digest(candidate_path) != conditions[condition]["sha256"]:
            raise ValueError("Candidate replay evidence changed")
        candidate = json.loads(candidate_path.read_text())
        validate_candidate(candidate, reference, digest(reference_path), capture)
        rows.extend(originals)
        rows.extend(candidate_rows(candidate, originals, args.label))
        for score in candidate["scores"]:
            name, scenario = score["name"], score["scenario"]
            if name.endswith("_joint"):
                path = (
                    args.candidate
                    / "campaign"
                    / condition
                    / scenario
                    / name
                    / "covariance.csv"
                )
                audit = score["full16_covariance"]
                if digest(path) != audit["covariance_sha256"] or audit[
                    "horizon"
                ] != name.startswith("horizon"):
                    raise ValueError("Full pressure covariance binding differs")
                full16.append(
                    dict(
                        capture=condition,
                        estimator=args.label + name,
                        scenario=scenario,
                        **audit,
                    )
                )
            ancillary.append(
                dict(
                    capture=condition,
                    estimator=args.label + name,
                    scenario=scenario,
                    consistency=score["consistency"],
                    timing=score["timing"],
                    transition=score.get("transition"),
                )
            )
        destination = args.output / "cases" / condition
        destination.mkdir(parents=True)
        shutil.copyfile(candidate_path, destination / "candidate.json")
        bindings.append(
            dict(capture=condition, candidate_sha256=digest(candidate_path), **binding)
        )
    pairs = [
        (args.label + name, other) for name in VARIANTS for other in (name, *NATIVE)
    ]
    comparisons = paired(rows, pairs, sorted(expected))
    write_csv(args.output / "comparison.csv", rows)
    write_csv(args.output / "pairs.csv", comparisons)
    (args.output / "ancillary.json").write_text(json.dumps(ancillary, indent=2) + "\n")
    result = dict(
        complete=True,
        captures=len(expected),
        candidate_replays=12 * len(expected),
        invalid_rows=[
            {k: row[k] for k in ("capture", "estimator", "scenario", "window")}
            for row in rows
            if not row["state_valid"] or not row["covariance_valid"]
        ],
        full16_covariance_cases=len(full16),
        full16_covariance_rows=sum(a["rows"] for a in full16),
        full16_covariance=full16,
        bindings=bindings,
        comparisons=comparisons,
        declaration_sha256=digest(declaration_path),
        campaign_sha256=digest(campaign_path),
        reference_summary_sha256=digest(args.reference_summary),
        build_sha256=digest(build_path),
        reporter_sha256=digest(Path(__file__)),
        source_sha256={
            name: digest(Path(__file__).with_name(name))
            for name in (
                "report_eskf_ablation.py",
                "report_readiness_campaign.py",
                "replay_eskf_innovations.py",
                "manifest.py",
            )
        },
        comparison_sha256=digest(args.output / "comparison.csv"),
        pairs_sha256=digest(args.output / "pairs.csv"),
        scope="All declared development captures, windows, metrics and losses retained. Same physical captures and packet delivery, frozen native readiness and common-15 covariance evidence. Native effective Q/R remain stack-specific. New per-sensor NIS is not measured by this report. Total CPU includes warmup and host effects. Existing development captures are not untouched validation or a universal superiority proof.",
    )
    (args.output / "summary.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for option in (
        "candidate",
        "references",
        "reference-summary",
        "reference-declaration",
        "baseline-build",
        "output",
    ):
        parser.add_argument("--" + option, type=Path, required=True)
    parser.add_argument("--flag", default="-DSTATIONARY_IMU_MODEL")
    parser.add_argument("--label", default="stationary_")
    report(parser.parse_args())
