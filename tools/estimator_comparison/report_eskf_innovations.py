"""Bind per-sensor ESKF NIS to all cases of a frozen matched campaign."""

import argparse
import csv
import itertools
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np

from eskf_innovations import check
from manifest import digest
from replay_eskf_innovations import VARIANTS


def report(args):
    if args.output.exists():
        raise ValueError("Choose a fresh review directory")
    campaign_path = args.qualification / "campaign.json"
    campaign = json.loads(campaign_path.read_text())
    capture_path = args.references / "captures.json"
    captures = json.loads(capture_path.read_text())
    expected = {c["name"] for c in captures["cases"]}
    conditions = campaign["conditions"]
    if (
        not campaign["complete"]
        or not captures["complete"]
        or len(conditions) != len(expected)
        or {c["name"] for c in conditions} != expected
        or not all(c["complete"] for c in conditions)
    ):
        raise ValueError("Require every declared capture, including any failures")
    args.output.mkdir(parents=True)
    records, qualified, analyses = [], [], []
    combinations = set(itertools.product(("gps", "denied", "transition"), VARIANTS))
    for condition in sorted(expected):
        reference_path = args.references / condition / "pilot.json"
        reference = json.loads(reference_path.read_text())
        directory = args.qualification / "campaign" / condition
        qualification_path = directory / "qualification.json"
        qualification = json.loads(qualification_path.read_text())
        cases = qualification["cases"]
        if (
            not qualification["complete"]
            or qualification["reference_sha256"] != digest(reference_path)
            or len(cases) != len(combinations)
            or {(c["scenario"], c["name"]) for c in cases} != combinations
        ):
            raise ValueError("Qualification does not cover every frozen case")
        for name, expected_sha in qualification["source_sha256"].items():
            if digest(Path(__file__).with_name(name)) != expected_sha:
                raise ValueError("Qualification tool source differs from its evidence")
        destination = args.output / "cases" / condition
        destination.mkdir(parents=True)
        shutil.copyfile(qualification_path, destination / "qualification.json")
        qualified.append(dict(condition=condition, sha256=digest(qualification_path)))
        for case in cases:
            scenario, name = case["scenario"], case["name"]
            frozen = next(
                s
                for s in reference["scores"]
                if s["scenario"] == scenario and s["name"] == name
            )
            executions = case["executions"]
            if (
                not case["state_parity"]
                or not case["full_covariance_parity"]
                or case["input_sha256"] != reference["input_sha256"]
                or case["baseline_binary_sha256"] != reference["binary_sha256"][name]
                or any(
                    e["states_sha256"] != frozen["output_sha256"]
                    for e in executions.values()
                )
                or executions["baseline"]["covariance_sha256"]
                != executions["observer"]["covariance_sha256"]
            ):
                raise ValueError("State/covariance/input parity is unproven")
            raw = directory / "qualification" / scenario / name / "innovations.csv"
            if digest(raw) != case["innovations_sha256"]:
                raise ValueError("Actual observation trace differs from qualification")
            output = destination / f"{scenario}-{name}.json"
            analysis = check(
                SimpleNamespace(
                    innovations=raw,
                    output=output,
                    windows=reference["mission"]["windows"],
                )
            )
            analyses.append(
                dict(
                    condition=condition,
                    scenario=scenario,
                    name=name,
                    sha256=digest(output),
                    raw_rows=analysis["raw_rows"],
                    unique_candidates=analysis["unique_candidates"],
                    repeated_evaluations=analysis["repeated_identical_evaluations"],
                )
            )
            for row in analysis["records"]:
                records.append(
                    dict(condition=condition, scenario=scenario, name=name, **row)
                )
    (args.output / "records.json").write_text(json.dumps(records, indent=2) + "\n")
    groups = {}
    for row in records:
        key = tuple(row[k] for k in ("scenario", "name", "sensor", "window"))
        groups.setdefault(key, []).append(row)
    summaries = []
    for key, rows in sorted(groups.items()):
        values = [
            r["all_candidates"]["mean_joint_nis_per_dimension"]
            for r in rows
            if r["computed_rows"]
        ]
        summaries.append(
            dict(
                **dict(zip(("scenario", "name", "sensor", "window"), key)),
                captures=len(rows),
                captures_with_computed_nis=len(values),
                attempted_candidates=sum(r["attempted_rows"] for r in rows),
                prelinear_rejected_candidates=sum(
                    r["prelinear_rejected_rows"] for r in rows
                ),
                accepted_candidates=sum(r["accepted_rows"] for r in rows),
                median_capture_mean_joint_nis_per_dimension=float(np.median(values))
                if values
                else None,
                minimum_capture_mean_joint_nis_per_dimension=min(values)
                if values
                else None,
                maximum_capture_mean_joint_nis_per_dimension=max(values)
                if values
                else None,
            )
        )
    with (args.output / "summary.csv").open("w") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(summaries[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(summaries)
    result = dict(
        complete=True,
        captures=len(expected),
        replay_cases=len(analyses),
        state_and_full_covariance_parity_cases=len(analyses),
        raw_evaluations=sum(a["raw_rows"] for a in analyses),
        unique_candidates=sum(a["unique_candidates"] for a in analyses),
        invalid_observations=0,
        capture_manifest_sha256=digest(capture_path),
        qualification_campaign_sha256=digest(campaign_path),
        qualifications=qualified,
        analyses=analyses,
        source_sha256={
            name: digest(Path(__file__).with_name(name))
            for name in (
                "report_eskf_innovations.py",
                "eskf_innovations.py",
                "replay_eskf_innovations.py",
                "instrument_eskf_innovations.py",
                "eskf_innovation_dump.h",
            )
        },
        summary_sha256=digest(args.output / "summary.csv"),
        records_sha256=digest(args.output / "records.json"),
        scope="Every declared case retained. Joint vector NIS divided by dimension, summarized first within capture then across eight capture conditions (only two independent sensor seeds). Candidate/accepted/rejected statistics are separate in records.json. Identical repeated generated evaluations are audited and counted once; raw trace hashes retain every evaluation. These results do not establish matching effective native Q/R, independent temporal trials, or superiority of an estimator. States and all raw covariance output bytes match their frozen baselines.",
    )
    (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--references", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    report(parser.parse_args())
