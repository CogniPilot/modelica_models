"""Isolate native noise groups on one declared frozen flight condition."""

import json
from pathlib import Path
from types import SimpleNamespace

from compare_exposure import digest
from compare_native_aiding_noise import arguments, run
from native_aiding_noise import PARAMETER_GROUPS


def diagnose(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose fresh owned work and evidence paths")
    if args.condition is None or args.noise_groups is not None:
        raise ValueError(
            "Declare one frozen condition; this diagnostic fixes all groups"
        )
    full_reference = json.loads(args.full_reference.read_text())
    reference = json.loads(args.reference.read_text())
    if (
        not full_reference.get("complete")
        or full_reference["reference_sha256"] != digest(args.reference)
        or full_reference["observer_reference_sha256"]
        != digest(args.observer_reference)
    ):
        raise ValueError("Require the completed full-profile campaign")
    variants = {"published": [], "full": None}
    for group in PARAMETER_GROUPS:
        variants["only_" + group] = [group]
        variants["without_" + group] = [
            name for name in PARAMETER_GROUPS if name != group
        ]
    args.work.mkdir(parents=True)
    result = dict(
        reference_sha256=digest(args.reference),
        full_reference_sha256=digest(args.full_reference),
        condition=args.condition,
        declared_variants=variants,
        scores=[],
        scope="One-condition parameter-group isolation, not a tuning search or a validated comparison baseline. Fresh published/full controls must reproduce frozen state CSVs. No physical data, arrival or native core changes.",
    )
    for variant, groups in variants.items():
        evidence = args.work / (variant + ".json")
        options = SimpleNamespace(
            **(
                vars(args)
                | dict(
                    work=args.work / variant,
                    output=evidence,
                    noise_groups=groups,
                )
            )
        )
        run(options)
        campaign = json.loads(evidence.read_text())
        if not campaign.get("complete"):
            raise ValueError("Incomplete native group diagnostic")
        for row in campaign["scores"]:
            control_identical = None
            if variant in ("published", "full"):
                frozen = (reference if variant == "published" else full_reference)[
                    "scores"
                ]
                matches = [
                    previous
                    for previous in frozen
                    if all(
                        previous[key] == row[key]
                        for key in (
                            "name",
                            "seed",
                            "frequency",
                            "climb_height_m",
                            "scenario",
                        )
                    )
                    and (variant == "full" or previous["explicit_exposure"])
                ]
                if len(matches) != 1:
                    raise ValueError("Missing or duplicated frozen diagnostic control")
                expected = (
                    matches[0]["output_sha256"]
                    if variant == "published"
                    else matches[0]["native"]["output_sha256"]
                )
                control_identical = row["native"]["output_sha256"] == expected
                if not control_identical:
                    raise ValueError(
                        "Fresh diagnostic control changed frozen native states"
                    )
            result["scores"].append(
                dict(
                    variant=variant,
                    selected_groups=groups,
                    campaign_sha256=digest(evidence),
                    frozen_control_identical=control_identical,
                    **row,
                )
            )
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    expected = len(variants) * (1 if args.estimator else 2)
    if len(result["scores"]) != expected:
        raise ValueError("Missing declared native group diagnostic")
    result["complete"] = True
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = arguments()
    parser.description = __doc__
    parser.add_argument("--full-reference", type=Path, required=True)
    diagnose(parser.parse_args())
