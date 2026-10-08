"""Summarize the parity-qualified public height-mode observations."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


def summarize(root):
    observer = json.loads((root / "observer.json").read_text())
    cases = []
    modes = (
        "range_height",
        "range_terrain",
        "flow_terrain",
        "barometer_height",
        "gps_height",
        "in_air",
        "range_kinematic_consistent",
        "range_fault",
    )
    for scenario in ("gps", "denied", "transition"):
        raw = root / (scenario + "-height.csv")
        binding = next(c for c in observer["results"] if c["scenario"] == scenario)
        if hashlib.sha256(raw.read_bytes()).hexdigest() != binding["height_sha256"]:
            raise ValueError("Observed trace changed")
        with raw.open() as stream:
            rows = [
                r
                for r in csv.DictReader(stream)
                if 120 <= float(r["publication_s"]) < 166.7
            ]
        if not rows or any(
            int(row[key]) not in (0, 1) for row in rows for key in modes
        ):
            raise ValueError("Missing or invalid observed mode flags")
        cases.append(
            dict(
                scenario=scenario,
                rows=len(rows),
                mode_fractions={
                    key: sum(int(row[key]) for row in rows) / len(rows) for key in modes
                },
                height_reference_fractions={
                    str(reference): sum(
                        int(row["height_reference"]) == reference for row in rows
                    )
                    / len(rows)
                    for reference in range(5)
                },
            )
        )
    return dict(
        cases=cases,
        scope="Known seed-911 diagnostic. Public post-update mode flags, not accepted fusion counts, accuracy ablation, or attribution of the vertical RMS difference.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a fresh evidence path")
    args.output.write_text(json.dumps(summarize(args.root), indent=2) + "\n")
