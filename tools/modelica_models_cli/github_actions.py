"""Small cross-platform helpers used only by the GitHub Actions workflow."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import Sequence

from .common import ToolError, repository_root


def require_nonempty(path: Path) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise ToolError(f"expected nonempty artifact: {path}")


def verify(vehicle: str, figure: str, report: str) -> None:
    root = repository_root()
    directory = root / "artifacts" / "vehicles" / vehicle
    require_nonempty(directory / figure)
    require_nonempty(directory / report)


def summary(
    vehicle: str,
    label: str,
    report: str,
    artifact_url: str,
    qualification_outcome: str,
    report_outcome: str,
) -> None:
    destination = os.environ.get("GITHUB_STEP_SUMMARY")
    if not destination:
        raise ToolError("GITHUB_STEP_SUMMARY is unset")
    lines = [
        f"## {label} mission qualification",
        "",
        f"Qualification: **{qualification_outcome}**  ",
        f"Plot and report: **{report_outcome}**",
        "",
    ]
    if artifact_url:
        lines.extend(
            (f"[Download the plots and self-contained HTML report]({artifact_url})", "")
        )
    lines.append(f"Open **{report}** from the artifact to view the qualification plots and checks.")
    if vehicle == "rdd2":
        lines.extend(
            (
                "",
                "The RDD2 artifact contains directly viewable PNGs:",
                "",
                "- `rdd2-waypoint-summary.png` (top-down trajectory)",
                "- `rdd2-position-history.png` (east/north/altitude versus time)",
                "- `rdd2-velocity-history.png` (ENU velocity versus time)",
                "- `rdd2-nees-nis-consistency.png` (ESKF covariance consistency)",
                "- `rdd2-estimator-consistency.png` (ESKF/UKF NEES and NIS)",
                "- `rdd2-estimator-comparison.png` (accuracy and runtime comparison)",
            )
        )
        details = repository_root() / "artifacts/vehicles/rdd2/waypoint-summary.md"
        if details.is_file() and details.stat().st_size:
            lines.extend(("", details.read_text(encoding="utf-8").rstrip()))
    with Path(destination).open("a", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


def main(arguments: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    verify_parser = subparsers.add_parser("verify")
    verify_parser.add_argument("--vehicle", required=True)
    verify_parser.add_argument("--figure", required=True)
    verify_parser.add_argument("--report", required=True)
    summary_parser = subparsers.add_parser("summary")
    summary_parser.add_argument("--vehicle", required=True)
    summary_parser.add_argument("--label", required=True)
    summary_parser.add_argument("--report", required=True)
    summary_parser.add_argument("--artifact-url", default="")
    summary_parser.add_argument("--qualification-outcome", required=True)
    summary_parser.add_argument("--report-outcome", required=True)
    options = parser.parse_args(arguments)
    try:
        if options.command == "verify":
            verify(options.vehicle, options.figure, options.report)
        else:
            summary(
                options.vehicle,
                options.label,
                options.report,
                options.artifact_url,
                options.qualification_outcome,
                options.report_outcome,
            )
    except (OSError, ToolError) as error:
        print(f"modelica-models github-actions: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
