"""Cross-platform orchestration for the modelica_models repository."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
from typing import Sequence

from .common import (
    ToolError,
    executable,
    executable_path,
    repository_environment,
    repository_root,
    require_executable,
    run,
)
from .structure import check as check_structure


EXPORTS = {
    "cubs2-controller": (
        "Vehicles/Cubs2/OuterLoop.mo",
        "Vehicles.Cubs2.OuterLoop",
        "--target",
        "galec-production",
        "artifacts/vehicles/cubs2/controller",
    ),
    "cubs2-plant": (
        "Vehicles/Cubs2/AvionicsPlant.mo",
        "Vehicles.Cubs2.AvionicsPlant",
        "--target",
        "fmi3",
        "artifacts/vehicles/cubs2/plant",
    ),
    "rdd2-controller": (
        "Vehicles/Rdd2/Controller.mo",
        "Vehicles.Rdd2.Controller",
        "--target",
        "galec-production",
        "artifacts/vehicles/rdd2/controller",
    ),
    "rdd2-estimator": (
        "Vehicles/Rdd2/NavigationEstimator.mo",
        "Vehicles.Rdd2.NavigationEstimator",
        "--target",
        "galec-production",
        "artifacts/vehicles/rdd2/estimator",
    ),
    "rdd2-plant": (
        "Vehicles/Rdd2/Plant.mo",
        "Vehicles.Rdd2.Plant",
        "--target",
        "fmi3",
        "artifacts/vehicles/rdd2/plant",
    ),
}

QUALIFICATIONS = {
    "cubs2": "Vehicles/Cubs2/Test/run_qualification.py",
    "rdd2": "Vehicles/Rdd2/Test/run_waypoint_qualification.py",
}


def script(root: Path, relative: str, arguments: Sequence[str]) -> None:
    interpreter = os.environ.get("MODELICA_MODELS_PYTHON") or sys.executable
    run(
        [interpreter, root / relative, *arguments],
        root,
        environment=repository_environment(root),
    )


def rumoca_version(root: Path) -> None:
    cli = require_executable("MODELICA_MODELS_RUMOCA", "rumoca", "Rumoca")
    cli_result = subprocess.run(
        [str(cli), "--version"], cwd=root, text=True, capture_output=True, check=True
    ).stdout.strip()
    cli_version = cli_result.rsplit(" ", 1)[-1]
    try:
        import rumoca
    except ImportError as error:
        raise ToolError(
            "Rumoca's Python module is unavailable; install the matching binding "
            "or use the optional Nix environment"
        ) from error
    python_version = str(rumoca.version())
    if cli_version != python_version:
        raise ToolError(
            f"Rumoca CLI version {cli_version} does not match Python version {python_version}"
        )
    build_info = subprocess.run(
        [str(cli), "build-info"], cwd=root, text=True, capture_output=True, check=False
    )
    if build_info.returncode == 0 and hasattr(rumoca, "build_identity"):
        cli_identity = build_info.stdout.strip()
        python_identity = str(rumoca.build_identity())
        if cli_identity != python_identity:
            raise ToolError(
                f"Rumoca CLI build {cli_identity} does not match Python build {python_identity}"
            )
        print(f"Rumoca CLI/Python build: {cli_identity}")
    else:
        print(f"Rumoca CLI/Python version: {cli_version}")


def qualify(root: Path, vehicle: str, arguments: Sequence[str]) -> None:
    rumoca_version(root)
    vehicles = QUALIFICATIONS if vehicle == "all" else {vehicle: QUALIFICATIONS[vehicle]}
    for relative in vehicles.values():
        script(root, relative, arguments)


def export(root: Path, name: str, arguments: Sequence[str]) -> None:
    rumoca = require_executable("MODELICA_MODELS_RUMOCA", "rumoca", "Rumoca")
    model_file, model_name, format_option, format_name, output = EXPORTS[name]
    source = root / model_file
    if not source.is_file():
        raise ToolError(f"model source not found: {source}")
    (root / output).parent.mkdir(parents=True, exist_ok=True)
    run(
        [
            rumoca,
            "compile",
            model_file,
            "--source-root",
            ".",
            "--model",
            model_name,
            format_option,
            format_name,
            "--output",
            output,
            *arguments,
        ],
        root,
        environment=repository_environment(root),
    )


def mission_report(root: Path, mode: str, arguments: Sequence[str]) -> None:
    environment = repository_environment(root)
    interpreter = os.environ.get("MODELICA_MODELS_PYTHON") or sys.executable
    environment.setdefault("RDD2_MISSION_PYTHON", str(Path(interpreter).resolve()))
    for variable, fallback in (
        ("RDD2_MISSION_GIT", "git"),
        ("RDD2_MISSION_OMC", "omc"),
    ):
        configured = environment.get(variable) or executable_path(fallback)
        if configured:
            environment[variable] = str(configured)
    run(
        [
            interpreter,
            root / "Vehicles/Rdd2/Test/run_mission_plot.py",
            *arguments,
            "--mode",
            mode,
        ],
        root,
        environment=environment,
    )


def doctor(root: Path) -> int:
    print(f"repository: {root}")
    missing = False
    for label, variable, fallback in (
        ("Rumoca", "MODELICA_MODELS_RUMOCA", "rumoca"),
        ("OpenModelica", "MODELICA_MODELS_OMC", "omc"),
        ("Docker", "MODELICA_MODELS_DOCKER", "docker"),
    ):
        configured = executable(variable, fallback)
        resolved = executable_path(configured)
        state = str(resolved) if resolved else "not found"
        print(f"{label}: {state}")
        if label == "Rumoca" and resolved is None:
            missing = True
    try:
        import rumoca

        print(f"Rumoca Python: {rumoca.version()}")
    except ImportError:
        print("Rumoca Python: not found")
        missing = True
    print(f"Python: {sys.executable}")
    if not executable_path(executable("MODELICA_MODELS_OMC", "omc")) and not executable_path(
        executable("MODELICA_MODELS_DOCKER", "docker")
    ):
        print("OpenModelica runner: unavailable (install omc or Docker)")
        missing = True
    return 1 if missing else 0


def parser() -> argparse.ArgumentParser:
    command_parser = argparse.ArgumentParser(description=__doc__)
    subparsers = command_parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("check-structure", help="validate Modelica package metadata")
    test_parser = subparsers.add_parser("test", help="run compiler and plot checks")
    test_parser.add_argument(
        "suite", nargs="?", default="test", choices=("test", "ci", "omc", "rumoca", "plots")
    )
    qualify_parser = subparsers.add_parser("qualify", help="run vehicle qualification")
    qualify_parser.add_argument("vehicle", choices=(*QUALIFICATIONS, "all"))
    export_parser = subparsers.add_parser("export", help="export a deployable model")
    export_parser.add_argument("model", choices=tuple(EXPORTS))
    report_parser = subparsers.add_parser("mission-report", help="render a receipted RDD2 mission trace")
    report_parser.add_argument("mode", choices=("optical", "gps"))
    subparsers.add_parser("trajectory-compare", help="compare canonical trajectory logs")
    subparsers.add_parser("rumoca-version", help="verify Rumoca CLI/Python identity")
    subparsers.add_parser("doctor", help="show repository tool availability")
    return command_parser


def main(arguments: Sequence[str] | None = None) -> int:
    command_parser = parser()
    options, forwarded = command_parser.parse_known_args(arguments)
    if forwarded and forwarded[0] == "--":
        forwarded = forwarded[1:]
    try:
        root = repository_root()
        if options.command == "check-structure":
            if forwarded:
                raise ToolError("check-structure accepts no additional arguments")
            check_structure(root)
        elif options.command == "test":
            script(root, "tools/ci.py", [options.suite, *forwarded])
        elif options.command == "qualify":
            qualify(root, options.vehicle, forwarded)
        elif options.command == "export":
            export(root, options.model, forwarded)
        elif options.command == "mission-report":
            mission_report(root, options.mode, forwarded)
        elif options.command == "trajectory-compare":
            script(root, "tools/trajectory_compare.py", forwarded)
        elif options.command == "rumoca-version":
            if forwarded:
                raise ToolError("rumoca-version accepts no additional arguments")
            rumoca_version(root)
        else:
            if forwarded:
                raise ToolError("doctor accepts no additional arguments")
            return doctor(root)
    except (OSError, subprocess.CalledProcessError, ToolError) as error:
        print(f"modelica-models: {error}", file=sys.stderr)
        return 1
    return 0
