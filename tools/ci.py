#!/usr/bin/env python3
"""Cross-platform orchestration for Modelica tests and planning plots."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from typing import NoReturn, Sequence


OPENMODELICA_IMAGE = "openmodelica/openmodelica:v1.27.0-minimal"

# Packaged environments may provide absolute defaults. Native use relies on
# environment overrides or executable discovery through PATH.
DEFAULT_DOCKER = None
DEFAULT_OMC = None
DEFAULT_RUMOCA = None


class TaskError(RuntimeError):
    """An expected task failure with a concise user-facing message."""


def main(arguments: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "command",
        nargs="?",
        default="test",
        choices=("test", "ci", "omc", "rumoca", "plots"),
        help="task to run (default: test)",
    )
    options = parser.parse_args(arguments)

    try:
        repository = find_repository_root(Path.cwd())
        if options.command in ("test", "ci", "omc"):
            run_openmodelica_tests(repository)
        if options.command in ("test", "ci", "rumoca"):
            run_rumoca_tests(repository)
        if options.command in ("test", "ci", "plots"):
            run_planning_plots(repository)
    except (OSError, subprocess.CalledProcessError, TaskError) as error:
        print(f"modelica-models-ci: {error}", file=sys.stderr)
        return 1
    return 0


def find_repository_root(start: Path) -> Path:
    for directory in (start, *start.parents):
        if (directory / "Tests/run.mos").is_file() and (
            directory / "Vehicles/package.mo"
        ).is_file():
            return directory
    raise TaskError(f"could not find the repository root above {start}")


def run_openmodelica_tests(repository: Path) -> None:
    print("==> OpenModelica assertion suite", flush=True)
    run_omc_script(repository, "Tests/run.mos")
    run_omc_script(repository, "Tests/run-loglinear.mos")
    run_omc_script(repository, "Tests/run-position-loop.mos")
    run_omc_script(repository, "Tests/run-manual-guidance.mos")
    run_omc_script(repository, "Tests/check-vehicle-boundaries.mos")


def run_rumoca_tests(repository: Path) -> None:
    print("==> Rumoca compiler checks", flush=True)
    rumoca = program("MODELICA_MODELS_RUMOCA", DEFAULT_RUMOCA, "rumoca")
    require_program(rumoca, "Rumoca")

    with tempfile.TemporaryDirectory(prefix="modelica-models-rumoca-") as temporary:
        output = Path(temporary)
        run_command(
            [
                rumoca,
                "compile",
                "Tests/package.mo",
                "--model",
                "Tests.All",
                "--source-root",
                str(repository),
                "--emit",
                "dae-json",
                "--output",
                str(output / "Tests.All.dae.json"),
            ],
            repository,
            "Rumoca DAE lowering for Tests.All",
        )
        for model, filename in (
            ("Tests.LieGroupTests.SO2", "SO2.html"),
            ("Tests.LieGroupTests.SE2", "SE2.html"),
        ):
            run_command(
                [
                    rumoca,
                    "sim",
                    "Tests/package.mo",
                    "--model",
                    model,
                    "--source-root",
                    str(repository),
                    "--t-end",
                    "0.0",
                    "--output",
                    str(output / filename),
                ],
                repository,
                f"Rumoca simulation for {model}",
            )

        named_models = (
            (
                "Vehicles/Cubs2/OuterLoop.mo",
                "Vehicles.Cubs2.OuterLoop",
                "--target",
                "galec-production",
                "cubs2-controller",
            ),
            (
                "Vehicles/Cubs2/AvionicsPlant.mo",
                "Vehicles.Cubs2.AvionicsPlant",
                "--emit",
                "dae-json",
                "cubs2-plant.dae.json",
            ),
            (
                "Vehicles/Cubs2/AvionicsSystem.mo",
                "Vehicles.Cubs2.AvionicsSystem",
                "--emit",
                "dae-json",
                "cubs2-avionics-system.dae.json",
            ),
            (
                "Vehicles/Cubs2/Test/Scenarios.mo",
                "Vehicles.Cubs2.Test.Scenarios.Mission",
                "--emit",
                "dae-json",
                "cubs2-mission.dae.json",
            ),
            (
                "Vehicles/Rdd2/Controller.mo",
                "Vehicles.Rdd2.Controller",
                "--target",
                "galec-production",
                "rdd2-controller",
            ),
            (
                "Vehicles/Rdd2/NavigationEstimator.mo",
                "Vehicles.Rdd2.NavigationEstimator",
                "--target",
                "galec-production",
                "rdd2-estimator",
            ),
            (
                "Tests/StrapdownEstimatorInterfaceTests.mo",
                "Tests.StrapdownEstimatorInterfaceTests",
                "--emit",
                "dae-json",
                "strapdown-estimator-interface.dae.json",
            ),
            (
                "Estimation/StrapdownINS/UKF/Estimator.mo",
                "Estimation.StrapdownINS.UKF.Estimator",
                "--target",
                "galec-production",
                "strapdown-ukf-estimator",
            ),
            (
                "Vehicles/Rdd2/Plant.mo",
                "Vehicles.Rdd2.Plant",
                "--emit",
                "dae-json",
                "rdd2-plant.dae.json",
            ),
            (
                "Vehicles/Rdd2/AvionicsSystem.mo",
                "Vehicles.Rdd2.AvionicsSystem",
                "--emit",
                "dae-json",
                "rdd2-avionics-system.dae.json",
            ),
            (
                "Vehicles/Rdd2/Test/WaypointMission.mo",
                "Vehicles.Rdd2.Test.WaypointMission",
                "--emit",
                "dae-json",
                "rdd2-waypoint-mission.dae.json",
            ),
            (
                "Vehicles/Rdd2/Test/ManualFlightMission.mo",
                "Vehicles.Rdd2.Test.ManualFlightMission",
                "--emit",
                "dae-json",
                "rdd2-manual-flight-mission.dae.json",
            ),
        )
        for model_file, model_name, option, format_name, artifact in named_models:
            run_command(
                [
                    rumoca,
                    "compile",
                    model_file,
                    "--model",
                    model_name,
                    "--source-root",
                    str(repository),
                    option,
                    format_name,
                    "--output",
                    str(output / artifact),
                ],
                repository,
                f"Rumoca {format_name} compile for {model_name}",
            )


def run_planning_plots(repository: Path) -> None:
    print("==> Planning trajectory plots", flush=True)
    artifact_directory = repository / "artifacts/planning"
    artifact_directory.mkdir(parents=True, exist_ok=True)
    remove_files(artifact_directory)

    run_omc_script(repository, "Tests/plot-planning.mos")
    render_planning_plots(artifact_directory)

    expected = (
        "dubins_families_res.csv",
        "dubins_aircraft_flightplan_res.csv",
        "dubins_polynomial_examples_res.csv",
        "dubins_families.png",
        "dubins_aircraft_flightplan.png",
        "dubins_polynomial_continuity.png",
        "dubins_bank_angle.png",
        "dubins_polynomial_paper_examples.png",
    )
    for filename in expected:
        require_nonempty_file(artifact_directory / filename)
    remove_build_products(artifact_directory)


def run_omc_script(repository: Path, script: str) -> None:
    docker = program("MODELICA_MODELS_DOCKER", DEFAULT_DOCKER, "docker")
    if docker_daemon_is_available(docker):
        command = [docker, "run", "--rm"]
        if hasattr(os, "getuid") and hasattr(os, "getgid"):
            command.extend(("--user", f"{os.getuid()}:{os.getgid()}"))
        command.extend(
            (
                "--volume",
                f"{repository}:/workspace",
                "--workdir",
                "/workspace",
                "--env",
                "OPENMODELICALIBRARY=/workspace:/root/.openmodelica/libraries",
                OPENMODELICA_IMAGE,
                "omc",
                script,
            )
        )
        run_command(command, repository, f"OpenModelica script {script}")
        return

    omc = program("MODELICA_MODELS_OMC", DEFAULT_OMC, "omc")
    if executable_exists(omc):
        environment = os.environ.copy()
        default_library = Path.home() / ".openmodelica" / "libraries"
        inherited_library = environment.get(
            "OPENMODELICALIBRARY", str(default_library)
        )
        environment["OPENMODELICALIBRARY"] = os.pathsep.join(
            (str(repository), inherited_library)
        )
        run_command(
            (omc, script),
            repository,
            f"OpenModelica script {script}",
            environment,
        )
        return

    raise TaskError(
        "OpenModelica requires a working Docker daemon or a local 'omc' executable"
    )


def run_command(
    command: Sequence[str],
    working_directory: Path,
    label: str,
    environment: dict[str, str] | None = None,
) -> None:
    print(f"    {label}", flush=True)
    subprocess.run(
        command,
        cwd=working_directory,
        env=environment,
        check=True,
    )


def program(environment: str, packaged: str | None, fallback: str) -> str:
    return os.environ.get(environment) or packaged or fallback


def executable_exists(executable: str) -> bool:
    path = Path(executable)
    return path.is_file() if path.parent != Path(".") else shutil.which(executable) is not None


def require_program(executable: str, label: str) -> None:
    if not executable_exists(executable):
        raise TaskError(f"{label} executable not found: {executable}")


def docker_daemon_is_available(docker: str) -> bool:
    if not executable_exists(docker):
        return False
    return (
        subprocess.run(
            (docker, "info"),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        ).returncode
        == 0
    )


def remove_files(directory: Path) -> None:
    for path in directory.iterdir():
        if path.is_file():
            path.unlink()


def remove_build_products(directory: Path) -> None:
    for path in directory.iterdir():
        if path.is_file() and path.suffix not in (".csv", ".png"):
            path.unlink()


def require_nonempty_file(path: Path) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise TaskError(f"expected nonempty artifact {path}")


def read_csv_columns(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise TaskError(f"CSV file has no header: {path}")
        columns = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in reader.fieldnames:
                columns[name].append(float(row[name]))
    return columns


def render_planning_plots(directory: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise TaskError(
            "planning plots require Matplotlib; install the project tools with "
            "'python -m pip install -e .'"
        ) from error

    families = read_csv_columns(directory / "dubins_families_res.csv")
    figure, axes = plt.subplots(figsize=(10, 6.25), dpi=160, layout="constrained")
    for name in ("lsl", "rsr", "lsr", "rsl", "rlr", "lrl"):
        axes.plot(families[f"{name}.x"], families[f"{name}.y"], linewidth=2, label=name.upper())
    configure_path_axes(axes, "All Six Dubins Path Families")
    axes.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    figure.savefig(directory / "dubins_families.png")
    plt.close(figure)

    trajectory = read_csv_columns(directory / "dubins_aircraft_flightplan_res.csv")
    leg_indices = sorted(
        int(match.group(1))
        for name in trajectory
        if (match := re.fullmatch(r"leg\[(\d+)\]\.nominalX", name))
    )
    if not leg_indices:
        raise TaskError("flight-plan CSV contains no trajectory legs")
    legs = tuple(f"leg[{index}]" for index in leg_indices)
    leg_lengths = [trajectory[f"{leg}.nominalPathDistance"][-1] for leg in legs]
    waypoint_distances = [0.0]
    for leg_length in leg_lengths:
        waypoint_distances.append(waypoint_distances[-1] + leg_length)
    closed_flightplan = math.hypot(
        trajectory[f"{legs[0]}.nominalX"][0]
        - trajectory[f"{legs[-1]}.nominalX"][-1],
        trajectory[f"{legs[0]}.nominalY"][0]
        - trajectory[f"{legs[-1]}.nominalY"][-1],
    ) < 1.0e-8
    figure, axes = plt.subplots(figsize=(10, 6.25), dpi=160, layout="constrained")
    for index, leg in enumerate(legs):
        axes.plot(
            trajectory[f"{leg}.nominalX"],
            trajectory[f"{leg}.nominalY"],
            color="#d95f02",
            linewidth=2.5,
            linestyle="--",
            label="Dubins plan (RLR/LRL disabled)" if index == 0 else None,
        )
        axes.plot(
            trajectory[f"{leg}.smoothX"],
            trajectory[f"{leg}.smoothY"],
            color="#1f78b4",
            linewidth=2.5,
            label="flyable Dubins-polynomial flight plan" if index == 0 else None,
        )
        final_waypoint = len(legs) + 1
        waypoint_label = (
            f"WP1 / WP{final_waypoint}"
            if index == 0 and closed_flightplan
            else f"WP{index + 1}"
        )
        add_boundary_pose(axes, trajectory, leg, 0, waypoint_label)
    if not closed_flightplan:
        add_boundary_pose(
            axes, trajectory, legs[-1], -1, f"WP{len(legs) + 1}"
        )
    configure_path_axes(
        axes, "Figure-Eight Flight Plan with Coincident Center Waypoints"
    )
    axes.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    figure.savefig(directory / "dubins_aircraft_flightplan.png")
    plt.close(figure)

    figure, axes = plt.subplots(
        2, 1, figsize=(10, 6.25), dpi=160, sharex=True, layout="constrained"
    )
    figure.suptitle(f"{len(legs)}-Leg Flyable Trajectory Continuity")
    for index, leg in enumerate(legs):
        mission_distance = [
            waypoint_distances[index] + value
            for value in trajectory[f"{leg}.nominalPathDistance"]
        ]
        axes[0].plot(
            mission_distance,
            trajectory[f"{leg}.curvature"],
            color="#1f78b4",
            linewidth=2,
        )
        axes[1].plot(
            mission_distance,
            trajectory[f"{leg}.curvatureDerivative"],
            color="#1f78b4",
            linewidth=2,
        )
    axes[0].set_title("Continuous curvature")
    axes[0].set_ylabel("curvature")
    axes[1].set_title("Continuous curvature derivative")
    axes[1].set_xlabel("nominal mission distance")
    axes[1].set_ylabel("curvature derivative")
    for axes_item in axes:
        axes_item.grid(True, color="#d9d9d9")
        for waypoint_distance in waypoint_distances[1:-1]:
            axes_item.axvline(
                waypoint_distance,
                color="#777777",
                linewidth=0.8,
                linestyle=":",
            )
    figure.savefig(directory / "dubins_polynomial_continuity.png")
    plt.close(figure)

    figure, axes = plt.subplots(figsize=(10, 6.25), dpi=160, layout="constrained")
    for index, leg in enumerate(legs):
        mission_distance = [
            waypoint_distances[index] + value
            for value in trajectory[f"{leg}.nominalPathDistance"]
        ]
        bank_angle_degrees = [
            math.degrees(value)
            for value in trajectory[f"{leg}.coordinatedBankAngle"]
        ]
        axes.plot(
            mission_distance,
            bank_angle_degrees,
            color="#1f78b4",
            linewidth=2,
        )
    axes.axhline(0.0, color="#202020", linewidth=1)
    for waypoint_distance in waypoint_distances[1:-1]:
        axes.axvline(
            waypoint_distance,
            color="#777777",
            linewidth=0.8,
            linestyle=":",
        )
    axes.set_title("Coordinated Bank-Angle Command (Positive Left, Negative Right)")
    axes.set_xlabel("nominal mission distance")
    axes.set_ylabel("bank angle [deg]")
    axes.grid(True, color="#d9d9d9")
    figure.savefig(directory / "dubins_bank_angle.png")
    plt.close(figure)

    examples = read_csv_columns(directory / "dubins_polynomial_examples_res.csv")
    figure, axes = plt.subplots(
        2, 2, figsize=(10, 8), dpi=160, layout="constrained"
    )
    for case_index, axes_item in enumerate(axes.flat, start=1):
        prefix = f"case{case_index}"
        axes_item.plot(
            examples[f"{prefix}.nominalX"],
            examples[f"{prefix}.nominalY"],
            color="#202020",
            linewidth=2.2,
            linestyle="--",
            label="Dubins reference",
        )
        axes_item.plot(
            examples[f"{prefix}.smoothX"],
            examples[f"{prefix}.smoothY"],
            color="#2040d0",
            linewidth=2.4,
            label="optimized polynomial",
        )
        segment_index = examples[f"{prefix}.nominalSegmentIndex"]
        for junction_segment in (2, 3):
            junction_index = next(
                (
                    sample_index
                    for sample_index, value in enumerate(segment_index)
                    if int(value) == junction_segment
                ),
                None,
            )
            if junction_index is not None:
                axes_item.scatter(
                    (examples[f"{prefix}.nominalX"][junction_index],),
                    (examples[f"{prefix}.nominalY"][junction_index],),
                    color="#202020",
                    marker="x",
                    s=45,
                    zorder=5,
                )
        axes_item.set_title(f"({chr(96 + case_index)})")
        axes_item.set_aspect("equal", adjustable="datalim")
        axes_item.grid(True, color="#d9d9d9", linestyle=":")
    axes[0, 0].legend(loc="best")
    figure.suptitle("Jointly Optimized Dubins-Polynomial Examples")
    figure.supxlabel("east position")
    figure.supylabel("north position")
    figure.savefig(directory / "dubins_polynomial_paper_examples.png")
    plt.close(figure)


def configure_path_axes(axes: object, title: str) -> None:
    axes.set_title(title)
    axes.set_xlabel("east position")
    axes.set_ylabel("north position")
    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(True, color="#d9d9d9")


def add_boundary_pose(
    axes: object,
    trajectory: dict[str, list[float]],
    leg: str,
    index: int,
    label: str,
) -> None:
    x = trajectory[f"{leg}.nominalX"][index]
    y = trajectory[f"{leg}.nominalY"][index]
    heading = trajectory[f"{leg}.nominalHeading"][index]
    axes.scatter((x,), (y,), color="#202020", s=30, zorder=5)
    axes.arrow(
        x,
        y,
        1.5 * math.cos(heading),
        1.5 * math.sin(heading),
        width=0.015,
        head_width=0.18,
        head_length=0.25,
        color="#202020",
        length_includes_head=True,
        zorder=4,
    )
    axes.annotate(label, (x, y), xytext=(6, 7), textcoords="offset points")


if __name__ == "__main__":
    raise SystemExit(main())
