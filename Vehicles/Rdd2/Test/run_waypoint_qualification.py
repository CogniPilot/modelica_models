#!/usr/bin/env python3
"""Qualify RDD2 GPS waypoint navigation in local and global mission modes.

The local mission follows a box authored in the local East-North-Up frame. The
global mission authors the same box in latitude/longitude/altitude and projects
it back through the mission origin. Both run the reusable log-linear controller,
waypoint guidance, body-rate loop, and control allocation, so a passing run
proves the geometric stack flies the box and that the geodetic projection is a
faithful round trip (the two ground tracks must coincide).
"""

from __future__ import annotations

import base64
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rumoca as rum


def repository_root() -> Path:
    configured = os.environ.get("MODELICA_MODELS_ROOT")
    candidates = ([Path(configured).expanduser()] if configured else []) + [
        Path.cwd(),
        *Path.cwd().parents,
        Path(__file__).resolve().parent,
        *Path(__file__).resolve().parents,
    ]
    for candidate in candidates:
        if (candidate / "Vehicles/Rdd2/Plant.mo").is_file():
            return candidate.resolve()
    raise RuntimeError("could not find the modelica_models repository")


ROOT = repository_root()
MODEL_DIR = ROOT / "Vehicles" / "Rdd2" / "Test"
ARTIFACT_DIR = ROOT / "artifacts" / "vehicles" / "rdd2"
MISSION_DURATION_S = 45.0
CRUISE_ALTITUDE_M = 2.0
BOX_SIDE_M = 4.0
BOX_CORNERS = [
    ("east", (BOX_SIDE_M, 0.0)),
    ("northeast", (BOX_SIDE_M, BOX_SIDE_M)),
    ("north", (0.0, BOX_SIDE_M)),
    ("home", (0.0, 0.0)),
]
SCENARIOS = {
    "local": MODEL_DIR / "rumoca-scenario.waypoint-local.toml",
    "global": MODEL_DIR / "rumoca-scenario.waypoint-global.toml",
}
TRACE_NAMES = [
    "time_s",
    *[f"position_m[{index}]" for index in range(1, 4)],
    "velocity_m_s[3]",
    *[f"euler_rad[{index}]" for index in range(1, 4)],
    "thrust_N",
    *[f"motorCommand[{index}]" for index in range(1, 5)],
    *[f"flightControl.reference.position[{index}]" for index in range(1, 4)],
    "flightControl.reference.trajectoryTime",
    *[f"geodetic[{index}]" for index in range(1, 4)],
    "missionPhase",
]


def columns(result: "rum.Result") -> dict[str, list[float]]:
    """Copy only qualification signals, then release Rumoca's full trace."""
    values = {"time": [float(value) for value in result.time]}
    for requested in TRACE_NAMES:
        matches = [
            name
            for name in result.names
            if name == requested or name.endswith(f".{requested}")
        ]
        if len(matches) != 1:
            raise KeyError(
                f"Rumoca result has no unambiguous signal {requested!r}: {matches}"
            )
        values[requested] = [float(value) for value in result[matches[0]]]
    return values


def signal(values: dict[str, list[float]], name: str) -> list[float]:
    if name in values:
        return values[name]
    suffix_matches = [column for column in values if column.endswith(f".{name}")]
    if len(suffix_matches) == 1:
        return values[suffix_matches[0]]
    raise KeyError(f"Rumoca result has no unambiguous signal {name!r}")


def source_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def simulate(scenario: Path) -> dict[str, list[float]]:
    _session, model, simulation = rum.Session.from_scenario(str(scenario))
    result = model.simulate(t=(0.0, MISSION_DURATION_S), config=simulation)
    return columns(result)


def corner_error(
    time: list[float], x: list[float], y: list[float], target: tuple[float, float]
) -> float:
    return min(
        math.hypot(x[index] - target[0], y[index] - target[1])
        for index in range(len(time))
    )


def evaluate(values: dict[str, list[float]]) -> dict[str, object]:
    time = signal(values, "time_s")
    x = signal(values, "position_m[1]")
    y = signal(values, "position_m[2]")
    altitude = signal(values, "position_m[3]")
    vertical_speed = signal(values, "velocity_m_s[3]")
    roll_deg = [math.degrees(value) for value in signal(values, "euler_rad[1]")]
    pitch_deg = [math.degrees(value) for value in signal(values, "euler_rad[2]")]
    motors = [signal(values, f"motorCommand[{index}]") for index in range(1, 5)]
    thrust = signal(values, "thrust_N")

    corner_errors = {
        name: corner_error(time, x, y, target) for name, target in BOX_CORNERS
    }
    max_tilt = max(max(map(abs, roll_deg)), max(map(abs, pitch_deg)))
    traced = [time, x, y, altitude, vertical_speed, roll_deg, pitch_deg, thrust, *motors]
    finite_trace = all(
        math.isfinite(sample) for samples in traced for sample in samples
    )
    minimum_motor = min(min(samples) for samples in motors)
    maximum_motor = max(max(samples) for samples in motors)

    metrics = {
        "simulated_seconds": time[-1],
        "max_altitude_m": max(altitude),
        "max_tilt_deg": max_tilt,
        "hover_thrust_n": max(thrust),
        **{f"corner_{name}_error_m": error for name, error in corner_errors.items()},
        "final_altitude_m": altitude[-1],
        "final_vertical_speed_m_s": vertical_speed[-1],
        "final_horizontal_error_m": math.hypot(x[-1], y[-1]),
        "minimum_motor_command": minimum_motor,
        "maximum_motor_command": maximum_motor,
    }
    checks = {
        "finite_trace": finite_trace,
        "bounded_motors": minimum_motor >= -1.0e-9 and maximum_motor <= 1.0 + 1.0e-9,
        "takeoff": metrics["max_altitude_m"] >= CRUISE_ALTITUDE_M - 0.5,
        **{
            f"box_corner_{name}": error <= 1.0
            for name, error in corner_errors.items()
        },
        "bounded_tilt": max_tilt <= 45.0,
        "landing_altitude": metrics["final_altitude_m"] <= 0.30,
        "landing_speed": abs(metrics["final_vertical_speed_m_s"]) <= 0.50,
        "landing_position": metrics["final_horizontal_error_m"] <= 1.0,
    }
    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics}


def mode_agreement(
    local: dict[str, list[float]], glob: dict[str, list[float]]
) -> dict[str, object]:
    """Local and global modes must fly the same local trajectory."""
    max_difference_m = 0.0
    for name in ("position_m[1]", "position_m[2]", "position_m[3]"):
        local_samples = signal(local, name)
        global_samples = signal(glob, name)
        count = min(len(local_samples), len(global_samples))
        for index in range(count):
            max_difference_m = max(
                max_difference_m,
                abs(local_samples[index] - global_samples[index]),
            )
    passed = max_difference_m <= 1.0e-3
    return {
        "passed": passed,
        "checks": {"local_global_tracks_match": passed},
        "metrics": {"max_track_difference_m": max_difference_m},
    }


def write_trace(values: dict[str, list[float]], path: Path) -> None:
    selected = {name: signal(values, name) for name in TRACE_NAMES}
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_NAMES)
        writer.writeheader()
        for index in range(len(selected["time_s"])):
            writer.writerow({name: selected[name][index] for name in TRACE_NAMES})


def plot_missions(
    local: dict[str, list[float]], glob: dict[str, list[float]]
) -> Path:
    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    figure.suptitle("RDD2 Waypoint Navigation Qualification", fontsize=16)

    corner_x = [0.0, BOX_SIDE_M, BOX_SIDE_M, 0.0, 0.0]
    corner_y = [0.0, 0.0, BOX_SIDE_M, BOX_SIDE_M, 0.0]
    axes[0, 0].plot(corner_x, corner_y, "k--", label="commanded box")
    axes[0, 0].plot(
        signal(local, "position_m[1]"),
        signal(local, "position_m[2]"),
        label="local mode",
    )
    axes[0, 0].plot(
        signal(glob, "position_m[1]"),
        signal(glob, "position_m[2]"),
        linestyle=":",
        label="global mode",
    )
    axes[0, 0].set_title("Ground Track")
    axes[0, 0].set_xlabel("east [m]")
    axes[0, 0].set_ylabel("north [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True)
    axes[0, 0].legend(loc="best")

    axes[0, 1].plot(
        signal(local, "time_s"),
        signal(local, "position_m[3]"),
        label="altitude",
    )
    axes[0, 1].plot(
        signal(local, "time_s"),
        signal(local, "flightControl.reference.position[3]"),
        linestyle="--",
        label="target",
    )
    axes[0, 1].set_title("Altitude")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("up [m]")
    axes[0, 1].grid(True)
    axes[0, 1].legend(loc="best")

    axes[1, 0].plot(
        signal(local, "geodetic[2]"),
        signal(local, "geodetic[1]"),
        label="global track",
    )
    axes[1, 0].set_title("Geodetic Track (local pose lifted through origin)")
    axes[1, 0].set_xlabel("longitude [deg]")
    axes[1, 0].set_ylabel("latitude [deg]")
    axes[1, 0].grid(True)
    axes[1, 0].ticklabel_format(useOffset=False)

    for motor in range(4):
        axes[1, 1].plot(
            signal(local, "time_s"),
            signal(local, f"motorCommand[{motor + 1}]"),
            label=f"motor {motor}",
        )
    axes[1, 1].set_title("Motor Commands (local mode)")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("normalized command")
    axes[1, 1].grid(True)
    axes[1, 1].legend(loc="best", ncols=2)

    path = ARTIFACT_DIR / "rdd2-waypoint-summary.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(f"wrote {path}")
    return path


def write_reports(report: dict[str, object], plot_path: Path) -> None:
    report_path = ARTIFACT_DIR / "waypoint-report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# RDD2 waypoint navigation qualification",
        "",
        f"Result: **{'PASS' if report['passed'] else 'FAIL'}**",
        "",
    ]
    for mode in ("local", "global", "agreement"):
        section = report[mode]
        assert isinstance(section, dict)
        checks = section["checks"]
        metrics = section["metrics"]
        assert isinstance(checks, dict) and isinstance(metrics, dict)
        lines.extend([f"## {mode}", "", "| Check | Result |", "| --- | --- |"])
        lines.extend(
            f"| `{name}` | {'PASS' if passed else 'FAIL'} |"
            for name, passed in checks.items()
        )
        lines.extend(["", "| Metric | Value |", "| --- | ---: |"])
        lines.extend(f"| `{name}` | {value:.6g} |" for name, value in metrics.items())
        lines.append("")
    lines.extend(
        [
            "## Mission plot",
            "",
            f"- `{plot_path.name}`",
            "",
            "Local and global modes fly the same box; the geodetic track is the "
            "local pose lifted through the mission origin.",
        ]
    )
    markdown = "\n".join(lines) + "\n"
    (ARTIFACT_DIR / "waypoint-summary.md").write_text(markdown, encoding="utf-8")

    image_data = base64.b64encode(plot_path.read_bytes()).decode("ascii")
    (ARTIFACT_DIR / "waypoint-report.html").write_text(
        "<!doctype html><meta charset='utf-8'>"
        "<title>RDD2 Waypoint Navigation Qualification</title>"
        "<style>body{font-family:system-ui,sans-serif;margin:2rem;color:#111827}"
        "pre{white-space:pre-wrap}img{max-width:100%;height:auto;"
        "border:1px solid #d1d5db}</style>"
        f"<body><pre>{html.escape(markdown)}</pre>"
        f"<img src='data:image/png;base64,{image_data}' "
        "alt='RDD2 waypoint qualification plots'>"
        "</body>\n",
        encoding="utf-8",
    )


def main() -> None:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    traces = {mode: simulate(scenario) for mode, scenario in SCENARIOS.items()}
    for mode, values in traces.items():
        write_trace(values, ARTIFACT_DIR / f"waypoint-{mode}.csv")

    local_report = evaluate(traces["local"])
    global_report = evaluate(traces["global"])
    agreement = mode_agreement(traces["local"], traces["global"])
    plot_path = plot_missions(traces["local"], traces["global"])

    report = {
        "passed": local_report["passed"]
        and global_report["passed"]
        and agreement["passed"],
        "local": local_report,
        "global": global_report,
        "agreement": agreement,
        "provenance": {
            "mission": {
                "path": "Vehicles/Rdd2/Test/WaypointMission.mo",
                "sha256": source_digest(MODEL_DIR / "WaypointMission.mo"),
            },
            "guidance": {
                "path": "Control/Multirotor/Navigation/package.mo",
                "sha256": source_digest(
                    ROOT / "Control" / "Multirotor" / "Navigation" / "package.mo"
                ),
            },
            "geodesy": {
                "path": "Geodesy/package.mo",
                "sha256": source_digest(ROOT / "Geodesy" / "package.mo"),
            },
            "rumoca": getattr(rum, "__version__", "workspace"),
        },
    }
    write_reports(report, plot_path)
    print(f"wrote {ARTIFACT_DIR / 'waypoint-summary.md'}")
    print(f"wrote {ARTIFACT_DIR / 'waypoint-report.json'}")
    if not report["passed"]:
        failures = []
        for mode in ("local", "global", "agreement"):
            section = report[mode]
            assert isinstance(section, dict)
            failures.extend(
                f"{mode}.{name}"
                for name, passed in section["checks"].items()
                if not passed
            )
        raise SystemExit(
            "RDD2 waypoint qualification failed: " + ", ".join(failures)
        )


if __name__ == "__main__":
    main()
