#!/usr/bin/env python3
"""Qualify the RDD2 controller and plant together in Modelica."""

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
MODEL_DIR = ROOT / "Vehicles" / "Rdd2" / "Qualification"
ARTIFACT_DIR = ROOT / "artifacts" / "vehicles" / "rdd2"
SCENARIO = MODEL_DIR / "rumoca-scenario.mission.toml"
MISSION_DURATION_S = 45.0
BOX_WAYPOINTS = [
    (0.0, 0.0),
    (4.0, 0.0),
    (4.0, 4.0),
    (0.0, 4.0),
    (0.0, 0.0),
]
BOX_WINDOWS = [
    ("east", 6.0, 13.0, (4.0, 0.0)),
    ("northeast", 13.0, 20.0, (4.0, 4.0)),
    ("north", 20.0, 27.0, (0.0, 4.0)),
    ("home", 27.0, 38.0, (0.0, 0.0)),
]


def columns(result: rum.Result) -> dict[str, list[float]]:
    values = {"time": [float(value) for value in result.time]}
    for name in result.names:
        values[name] = [float(value) for value in result[name]]
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


def evaluate(values: dict[str, list[float]]) -> dict[str, object]:
    time = signal(values, "time_s")
    x = signal(values, "x_m")
    y = signal(values, "y_m")
    altitude = signal(values, "z_m")
    vertical_speed = signal(values, "vz_m_s")
    roll_deg = [math.degrees(value) for value in signal(values, "roll_rad")]
    pitch_deg = [math.degrees(value) for value in signal(values, "pitch_rad")]
    estimated_roll_deg = [
        math.degrees(value) for value in signal(values, "estimated_roll_rad")
    ]
    estimated_pitch_deg = [
        math.degrees(value) for value in signal(values, "estimated_pitch_rad")
    ]
    motors = [signal(values, f"motor{index}") for index in range(4)]
    qualification_signals = [
        time,
        x,
        y,
        altitude,
        vertical_speed,
        roll_deg,
        pitch_deg,
        estimated_roll_deg,
        estimated_pitch_deg,
        *motors,
    ]

    def waypoint_error(start: float, end: float, target: tuple[float, float]) -> float:
        indices = [index for index, value in enumerate(time) if start <= value <= end]
        return min(
            math.hypot(x[index] - target[0], y[index] - target[1])
            for index in indices
        )

    waypoint_errors = {
        name: waypoint_error(start, end, target)
        for name, start, end, target in BOX_WINDOWS
    }
    max_tilt = max(max(map(abs, roll_deg)), max(map(abs, pitch_deg)))
    max_estimation_error = max(
        max(abs(actual - estimated) for actual, estimated in zip(roll_deg, estimated_roll_deg)),
        max(
            abs(actual - estimated)
            for actual, estimated in zip(pitch_deg, estimated_pitch_deg)
        ),
    )
    metrics = {
        "simulated_seconds": time[-1],
        "max_altitude_m": max(altitude),
        "max_tilt_deg": max_tilt,
        "max_roll_response_deg": max(map(abs, roll_deg)),
        "max_pitch_response_deg": max(map(abs, pitch_deg)),
        "max_attitude_estimation_error_deg": max_estimation_error,
        **{
            f"waypoint_{name}_error_m": error
            for name, error in waypoint_errors.items()
        },
        "final_altitude_m": altitude[-1],
        "final_vertical_speed_m_s": vertical_speed[-1],
        "final_horizontal_error_m": math.hypot(x[-1], y[-1]),
        "minimum_motor_command": min(min(samples) for samples in motors),
        "maximum_motor_command": max(max(samples) for samples in motors),
    }
    finite_trace = all(
        math.isfinite(sample)
        for samples in qualification_signals
        for sample in samples
    )
    bounded_motors = (
        metrics["minimum_motor_command"] >= -1.0e-9
        and metrics["maximum_motor_command"] <= 1.0 + 1.0e-9
    )
    checks = {
        "finite_trace": finite_trace,
        "bounded_motors": bounded_motors,
        "takeoff": metrics["max_altitude_m"] >= 1.0,
        **{
            f"box_waypoint_{name}": error <= 1.0
            for name, error in waypoint_errors.items()
        },
        "bounded_tilt": max_tilt <= 45.0,
        "attitude_estimator_tracking": max_estimation_error <= 5.0,
        "landing_altitude": metrics["final_altitude_m"] <= 0.20,
        "landing_speed": abs(metrics["final_vertical_speed_m_s"]) <= 0.50,
        "landing_position": metrics["final_horizontal_error_m"] <= 1.0,
    }
    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics}


def write_trace(values: dict[str, list[float]], path: Path) -> None:
    names = [
        "time_s",
        "x_m",
        "y_m",
        "z_m",
        "vz_m_s",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "estimated_roll_rad",
        "estimated_pitch_rad",
        "estimated_yaw_rad",
        "motor0",
        "motor1",
        "motor2",
        "motor3",
        "target_x_m",
        "target_y_m",
        "target_vx_m_s",
        "target_vy_m_s",
        "target_ax_m_s2",
        "target_ay_m_s2",
        "mission_phase",
    ]
    selected = {name: signal(values, name) for name in names}
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        for index in range(len(selected["time_s"])):
            writer.writerow({name: selected[name][index] for name in names})


def plot_mission(values: dict[str, list[float]]) -> Path:
    time = signal(values, "time_s")
    x = signal(values, "x_m")
    y = signal(values, "y_m")
    altitude = signal(values, "z_m")
    vertical_speed = signal(values, "vz_m_s")
    attitudes = {
        "roll": [math.degrees(value) for value in signal(values, "roll_rad")],
        "pitch": [math.degrees(value) for value in signal(values, "pitch_rad")],
        "yaw": [math.degrees(value) for value in signal(values, "yaw_rad")],
    }
    estimated_attitudes = {
        "roll estimate": [
            math.degrees(value) for value in signal(values, "estimated_roll_rad")
        ],
        "pitch estimate": [
            math.degrees(value) for value in signal(values, "estimated_pitch_rad")
        ],
        "yaw estimate": [
            math.degrees(value) for value in signal(values, "estimated_yaw_rad")
        ],
    }

    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    figure.suptitle("RDD2 Modelica Mission Qualification", fontsize=16)

    box_x = [waypoint[0] for waypoint in BOX_WAYPOINTS]
    box_y = [waypoint[1] for waypoint in BOX_WAYPOINTS]
    axes[0, 0].plot(box_x, box_y, "k--", label="commanded box")
    axes[0, 0].plot(x, y, label="actual")
    axes[0, 0].scatter(box_x[:-1], box_y[:-1], color="black", zorder=3)
    axes[0, 0].scatter([x[0], x[-1]], [y[0], y[-1]], c=["green", "red"], zorder=3)
    axes[0, 0].set_title("Ground Track")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True)
    axes[0, 0].legend(loc="best")

    altitude_axis = axes[0, 1]
    vertical_speed_axis = altitude_axis.twinx()
    altitude_axis.plot(time, altitude, color="tab:blue", label="altitude")
    vertical_speed_axis.plot(
        time,
        vertical_speed,
        color="tab:orange",
        alpha=0.8,
        label="vertical speed",
    )
    altitude_axis.set_title("Takeoff and Landing")
    altitude_axis.set_xlabel("time [s]")
    altitude_axis.set_ylabel("altitude [m]", color="tab:blue")
    vertical_speed_axis.set_ylabel("vertical speed [m/s]", color="tab:orange")
    altitude_axis.grid(True)

    for name, samples in attitudes.items():
        axes[1, 0].plot(time, samples, label=name)
    for name, samples in estimated_attitudes.items():
        axes[1, 0].plot(time, samples, linestyle="--", alpha=0.75, label=name)
    axes[1, 0].set_title("Attitude Response")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("angle [deg]")
    axes[1, 0].grid(True)
    axes[1, 0].legend(loc="best")

    for motor in range(4):
        axes[1, 1].plot(time, signal(values, f"motor{motor}"), label=f"motor {motor}")
    axes[1, 1].set_title("Motor Commands")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("normalized command")
    axes[1, 1].grid(True)
    axes[1, 1].legend(loc="best", ncols=2)

    path = ARTIFACT_DIR / "rdd2-qualification-summary.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(f"wrote {path}")
    return path


def write_reports(report: dict[str, object], plot_path: Path) -> None:
    report_path = ARTIFACT_DIR / "modelica-report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    checks = report["checks"]
    metrics = report["metrics"]
    assert isinstance(checks, dict) and isinstance(metrics, dict)
    lines = [
        "# RDD2 pure Modelica qualification",
        "",
        f"Result: **{'PASS' if report['passed'] else 'FAIL'}**",
        "",
        "| Check | Result |",
        "| --- | --- |",
    ]
    lines.extend(
        f"| `{name}` | {'PASS' if passed else 'FAIL'} |" for name, passed in checks.items()
    )
    lines.extend(["", "| Metric | Value |", "| --- | ---: |"])
    lines.extend(f"| `{name}` | {value:.6g} |" for name, value in metrics.items())
    lines.extend(
        [
            "",
            "## Mission plot",
            "",
            f"- `{plot_path.name}`",
            "",
            "The controller and six-degree-of-freedom plant both execute as Modelica in Rumoca.",
        ]
    )
    markdown = "\n".join(lines) + "\n"
    (ARTIFACT_DIR / "modelica-summary.md").write_text(markdown, encoding="utf-8")
    image_data = base64.b64encode(plot_path.read_bytes()).decode("ascii")
    (ARTIFACT_DIR / "modelica-report.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>RDD2 Modelica Qualification</title>"
        "<style>body{font-family:system-ui,sans-serif;margin:2rem;color:#111827}"
        "pre{white-space:pre-wrap}img{max-width:100%;height:auto;border:1px solid #d1d5db}</style>"
        f"<body><pre>{html.escape(markdown)}</pre>"
        f"<img src='data:image/png;base64,{image_data}' alt='RDD2 mission qualification plots'>"
        "</body>\n",
        encoding="utf-8",
    )


def main() -> None:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    for path in ARTIFACT_DIR.iterdir():
        if path.is_file():
            path.unlink()
    _session, model, simulation = rum.Session.from_scenario(str(SCENARIO))
    result = model.simulate(t=(0.0, MISSION_DURATION_S), config=simulation)
    values = columns(result)
    write_trace(values, ARTIFACT_DIR / "mission.csv")
    write_trace(values, ARTIFACT_DIR / "mission-trajectory.csv")
    plot_path = plot_mission(values)
    report = evaluate(values)
    report["mode"] = "modelica"
    report["control"] = "Modelica"
    report["physics"] = "Modelica"
    report["provenance"] = {
        "controller": {
            "path": "Vehicles/Rdd2/Controller.mo",
            "sha256": source_digest(ROOT / "Vehicles" / "Rdd2" / "Controller.mo"),
        },
        "estimator": {
            "path": "Estimation/ComplementaryAttitude.mo",
            "sha256": source_digest(ROOT / "Estimation" / "ComplementaryAttitude.mo"),
        },
        "plant": {
            "configuration": {
                "path": "Vehicles/Rdd2/Plant.mo",
                "sha256": source_digest(ROOT / "Vehicles" / "Rdd2" / "Plant.mo"),
            },
            "template": {
                "path": "Vehicles/Templates/QuadrotorPlant.mo",
                "sha256": source_digest(
                    ROOT / "Vehicles" / "Templates" / "QuadrotorPlant.mo"
                ),
            },
        },
        "mission": {
            "path": "Vehicles/Rdd2/Qualification/Mission.mo",
            "sha256": source_digest(MODEL_DIR / "Mission.mo"),
        },
        "rumoca": getattr(rum, "__version__", "workspace"),
    }
    write_reports(report, plot_path)
    print(f"wrote {ARTIFACT_DIR / 'modelica-summary.md'}")
    print(f"wrote {ARTIFACT_DIR / 'modelica-report.json'}")
    if not report["passed"]:
        failed = [name for name, passed in report["checks"].items() if not passed]
        raise SystemExit(f"RDD2 pure Modelica qualification failed: {', '.join(failed)}")


if __name__ == "__main__":
    main()
