#!/usr/bin/env python3
"""Qualify the RDD2 controller and plant together in Modelica."""

from __future__ import annotations

import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path

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
    altitude = signal(values, "z_m")
    vertical_speed = signal(values, "vz_m_s")
    roll_deg = [math.degrees(value) for value in signal(values, "roll_rad")]
    pitch_deg = [math.degrees(value) for value in signal(values, "pitch_rad")]

    in_window = lambda start, end: [index for index, value in enumerate(time) if start <= value <= end]
    roll_response = max(abs(roll_deg[index]) for index in in_window(8.0, 10.5))
    pitch_response = max(abs(pitch_deg[index]) for index in in_window(11.0, 13.5))
    max_tilt = max(max(map(abs, roll_deg)), max(map(abs, pitch_deg)))
    metrics = {
        "simulated_seconds": time[-1],
        "max_altitude_m": max(altitude),
        "max_tilt_deg": max_tilt,
        "max_roll_response_deg": roll_response,
        "max_pitch_response_deg": pitch_response,
        "final_altitude_m": altitude[-1],
        "final_vertical_speed_m_s": vertical_speed[-1],
    }
    checks = {
        "takeoff": metrics["max_altitude_m"] >= 1.0,
        "roll_response": roll_response >= 2.0,
        "pitch_response": pitch_response >= 2.0,
        "bounded_tilt": max_tilt <= 45.0,
        "landing_altitude": metrics["final_altitude_m"] <= 0.20,
        "landing_speed": abs(metrics["final_vertical_speed_m_s"]) <= 0.50,
    }
    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics}


def write_trace(values: dict[str, list[float]]) -> None:
    names = [
        "time_s",
        "x_m",
        "y_m",
        "z_m",
        "vz_m_s",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "motor0",
        "motor1",
        "motor2",
        "motor3",
    ]
    selected = {name: signal(values, name) for name in names}
    with (ARTIFACT_DIR / "mission.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        for index in range(len(selected["time_s"])):
            writer.writerow({name: selected[name][index] for name in names})


def write_reports(report: dict[str, object]) -> None:
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
            "The controller and six-degree-of-freedom plant both execute as Modelica in Rumoca.",
        ]
    )
    markdown = "\n".join(lines) + "\n"
    (ARTIFACT_DIR / "modelica-summary.md").write_text(markdown, encoding="utf-8")
    (ARTIFACT_DIR / "modelica-report.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>RDD2 Modelica</title>"
        f"<body><pre>{html.escape(markdown)}</pre></body>\n",
        encoding="utf-8",
    )


def main() -> None:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    _session, model, simulation = rum.Session.from_scenario(str(SCENARIO))
    result = model.simulate(t=(0.0, 20.0), config=simulation)
    values = columns(result)
    write_trace(values)
    report = evaluate(values)
    report["mode"] = "modelica"
    report["control"] = "Modelica"
    report["physics"] = "Modelica"
    report["provenance"] = {
        "controller": {
            "path": "Vehicles/Rdd2/Controller.mo",
            "sha256": source_digest(ROOT / "Vehicles" / "Rdd2" / "Controller.mo"),
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
    write_reports(report)
    print(f"wrote {ARTIFACT_DIR / 'modelica-summary.md'}")
    print(f"wrote {ARTIFACT_DIR / 'modelica-report.json'}")
    if not report["passed"]:
        failed = [name for name, passed in report["checks"].items() if not passed]
        raise SystemExit(f"RDD2 pure Modelica qualification failed: {', '.join(failed)}")


if __name__ == "__main__":
    main()
