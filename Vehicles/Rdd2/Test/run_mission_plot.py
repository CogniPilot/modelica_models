#!/usr/bin/env python3
"""Run or replay and plot an RDD2 optical-flow or GPS waypoint mission.

The named mode selects an explicit Modelica class.  Live mode requests the
pinned OpenModelica and frozen Rumoca solvers and fails closed when either exact
compiler is unavailable.  Existing solver CSV files can only be replayed with a
matching provenance receipt.  The script retains a compact common trace and
writes PNG, self-contained HTML, and JSON reports.  This is a trace-reporting
tool, not a flight-acceptance or mission-qualification tool.
"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


DEFAULT_STOP_TIME_S = 45.0
DEFAULT_STEP_S = 0.005
GPS_MODEL_NAVIGATION_ERROR_DIAGNOSTIC_M = 0.5
RECEIPT_SCHEMA = "rdd2-mission-trace-receipt-v1"
EXPECTED_MODEL_REVISION = "a9e5037ab3e57b3fac6ca783c0bdbfdd2b6dd98e"
EXPECTED_RUMOCA_VERSION = "0.9.20"
MODEL_INPUT_PATHSPECS = (
    ":(glob)**/*.mo",
    ":(glob)**/*.toml",
    ":(glob)**/Resources/**",
    ":(glob)**/package.order",
)
EXPECTED_COMPILERS = {
    "omc": {
        "name": "OpenModelica",
        "identity": "OpenModelica/a96aa1a682c463b0fd2d285b486c09a8b7fe496d",
        "revision": "a96aa1a682c463b0fd2d285b486c09a8b7fe496d",
    },
    "rumoca": {
        "name": "Rumoca",
        "identity": "rdd2-flight-freeze-2-9860c307",
        "revision": "9860c30781242ff65dfcf47b136385ac5ecf4350",
    },
}

MODELS = {
    "optical": {
        "model": "Vehicles.Rdd2.Test.WaypointMission",
        "scenario": "rumoca-scenario.waypoint-local.toml",
        "aiding": "optical flow",
        "aiding_signals": ["estimator.status.opticalFlowCorrectionAccepted"],
    },
    "gps": {
        "model": "Vehicles.Rdd2.Test.GlobalWaypointMission",
        "scenario": "rumoca-scenario.waypoint-global.toml",
        "aiding": "GPS position and velocity",
        "aiding_signals": [
            "estimator.status.gpsPositionCorrectionAccepted",
            "estimator.status.gpsVelocityCorrectionAccepted",
        ],
    },
}

SIGNALS = [
    *[f"position_m[{axis}]" for axis in range(1, 4)],
    *[f"avionics.reference.position[{axis}]" for axis in range(1, 4)],
    "navigationError_m",
    "missionPhase",
    "estimator.estimate.valid",
    "estimator.status.initialized",
    "estimator.status.predictionAccepted",
    "estimator.status.mocapCorrectionAccepted",
    "estimator.status.opticalFlowCorrectionAccepted",
    "estimator.status.gpsPositionCorrectionAccepted",
    "estimator.status.gpsVelocityCorrectionAccepted",
    "estimator.status.consecutiveRejectedCorrections",
    "estimator.status.covarianceReinitialized",
    "estimator.status.innovationGateRejected",
]

COLORS = {"omc": "#2563eb", "rumoca": "#dc2626"}
LABELS = {"omc": "OpenModelica", "rumoca": "Rumoca"}


def repository_root() -> Path:
    configured = os.environ.get("MODELICA_MODELS_ROOT")
    candidates = ([Path(configured).expanduser()] if configured else []) + [
        Path.cwd(),
        *Path.cwd().parents,
        Path(__file__).resolve().parent,
        *Path(__file__).resolve().parents,
    ]
    for candidate in candidates:
        if (candidate / "Vehicles/Rdd2/Test/GlobalWaypointMission.mo").is_file():
            return candidate.resolve()
    raise RuntimeError("could not find the modelica_models repository")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def declared_executable(environment_name: str) -> Path:
    configured = os.environ.get(environment_name)
    if not configured:
        raise RuntimeError(f"{environment_name} is unset")
    declared = Path(configured)
    if not declared.is_absolute():
        raise RuntimeError(f"{environment_name} must be an absolute path")
    try:
        resolved = declared.resolve(strict=True)
    except OSError as error:
        raise RuntimeError(
            f"{environment_name} does not resolve to an executable: {declared}"
        ) from error
    if not os.access(resolved, os.X_OK):
        raise RuntimeError(f"{environment_name} is not executable: {resolved}")
    return resolved


def require_declared_python() -> dict[str, str]:
    expected = declared_executable("RDD2_MISSION_PYTHON")
    actual = Path(sys.executable).resolve(strict=True)
    if actual != expected:
        raise RuntimeError(
            f"Python executable is {actual}, expected declared interpreter {expected}"
        )
    return {"path": str(actual), "sha256": sha256(actual)}


def git_run(root: Path, *arguments: str) -> subprocess.CompletedProcess[str]:
    executable = declared_executable("RDD2_MISSION_GIT")
    return subprocess.run(
        [str(executable), "-C", str(root), *arguments],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def model_revision(root: Path) -> str:
    completed = git_run(root, "rev-parse", "HEAD")
    revision = completed.stdout.strip()
    if completed.returncode != 0 or len(revision) != 40:
        raise RuntimeError(
            "cannot identify the modelica_models revision required for provenance"
        )
    if revision != EXPECTED_MODEL_REVISION:
        raise RuntimeError(
            "this reporter is receipted for modelica_models "
            f"{EXPECTED_MODEL_REVISION}, but the checkout is {revision}"
        )
    return revision


def require_clean_model_inputs(root: Path, revision: str) -> dict[str, object]:
    completed = git_run(
        root,
        "status",
        "--porcelain=v1",
        "--untracked-files=all",
        "--",
        *MODEL_INPUT_PATHSPECS,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "cannot inspect live model inputs: " + completed.stderr.strip()
        )
    changes = [line for line in completed.stdout.splitlines() if line]
    if changes:
        rendered = "\n  ".join(changes[:20])
        suffix = "\n  ..." if len(changes) > 20 else ""
        raise RuntimeError(
            "live simulation refuses modified or untracked Modelica, scenario, "
            f"resource, or package-order inputs:\n  {rendered}{suffix}"
        )
    tree = git_run(root, "rev-parse", "HEAD^{tree}")
    tree_hash = tree.stdout.strip()
    if tree.returncode != 0 or len(tree_hash) != 40:
        raise RuntimeError("cannot identify the clean model input tree")
    if model_revision(root) != revision:
        raise RuntimeError("model revision changed while validating live inputs")
    return {
        "git_revision": revision,
        "git_tree": tree_hash,
        "relevant_inputs_clean": True,
        "pathspecs": list(MODEL_INPUT_PATHSPECS),
    }


def receipt_mapping(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise RuntimeError(f"{path}: invalid receipt JSON: {error}") from error
    if not isinstance(payload, dict):
        raise RuntimeError(f"{path}: receipt must be a JSON object")
    return payload


def receipt_number(receipt: dict[str, object], name: str, path: Path) -> float:
    value = receipt.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{path}: receipt field {name!r} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise RuntimeError(f"{path}: receipt field {name!r} must be finite")
    return number


def validate_receipt(
    receipt_path: Path,
    csv_path: Path,
    engine: str,
    mode: str,
    revision: str,
    stop_time: float,
    step: float,
) -> dict[str, object]:
    receipt = receipt_mapping(receipt_path)
    expected_fields: dict[str, object] = {
        "schema": RECEIPT_SCHEMA,
        "engine": engine,
        "mode": mode,
        "model": MODELS[mode]["model"],
        "compiler": EXPECTED_COMPILERS[engine],
        "model_revision": revision,
    }
    for name, expected in expected_fields.items():
        if receipt.get(name) != expected:
            raise RuntimeError(
                f"{receipt_path}: receipt field {name!r} is {receipt.get(name)!r}; "
                f"expected {expected!r}"
            )
    for name, expected in (("stop_time_s", stop_time), ("step_s", step)):
        actual = receipt_number(receipt, name, receipt_path)
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12):
            raise RuntimeError(
                f"{receipt_path}: receipt field {name!r} is {actual}; "
                f"expected {expected}"
            )
    expected_digest = receipt.get("csv_sha256")
    if not isinstance(expected_digest, str) or len(expected_digest) != 64:
        raise RuntimeError(
            f"{receipt_path}: receipt field 'csv_sha256' must be a lowercase "
            "SHA-256 hex digest"
        )
    if expected_digest != expected_digest.lower():
        raise RuntimeError(
            f"{receipt_path}: receipt field 'csv_sha256' must be lowercase"
        )
    try:
        int(expected_digest, 16)
    except ValueError as error:
        raise RuntimeError(
            f"{receipt_path}: receipt field 'csv_sha256' must be hexadecimal"
        ) from error
    actual_digest = sha256(csv_path)
    if expected_digest != actual_digest:
        raise RuntimeError(
            f"{receipt_path}: CSV SHA-256 mismatch: receipt {expected_digest}, "
            f"actual {actual_digest}"
        )
    return {
        "source": "receipted_csv_replay",
        "csv_path": str(csv_path),
        "csv_sha256": actual_digest,
        "receipt_path": str(receipt_path),
        "receipt_sha256": sha256(receipt_path),
        "compiler": EXPECTED_COMPILERS[engine],
        "model_revision": revision,
    }


def find_column(header: Iterable[str], requested: str) -> str:
    matches = [
        name for name in header if name == requested or name.endswith(f".{requested}")
    ]
    if len(matches) != 1:
        raise KeyError(f"no unambiguous column {requested!r}: {matches}")
    return matches[0]


def read_selected_csv(path: Path) -> dict[str, list[float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        try:
            header = next(reader)
        except StopIteration as error:
            raise RuntimeError(f"{path}: CSV is empty") from error
        if not header or not any(name.strip() for name in header):
            raise RuntimeError(f"{path}: CSV header is empty")
        time_name = "time" if "time" in header else find_column(header, "time_s")
        source_names = {name: find_column(header, name) for name in SIGNALS}
        indices = {name: header.index(source) for name, source in source_names.items()}
        time_index = header.index(time_name)
        values = {"time": []}
        values.update({name: [] for name in SIGNALS})
        for row_number, row in enumerate(reader, start=2):
            try:
                values["time"].append(float(row[time_index]))
                for name, index in indices.items():
                    values[name].append(float(row[index]))
            except (IndexError, ValueError) as error:
                raise RuntimeError(
                    f"{path}:{row_number}: invalid numeric row"
                ) from error
    if not values["time"]:
        raise RuntimeError(f"{path}: trace is empty")
    return values


def validate_trace(
    values: dict[str, list[float]], path: Path, stop_time: float, step: float
) -> dict[str, object]:
    count = len(values["time"])
    lengths = {name: len(samples) for name, samples in values.items()}
    if any(length != count for length in lengths.values()):
        raise RuntimeError(f"{path}: trace columns have unequal lengths: {lengths}")
    if not finite_trace(values):
        raise RuntimeError(f"{path}: trace contains a non-finite value")

    times = values["time"]
    time_tolerance = max(1.0e-9, step * 1.0e-6)
    if abs(times[0]) > time_tolerance:
        raise RuntimeError(f"{path}: trace starts at {times[0]}, expected 0")
    for index, (left, right) in enumerate(zip(times, times[1:]), start=1):
        delta = right - left
        if delta <= 0.0:
            raise RuntimeError(
                f"{path}: time is not strictly increasing at sample {index}: "
                f"{left} then {right}"
            )
        if not math.isclose(delta, step, rel_tol=1.0e-6, abs_tol=1.0e-9):
            raise RuntimeError(
                f"{path}: cadence is {delta} s at sample {index}; expected {step} s"
            )
    if abs(times[-1] - stop_time) > time_tolerance:
        raise RuntimeError(f"{path}: trace ends at {times[-1]}, expected {stop_time} s")
    expected_samples = round(stop_time / step) + 1
    if count != expected_samples:
        raise RuntimeError(
            f"{path}: trace has {count} samples; expected {expected_samples}"
        )
    return {
        "start_time_s": times[0],
        "end_time_s": times[-1],
        "samples": count,
        "strictly_increasing_time": True,
        "cadence_s": step,
        "finite": True,
    }


def result_columns(result: object) -> dict[str, list[float]]:
    names = list(result.names)
    values = {"time": [float(value) for value in result.time]}
    for requested in SIGNALS:
        source = find_column(names, requested)
        values[requested] = [float(value) for value in result[source]]
    return values


def write_selected_csv(values: dict[str, list[float]], path: Path) -> None:
    names = ["time", *SIGNALS]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(len(values["time"])):
            writer.writerow([values[name][index] for name in names])


def run_openmodelica(
    root: Path,
    model: str,
    stop_time: float,
    step: float,
    tolerance: float,
    solver: str,
    executable: Path,
) -> tuple[dict[str, list[float]], dict[str, object]]:
    intervals = max(1, round(stop_time / step))
    with tempfile.TemporaryDirectory(prefix="rdd2-mission-omc-") as directory:
        working = Path(directory)
        script = working / "simulate.mos"
        script.write_text(
            "\n".join(
                [
                    'setCommandLineOptions("--std=3.6");',
                    f'loadFile("{root / "Vehicles/package.mo"}");',
                    "print(getErrorString());",
                    f'cd("{working}");',
                    "result := simulate(",
                    f"  {model},",
                    "  startTime = 0.0,",
                    f"  stopTime = {stop_time:.17g},",
                    f"  numberOfIntervals = {intervals},",
                    f"  tolerance = {tolerance:.17g},",
                    f'  method = "{solver}",',
                    '  outputFormat = "csv",',
                    '  fileNamePrefix = "mission");',
                    'print("\\nOMC_RESULT=" + String(result) + "\\n");',
                    "print(getErrorString());",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        completed = subprocess.run(
            [str(executable), str(script)],
            cwd=root,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        csv_path = working / "mission_res.csv"
        if completed.returncode != 0 or not csv_path.is_file():
            tail = "\n".join(completed.stdout.splitlines()[-30:])
            raise RuntimeError(
                f"OpenModelica did not produce {csv_path} "
                f"(exit {completed.returncode}):\n{tail}"
            )
        values = read_selected_csv(csv_path)
    return values, {
        "source": "simulation",
        "compiler": {
            **EXPECTED_COMPILERS["omc"],
            "executable": str(executable),
            "executable_sha256": sha256(executable),
        },
        "solver": solver,
        "tolerance": tolerance,
        "requested_intervals": intervals,
    }


def run_rumoca(
    scenario: Path,
    stop_time: float,
    step: float,
    tolerance: float,
    solver: str,
) -> tuple[dict[str, list[float]], dict[str, object]]:
    try:
        import rumoca as rum
    except ImportError as error:
        raise RuntimeError(
            "Rumoca Python bindings are unavailable"
        ) from error

    module_path_text = getattr(rum, "__file__", None)
    if not isinstance(module_path_text, str):
        raise RuntimeError("Rumoca module has no inspectable file origin")
    module_path = Path(module_path_text).resolve(strict=True)
    compiler_version = rum.version()
    if compiler_version != EXPECTED_RUMOCA_VERSION:
        raise RuntimeError(
            f"Rumoca Python version is {compiler_version}, expected "
            f"{EXPECTED_RUMOCA_VERSION}"
        )

    _session, model, _scenario_config = rum.Session.from_scenario(str(scenario))
    config = rum.SimConfig(
        solver=solver,
        dt=step,
        atol=tolerance,
        rtol=tolerance,
    )
    try:
        result = model.simulate(t=(0.0, stop_time), dt=step, config=config)
    except Exception as error:
        raise RuntimeError(
            f"Rumoca simulation failed for {model.name}: {error}"
        ) from error
    return result_columns(result), {
        "source": "simulation",
        "compiler": {
            **EXPECTED_COMPILERS["rumoca"],
            "version": compiler_version,
            "module_path": str(module_path),
            "module_sha256": sha256(module_path),
        },
        "solver": solver,
        "tolerance": tolerance,
        "termination": result.termination,
        "metrics": result.metrics,
    }


def finite_trace(values: dict[str, list[float]]) -> bool:
    return all(math.isfinite(value) for samples in values.values() for value in samples)


def evaluate_engine(values: dict[str, list[float]], mode: str) -> dict[str, object]:
    aiding_signals = MODELS[mode]["aiding_signals"]
    navigation_error = values["navigationError_m"]
    observations = {
        "estimate_initialized_at_end": values["estimator.status.initialized"][-1] > 0.5,
        "estimate_valid_at_end": values["estimator.estimate.valid"][-1] > 0.5,
        "prediction_accepted_samples": sum(
            value > 0.5 for value in values["estimator.status.predictionAccepted"]
        ),
        "covariance_reinitialized_samples": sum(
            value > 0.5 for value in values["estimator.status.covarianceReinitialized"]
        ),
        "innovation_gate_rejected_samples": sum(
            value > 0.5 for value in values["estimator.status.innovationGateRejected"]
        ),
        "maximum_consecutive_rejected_corrections": max(
            values["estimator.status.consecutiveRejectedCorrections"]
        ),
    }
    for signal in aiding_signals:
        observation_name = f"{signal.rsplit('.', maxsplit=1)[-1]}_samples"
        observations[observation_name] = sum(value > 0.5 for value in values[signal])
    metrics = {
        "samples": len(values["time"]),
        "final_time_s": values["time"][-1],
        "maximum_absolute_navigation_error_m": max(navigation_error),
        "terminal_absolute_navigation_error_m": navigation_error[-1],
    }
    diagnostics: dict[str, object] = {}
    if mode == "gps":
        diagnostics["established_model_navigation_error_metric"] = {
            "criterion_m": GPS_MODEL_NAVIGATION_ERROR_DIAGNOSTIC_M,
            "observed_maximum_m": max(navigation_error),
            "within_criterion": max(navigation_error)
            <= GPS_MODEL_NAVIGATION_ERROR_DIAGNOSTIC_M,
            "scope": "model diagnostic; not flight acceptance",
        }
    else:
        metrics["terminal_absolute_position_drift_m"] = navigation_error[-1]
        diagnostics["absolute_position_observability"] = (
            "optical-flow velocity aiding does not observe absolute position; "
            "no pass/fail threshold is applied"
        )
    return {
        "observations": observations,
        "metrics": metrics,
        "diagnostics": diagnostics,
    }


def interpolate(times: list[float], samples: list[float], target: float) -> float:
    if target <= times[0]:
        return samples[0]
    if target >= times[-1]:
        return samples[-1]
    low = 0
    high = len(times) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if times[middle] <= target:
            low = middle
        else:
            high = middle
    span = times[high] - times[low]
    fraction = 0.0 if span == 0.0 else (target - times[low]) / span
    return samples[low] + fraction * (samples[high] - samples[low])


def compare_engines(traces: dict[str, dict[str, list[float]]]) -> dict[str, object]:
    if set(traces) != {"omc", "rumoca"}:
        return {"available": False, "metrics": {}}
    omc = traces["omc"]
    rumoca = traces["rumoca"]
    end_time = min(omc["time"][-1], rumoca["time"][-1])
    differences: list[float] = []
    for index, time in enumerate(omc["time"]):
        if time > end_time:
            break
        squared = 0.0
        for axis in range(1, 4):
            signal = f"position_m[{axis}]"
            difference = omc[signal][index] - interpolate(
                rumoca["time"], rumoca[signal], time
            )
            squared += difference * difference
        differences.append(math.sqrt(squared))
    maximum = max(differences) if differences else math.inf
    return {
        "available": True,
        "metrics": {
            "common_duration_s": end_time,
            "max_enu_position_difference_m": maximum,
        },
        "scope": "cross-engine trace difference; no qualification threshold applied",
    }


def status_step(
    axis: object,
    values: dict[str, list[float]],
    name: str,
    **kwargs: object,
) -> None:
    axis.step(values["time"], values[name], where="post", **kwargs)


def plot_report(
    traces: dict[str, dict[str, list[float]]], mode: str, report: dict[str, object]
) -> plt.Figure:
    figure, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
    model = MODELS[mode]["model"]
    figure.suptitle(
        f"{model} — {MODELS[mode]['aiding']} mission\n"
        "Trace report — not flight acceptance",
        fontsize=15,
    )

    for engine, values in traces.items():
        color = COLORS[engine]
        label = LABELS[engine]
        axes[0, 0].plot(
            values["position_m[1]"],
            values["position_m[2]"],
            color=color,
            label=label,
        )
        axes[0, 1].plot(
            values["time"],
            values["position_m[3]"],
            color=color,
            label=label,
        )
        axes[0, 2].plot(
            values["time"],
            values["navigationError_m"],
            color=color,
            label=label,
        )
        status_step(
            axes[1, 0],
            values,
            "estimator.status.initialized",
            color=color,
            label=f"{label} initialized",
        )
        status_step(
            axes[1, 0],
            values,
            "estimator.estimate.valid",
            color=color,
            linestyle="--",
            label=f"{label} estimate valid",
        )
        status_step(
            axes[1, 0],
            values,
            "estimator.status.covarianceReinitialized",
            color=color,
            linestyle=":",
            label=f"{label} covariance reinitialized",
        )
        status_step(
            axes[1, 0],
            values,
            "estimator.status.innovationGateRejected",
            color=color,
            linestyle="-.",
            label=f"{label} innovation rejected",
        )
        for signal, linestyle in zip(
            MODELS[mode]["aiding_signals"], ["-", "--"], strict=False
        ):
            status_step(
                axes[1, 1],
                values,
                signal,
                color=color,
                linestyle=linestyle,
                label=f"{label} {signal.split('.')[-1]}",
            )

    reference = next(iter(traces.values()))
    axes[0, 0].plot(
        reference["avionics.reference.position[1]"],
        reference["avionics.reference.position[2]"],
        "k--",
        linewidth=1.2,
        label="reference",
    )
    axes[0, 1].plot(
        reference["time"],
        reference["avionics.reference.position[3]"],
        "k--",
        linewidth=1.2,
        label="reference",
    )
    if mode == "gps":
        axes[0, 2].axhline(
            GPS_MODEL_NAVIGATION_ERROR_DIAGNOSTIC_M,
            color="black",
            linestyle="--",
            linewidth=1.0,
            label="0.5 m model diagnostic",
        )

    comparison = report["comparison"]
    comparison_metrics = comparison["metrics"]
    if set(traces) == {"omc", "rumoca"}:
        omc = traces["omc"]
        rumoca = traces["rumoca"]
        common_times = [time for time in omc["time"] if time <= rumoca["time"][-1]]
        for axis in range(1, 4):
            signal = f"position_m[{axis}]"
            difference = [
                omc[signal][index] - interpolate(rumoca["time"], rumoca[signal], time)
                for index, time in enumerate(common_times)
            ]
            axes[1, 2].plot(common_times, difference, label=f"ENU"[axis - 1])
        axes[1, 2].axhline(0.0, color="black", linewidth=0.8)
        axes[1, 2].set_title(
            "OpenModelica − Rumoca ENU\n"
            f"max norm {comparison_metrics['max_enu_position_difference_m']:.3g} m"
        )
    else:
        engine = next(iter(traces))
        values = traces[engine]
        for axis, label in enumerate("ENU", start=1):
            signal = f"position_m[{axis}]"
            reference_signal = f"avionics.reference.position[{axis}]"
            axes[1, 2].plot(
                values["time"],
                [
                    actual - requested
                    for actual, requested in zip(
                        values[signal], values[reference_signal], strict=True
                    )
                ],
                label=label,
            )
        axes[1, 2].set_title(f"{LABELS[engine]} ENU tracking error")

    axes[0, 0].set_title("ENU ground track")
    axes[0, 0].set_xlabel("east [m]")
    axes[0, 0].set_ylabel("north [m]")
    axes[0, 0].axis("equal")
    axes[0, 1].set_title("Altitude")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("up [m]")
    axes[0, 2].set_title("Navigation error")
    axes[0, 2].set_xlabel("time [s]")
    axes[0, 2].set_ylabel("error norm [m]")
    axes[1, 0].set_title("Estimator and filter health")
    axes[1, 1].set_title("Aiding correction acceptance")
    for axis in axes[1, :]:
        axis.set_xlabel("time [s]")
    axes[1, 0].set_ylim(-0.05, 1.05)
    axes[1, 1].set_ylim(-0.05, 1.05)
    axes[1, 2].set_ylabel("position error [m]")
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best", fontsize="small")

    return figure


def write_outputs(
    traces: dict[str, dict[str, list[float]]],
    provenance: dict[str, dict[str, object]],
    mode: str,
    output_dir: Path,
    stop_time: float,
    step: float,
    revision: str,
    python_identity: dict[str, str],
) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    validation = {
        engine: validate_trace(values, Path(f"{engine} trace"), stop_time, step)
        for engine, values in traces.items()
    }
    for engine, values in traces.items():
        write_selected_csv(values, output_dir / f"{engine}-selected.csv")

    engines = {
        engine: {
            "trace_validation": validation[engine],
            **evaluate_engine(values, mode),
        }
        for engine, values in traces.items()
    }
    comparison = compare_engines(traces)
    report = {
        "schema": "rdd2-mission-trace-report-v1",
        "purpose": "mission trace reporting; not flight acceptance or qualification",
        "mode": mode,
        "model": MODELS[mode]["model"],
        "model_revision": revision,
        "reporter_python": python_identity,
        "aiding": MODELS[mode]["aiding"],
        "requested_stop_time_s": stop_time,
        "requested_step_s": step,
        "engines": engines,
        "comparison": comparison,
        "provenance": provenance,
    }
    report_path = output_dir / "report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    figure = plot_report(traces, mode, report)
    png_path = output_dir / f"rdd2-{mode}-mission.png"
    figure.savefig(png_path, dpi=160)
    plt.close(figure)

    image_data = base64.b64encode(png_path.read_bytes()).decode("ascii")
    report_text = json.dumps(report, indent=2)
    html_path = output_dir / f"rdd2-{mode}-mission.html"
    html_path.write_text(
        "<!doctype html><meta charset='utf-8'>"
        f"<title>{html.escape(MODELS[mode]['model'])} mission report</title>"
        "<style>body{font-family:system-ui,sans-serif;margin:2rem;color:#111827}"
        "h1{font-size:1.4rem}.scope{color:#374151}"
        "pre{white-space:pre-wrap;background:#f3f4f6;padding:1rem}"
        "img{max-width:100%;height:auto;border:1px solid #d1d5db}</style>"
        f"<h1>{html.escape(MODELS[mode]['model'])}</h1>"
        "<p class='scope'><strong>Trace report only — this is not flight "
        "acceptance or mission qualification.</strong></p>"
        f"<p>Mode: {html.escape(mode)}; aiding: "
        f"{html.escape(MODELS[mode]['aiding'])}</p>"
        f"<img src='data:image/png;base64,{image_data}' "
        f"alt='{html.escape(mode)} mission plots'>"
        f"<h2>Machine-readable report</h2><pre>{html.escape(report_text)}</pre>\n",
        encoding="utf-8",
    )
    print(f"wrote {png_path}")
    print(f"wrote {html_path}")
    print(f"wrote {report_path}")
    print("scope: trace report only; no flight-acceptance verdict")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        allow_abbrev=False,
        description=(
            "Run and plot one explicit RDD2 waypoint mission with OpenModelica "
            "and/or Rumoca. Optical selects Vehicles.Rdd2.Test.WaypointMission; "
            "GPS selects Vehicles.Rdd2.Test.GlobalWaypointMission."
        ),
    )
    parser.add_argument("--mode", choices=sorted(MODELS), required=True)
    parser.add_argument(
        "--engine",
        choices=("both", "omc", "rumoca"),
        default="both",
        help="solver traces to produce (default: both)",
    )
    parser.add_argument(
        "--stop-time",
        type=float,
        default=DEFAULT_STOP_TIME_S,
        help=f"mission stop time in seconds (default: {DEFAULT_STOP_TIME_S:g})",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=DEFAULT_STEP_S,
        help=f"output step in seconds (default: {DEFAULT_STEP_S:g})",
    )
    parser.add_argument("--tolerance", type=float, default=1.0e-8)
    parser.add_argument("--omc-solver", default="dassl")
    parser.add_argument("--rumoca-solver", default="rk-like")
    parser.add_argument(
        "--omc-csv",
        type=Path,
        help="replay this OpenModelica CSV (requires --omc-receipt)",
    )
    parser.add_argument(
        "--omc-receipt",
        type=Path,
        help="JSON provenance receipt for --omc-csv",
    )
    parser.add_argument(
        "--rumoca-csv",
        type=Path,
        help="replay this Rumoca CSV (requires --rumoca-receipt)",
    )
    parser.add_argument(
        "--rumoca-receipt",
        type=Path,
        help="JSON provenance receipt for --rumoca-csv",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "artifact directory (default: artifacts/vehicles/rdd2/mission-plots/<mode>)"
        ),
    )
    args = parser.parse_args()
    numeric_arguments = {
        "--stop-time": args.stop_time,
        "--step": args.step,
        "--tolerance": args.tolerance,
    }
    if any(
        not math.isfinite(value) or value <= 0.0 for value in numeric_arguments.values()
    ):
        parser.error("--stop-time, --step, and --tolerance must be positive and finite")
    if args.step > args.stop_time:
        parser.error("--step cannot exceed --stop-time")
    interval_count = args.stop_time / args.step
    if not math.isclose(
        interval_count, round(interval_count), rel_tol=0.0, abs_tol=1.0e-9
    ):
        parser.error("--stop-time must be an integer multiple of --step")
    if args.engine == "omc" and args.rumoca_csv:
        parser.error("--rumoca-csv cannot be used with --engine omc")
    if args.engine == "rumoca" and args.omc_csv:
        parser.error("--omc-csv cannot be used with --engine rumoca")
    for engine in ("omc", "rumoca"):
        csv_path = getattr(args, f"{engine}_csv")
        receipt_path = getattr(args, f"{engine}_receipt")
        if csv_path and not receipt_path:
            parser.error(f"--{engine}-csv requires --{engine}-receipt")
        if receipt_path and not csv_path:
            parser.error(f"--{engine}-receipt requires --{engine}-csv")
    return args


def require_live_compiler(engine: str) -> Path | None:
    expected = EXPECTED_COMPILERS[engine]
    environment_name = f"RDD2_MISSION_{engine.upper()}_REVISION"
    revision = os.environ.get(environment_name)
    if revision != expected["revision"]:
        if engine == "rumoca":
            raise RuntimeError(
                "live Rumoca reporting requires frozen compiler revision "
                f"{expected['revision']}; the portable flake pin does not yet expose "
                "that remote revision. Use verified replay with --rumoca-csv and "
                "--rumoca-receipt."
            )
        raise RuntimeError(
            f"live {expected['name']} identity is not receipted: "
            f"{environment_name}={revision!r}, expected {expected['revision']}"
        )
    if engine == "omc":
        return declared_executable("RDD2_MISSION_OMC")
    return None


def main() -> int:
    args = parse_args()
    python_identity = require_declared_python()
    root = repository_root()
    revision = model_revision(root)
    mode = MODELS[args.mode]
    output_dir = args.output_dir or (
        root / "artifacts/vehicles/rdd2/mission-plots" / args.mode
    )
    requested = ("omc", "rumoca") if args.engine == "both" else (args.engine,)
    live_engines = tuple(
        engine for engine in requested if getattr(args, f"{engine}_csv") is None
    )
    live_model_inputs = (
        require_clean_model_inputs(root, revision) if live_engines else None
    )
    traces: dict[str, dict[str, list[float]]] = {}
    provenance: dict[str, dict[str, object]] = {}

    if "omc" in requested:
        if args.omc_csv:
            path = args.omc_csv.expanduser().resolve()
            receipt_path = args.omc_receipt.expanduser().resolve()
            provenance["omc"] = validate_receipt(
                receipt_path,
                path,
                "omc",
                args.mode,
                revision,
                args.stop_time,
                args.step,
            )
            traces["omc"] = read_selected_csv(path)
        else:
            omc_executable = require_live_compiler("omc")
            assert omc_executable is not None
            traces["omc"], provenance["omc"] = run_openmodelica(
                root,
                mode["model"],
                args.stop_time,
                args.step,
                args.tolerance,
                args.omc_solver,
                omc_executable,
            )
            provenance["omc"]["python"] = python_identity
            provenance["omc"]["model_inputs"] = live_model_inputs

    if "rumoca" in requested:
        if args.rumoca_csv:
            path = args.rumoca_csv.expanduser().resolve()
            receipt_path = args.rumoca_receipt.expanduser().resolve()
            provenance["rumoca"] = validate_receipt(
                receipt_path,
                path,
                "rumoca",
                args.mode,
                revision,
                args.stop_time,
                args.step,
            )
            traces["rumoca"] = read_selected_csv(path)
        else:
            require_live_compiler("rumoca")
            traces["rumoca"], provenance["rumoca"] = run_rumoca(
                root / "Vehicles/Rdd2/Test" / mode["scenario"],
                args.stop_time,
                args.step,
                args.tolerance,
                args.rumoca_solver,
            )
            provenance["rumoca"]["python"] = python_identity
            provenance["rumoca"]["model_inputs"] = live_model_inputs

    if live_engines:
        final_model_inputs = require_clean_model_inputs(root, revision)
        if final_model_inputs != live_model_inputs:
            raise RuntimeError("live model inputs changed during simulation")

    return write_outputs(
        traces,
        provenance,
        args.mode,
        output_dir,
        args.stop_time,
        args.step,
        revision,
        python_identity,
    )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (KeyError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2) from error
