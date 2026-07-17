#!/usr/bin/env python3
"""Compare canonical vehicle trajectory logs and render overlay plots."""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import html
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


FIELDS = (
    "time_s",
    "x_m",
    "y_m",
    "z_m",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
)
ANGLE_FIELDS = frozenset(("roll_rad", "pitch_rad", "yaw_rad"))


@dataclass(frozen=True)
class Trajectory:
    label: str
    path: Path
    rows: tuple[dict[str, float], ...]

    @property
    def duration(self) -> float:
        return self.rows[-1]["time_s"]


@dataclass(frozen=True)
class Limits:
    duration_delta_s: float
    position_rmse_m: float
    position_p95_m: float
    altitude_rmse_m: float
    attitude_p95_deg: float


def labeled_path(value: str) -> tuple[str, Path]:
    label, separator, path = value.partition("=")
    if not separator or not label.strip() or not path.strip():
        raise argparse.ArgumentTypeError("expected LABEL=PATH")
    return label.strip(), Path(path).expanduser()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reference",
        required=True,
        type=labeled_path,
        metavar="LABEL=PATH",
        help="trajectory log used as the comparison gold standard",
    )
    parser.add_argument(
        "--candidate",
        required=True,
        action="append",
        type=labeled_path,
        metavar="LABEL=PATH",
        help="trajectory log to compare; repeat for every candidate",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-delta-max-s", type=float, default=0.25)
    parser.add_argument("--position-rmse-max-m", type=float, default=1.0)
    parser.add_argument("--position-p95-max-m", type=float, default=2.0)
    parser.add_argument("--altitude-rmse-max-m", type=float, default=0.5)
    parser.add_argument("--attitude-p95-max-deg", type=float, default=15.0)
    return parser.parse_args()


def read_trajectory(label: str, path: Path) -> Trajectory:
    resolved = path.resolve()
    with resolved.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = [field for field in FIELDS if field not in (reader.fieldnames or ())]
        if missing:
            raise ValueError(f"{resolved} is missing canonical fields: {', '.join(missing)}")
        parsed = [
            {field: float(row[field]) for field in FIELDS}
            for row in reader
        ]
    if len(parsed) < 2:
        raise ValueError(f"{resolved} must contain at least two trajectory samples")
    start = parsed[0]["time_s"]
    previous = -math.inf
    normalized: list[dict[str, float]] = []
    for row in parsed:
        if not all(math.isfinite(row[field]) for field in FIELDS):
            raise ValueError(f"{resolved} contains a non-finite trajectory value")
        sample = dict(row)
        sample["time_s"] -= start
        if sample["time_s"] <= previous:
            raise ValueError(f"{resolved} timestamps must be strictly increasing")
        normalized.append(sample)
        previous = sample["time_s"]
    return Trajectory(label=label, path=resolved, rows=tuple(normalized))


def interpolate(trajectory: Trajectory, times: list[float]) -> dict[str, list[float]]:
    source_times = [row["time_s"] for row in trajectory.rows]
    result = {field: [] for field in FIELDS if field != "time_s"}
    for time_s in times:
        right = bisect.bisect_left(source_times, time_s)
        if right == 0:
            left = right = 0
        elif right >= len(source_times):
            left = right = len(source_times) - 1
        else:
            left = right - 1
        left_row = trajectory.rows[left]
        right_row = trajectory.rows[right]
        span = right_row["time_s"] - left_row["time_s"]
        fraction = 0.0 if span == 0.0 else (time_s - left_row["time_s"]) / span
        for field in result:
            delta = right_row[field] - left_row[field]
            if field in ANGLE_FIELDS:
                delta = wrap_angle(delta)
            value = left_row[field] + fraction * delta
            result[field].append(wrap_angle(value) if field in ANGLE_FIELDS else value)
    return result


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def rmse(values: Iterable[float]) -> float:
    samples = list(values)
    return math.sqrt(fmean(value * value for value in samples))


def compare(reference: Trajectory, candidate: Trajectory, limits: Limits) -> dict[str, object]:
    overlap = min(reference.duration, candidate.duration)
    reference_rows = [row for row in reference.rows if row["time_s"] <= overlap]
    times = [row["time_s"] for row in reference_rows]
    sampled = interpolate(candidate, times)
    position_errors: list[float] = []
    altitude_errors: list[float] = []
    attitude_errors_deg: list[float] = []
    for index, row in enumerate(reference_rows):
        dx = sampled["x_m"][index] - row["x_m"]
        dy = sampled["y_m"][index] - row["y_m"]
        dz = sampled["z_m"][index] - row["z_m"]
        position_errors.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        altitude_errors.append(abs(dz))
        angle_errors = [
            wrap_angle(sampled[field][index] - row[field])
            for field in ("roll_rad", "pitch_rad", "yaw_rad")
        ]
        attitude_errors_deg.append(
            math.degrees(math.sqrt(sum(value * value for value in angle_errors)))
        )

    metrics = {
        "reference_duration_s": reference.duration,
        "candidate_duration_s": candidate.duration,
        "duration_delta_s": abs(candidate.duration - reference.duration),
        "overlap_s": overlap,
        "samples_compared": len(times),
        "position_rmse_m": rmse(position_errors),
        "position_p95_m": percentile(position_errors, 0.95),
        "position_max_m": max(position_errors),
        "altitude_rmse_m": rmse(altitude_errors),
        "attitude_p95_deg": percentile(attitude_errors_deg, 0.95),
        "attitude_max_deg": max(attitude_errors_deg),
    }
    checks = {
        "duration": metrics["duration_delta_s"] <= limits.duration_delta_s,
        "position_rmse": metrics["position_rmse_m"] <= limits.position_rmse_m,
        "position_p95": metrics["position_p95_m"] <= limits.position_p95_m,
        "altitude_rmse": metrics["altitude_rmse_m"] <= limits.altitude_rmse_m,
        "attitude_p95": metrics["attitude_p95_deg"] <= limits.attitude_p95_deg,
    }
    return {
        "label": candidate.label,
        "path": str(candidate.path),
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": metrics,
        "series": {
            "time_s": times,
            "position_error_m": position_errors,
            "altitude_error_m": altitude_errors,
            "attitude_error_deg": attitude_errors_deg,
        },
    }


def column(trajectory: Trajectory, name: str) -> list[float]:
    return [row[name] for row in trajectory.rows]


def save(fig: plt.Figure, output: Path, name: str) -> Path:
    path = output / name
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(f"wrote {path}")
    return path


def plot_overview(
    trajectories: list[Trajectory], comparisons: list[dict[str, object]], output: Path
) -> Path:
    fig = plt.figure(figsize=(15, 13), constrained_layout=True)
    ax_xy = fig.add_subplot(3, 2, 1)
    ax_3d = fig.add_subplot(3, 2, 2, projection="3d")
    ax_alt = fig.add_subplot(3, 2, 3)
    ax_pos = fig.add_subplot(3, 2, 4)
    ax_att = fig.add_subplot(3, 2, 5)
    ax_error = fig.add_subplot(3, 2, 6)
    for trajectory in trajectories:
        time_s = column(trajectory, "time_s")
        x_m = column(trajectory, "x_m")
        y_m = column(trajectory, "y_m")
        z_m = column(trajectory, "z_m")
        line = ax_xy.plot(x_m, y_m, label=trajectory.label, linewidth=1.4)[0]
        color = line.get_color()
        ax_3d.plot(x_m, y_m, z_m, label=trajectory.label, color=color, linewidth=1.2)
        ax_alt.plot(time_s, z_m, label=trajectory.label, color=color)
        ax_pos.plot(time_s, x_m, label=f"{trajectory.label} x", color=color)
        ax_pos.plot(time_s, y_m, label=f"{trajectory.label} y", color=color, linestyle="--")
        ax_att.plot(
            time_s,
            [math.degrees(value) for value in column(trajectory, "roll_rad")],
            label=f"{trajectory.label} roll",
            color=color,
        )
        ax_att.plot(
            time_s,
            [math.degrees(value) for value in column(trajectory, "pitch_rad")],
            label=f"{trajectory.label} pitch",
            color=color,
            linestyle="--",
        )
        ax_att.plot(
            time_s,
            [math.degrees(value) for value in column(trajectory, "yaw_rad")],
            label=f"{trajectory.label} yaw",
            color=color,
            linestyle=":",
        )
    for comparison in comparisons:
        series = comparison["series"]
        assert isinstance(series, dict)
        ax_error.plot(
            series["time_s"], series["position_error_m"], label=str(comparison["label"])
        )
    ax_xy.set(title="Top-down trajectory", xlabel="x [m]", ylabel="y [m]")
    ax_xy.axis("equal")
    ax_3d.set(title="Three-dimensional trajectory", xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
    ax_alt.set(title="Altitude", xlabel="elapsed time [s]", ylabel="z [m]")
    ax_pos.set(title="Horizontal position", xlabel="elapsed time [s]", ylabel="position [m]")
    ax_att.set(title="Attitude", xlabel="elapsed time [s]", ylabel="angle [deg]")
    ax_error.set(title="Position error from reference log", xlabel="elapsed time [s]", ylabel="error [m]")
    for axis in (ax_xy, ax_alt, ax_pos, ax_att, ax_error):
        axis.grid(True)
        axis.legend(loc="best", fontsize=8)
    ax_3d.legend(loc="best", fontsize=8)
    return save(fig, output, "trajectory-comparison.png")


def plot_components(trajectories: list[Trajectory], output: Path) -> list[Path]:
    paths: list[Path] = []
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    for trajectory in trajectories:
        time_s = column(trajectory, "time_s")
        for axis, field in zip(axes, ("x_m", "y_m", "z_m"), strict=True):
            axis.plot(time_s, column(trajectory, field), label=trajectory.label)
    for axis, label in zip(axes, ("x [m]", "y [m]", "z [m]"), strict=True):
        axis.set_ylabel(label)
        axis.grid(True)
        axis.legend(loc="best")
    axes[-1].set_xlabel("elapsed time [s]")
    paths.append(save(fig, output, "trajectory-position.png"))

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    for trajectory in trajectories:
        time_s = column(trajectory, "time_s")
        for axis, field in zip(
            axes, ("roll_rad", "pitch_rad", "yaw_rad"), strict=True
        ):
            axis.plot(
                time_s,
                [math.degrees(value) for value in column(trajectory, field)],
                label=trajectory.label,
            )
    for axis, label in zip(axes, ("roll [deg]", "pitch [deg]", "yaw [deg]"), strict=True):
        axis.set_ylabel(label)
        axis.grid(True)
        axis.legend(loc="best")
    axes[-1].set_xlabel("elapsed time [s]")
    paths.append(save(fig, output, "trajectory-attitude.png"))
    return paths


def plot_errors(comparisons: list[dict[str, object]], output: Path) -> Path:
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    for comparison in comparisons:
        series = comparison["series"]
        assert isinstance(series, dict)
        label = str(comparison["label"])
        axes[0].plot(series["time_s"], series["position_error_m"], label=label)
        axes[1].plot(series["time_s"], series["altitude_error_m"], label=label)
        axes[2].plot(series["time_s"], series["attitude_error_deg"], label=label)
    for axis, label in zip(
        axes,
        ("position error [m]", "altitude error [m]", "attitude error [deg]"),
        strict=True,
    ):
        axis.set_ylabel(label)
        axis.grid(True)
        axis.legend(loc="best")
    axes[-1].set_xlabel("elapsed time [s]")
    return save(fig, output, "trajectory-errors.png")


def reports(
    reference: Trajectory,
    comparisons: list[dict[str, object]],
    limits: Limits,
    plots: list[Path],
    output: Path,
) -> None:
    serializable = []
    for comparison in comparisons:
        serializable.append({key: value for key, value in comparison.items() if key != "series"})
    document = {
        "passed": all(bool(item["passed"]) for item in comparisons),
        "reference": {"label": reference.label, "path": str(reference.path)},
        "limits": limits.__dict__,
        "comparisons": serializable,
    }
    (output / "trajectory-comparison.json").write_text(
        json.dumps(document, indent=2) + "\n", encoding="utf-8"
    )
    lines = [
        "# Trajectory log comparison",
        "",
        f"Gold-standard log: `{reference.label}` (`{reference.path}`)",
        "",
        f"Result: **{'PASS' if document['passed'] else 'FAIL'}**",
        "",
        "| log | result | duration delta [s] | position RMSE [m] | position p95 [m] | altitude RMSE [m] | attitude p95 [deg] |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for comparison in comparisons:
        metrics = comparison["metrics"]
        assert isinstance(metrics, dict)
        lines.append(
            f"| {comparison['label']} | {'PASS' if comparison['passed'] else 'FAIL'} | "
            f"{metrics['duration_delta_s']:.6g} | {metrics['position_rmse_m']:.6g} | "
            f"{metrics['position_p95_m']:.6g} | {metrics['altitude_rmse_m']:.6g} | "
            f"{metrics['attitude_p95_deg']:.6g} |"
        )
    lines.extend(["", "## Plots", ""])
    lines.extend(f"- `{path.name}`" for path in plots)
    markdown = "\n".join(lines) + "\n"
    (output / "trajectory-comparison.md").write_text(markdown, encoding="utf-8")
    images = "\n".join(
        f'<h2>{html.escape(path.stem.replace("-", " ").title())}</h2>'
        f'<img src="{html.escape(path.name)}" alt="{html.escape(path.stem)}">'
        for path in plots
    )
    (output / "trajectory-comparison.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>Trajectory comparison</title>"
        "<style>body{font-family:system-ui;margin:2rem;max-width:1500px}"
        "img{max-width:100%;height:auto;border:1px solid #ccc}</style>"
        f"<body><pre>{html.escape(markdown)}</pre>{images}</body>\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    limits = Limits(
        duration_delta_s=args.duration_delta_max_s,
        position_rmse_m=args.position_rmse_max_m,
        position_p95_m=args.position_p95_max_m,
        altitude_rmse_m=args.altitude_rmse_max_m,
        attitude_p95_deg=args.attitude_p95_max_deg,
    )
    if any(value < 0.0 or not math.isfinite(value) for value in limits.__dict__.values()):
        raise ValueError("comparison limits must be finite and non-negative")
    reference = read_trajectory(*args.reference)
    candidates = [read_trajectory(*value) for value in args.candidate]
    labels = [reference.label, *(candidate.label for candidate in candidates)]
    if len(labels) != len(set(labels)):
        raise ValueError("trajectory labels must be unique")
    args.output.mkdir(parents=True, exist_ok=True)
    comparisons = [compare(reference, candidate, limits) for candidate in candidates]
    trajectories = [reference, *candidates]
    plots = [
        plot_overview(trajectories, comparisons, args.output),
        *plot_components(trajectories, args.output),
        plot_errors(comparisons, args.output),
    ]
    reports(reference, comparisons, limits, plots, args.output)
    print(f"wrote {args.output / 'trajectory-comparison.md'}")
    return 0 if all(bool(item["passed"]) for item in comparisons) else 1


if __name__ == "__main__":
    raise SystemExit(main())
