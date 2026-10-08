#!/usr/bin/env python3
"""Render the scored capture matrix and selected traces as review artifacts."""

import argparse
import csv
import html
import json
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from score import errors, read

matplotlib.rcParams["svg.fonttype"] = "path"
NAMES = {
    "modelica": "Modelica ESKF",
    "ukf": "Modelica UKF",
    "px4": "PX4 EKF2",
    "ekf3": "ArduPilot EKF3",
}
COLORS = {"modelica": "#2563eb", "ukf": "#737373", "px4": "#059669", "ekf3": "#c2410c"}
TRACKING = list(NAMES)
KEYS = {
    "horizontal_position_rmse_m": "Horizontal RMSE (m)",
    "vertical_position_rmse_m": "Vertical RMSE (m)",
    "velocity_3d_rmse_m_s": "Velocity 3D RMSE (m/s)",
    "yaw_rmse_deg": "Yaw RMSE (deg)",
}


def grouped(records):
    groups = {}
    for x in records:
        groups.setdefault((x["profile"], x["speed"], x["case"], x["name"]), []).append(
            x
        )
    return groups


def cell(values):
    good = [v for v in values if v is not None]
    if not good:
        return "not reached"
    result = f"{median(good):.3f} [{min(good):.3f}, {max(good):.3f}]"
    return result + (
        f"; {len(values) - len(good)} failed" if len(good) < len(values) else ""
    )


def render(args):
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    records = sum([json.loads(p.read_text()) for p in args.scores], [])
    groups = grouped(records)
    profiles = list(dict.fromkeys(x["profile"] for x in records))
    speeds = sorted(set(x["speed"] for x in records))
    # A flat CSV is convenient for independent reanalysis in a spreadsheet.
    flat = []
    for x in records:
        for phase in ["flight", "outage_window", "before_outage", "after_return"]:
            flat.append(
                {
                    **{k: x[k] for k in ["profile", "speed", "seed", "case", "name"]},
                    "phase": phase,
                    **x[phase],
                }
            )
    with (output / "estimator-metrics.csv").open("w") as f:
        writer = csv.DictWriter(
            f, sorted(set().union(*(x.keys() for x in flat))), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(flat)
    figures = []
    fig, axes = plt.subplots(len(profiles), 3, figsize=(13, 7), constrained_layout=True)
    for i, profile in enumerate(profiles):
        for j, case in enumerate(["gps", "denied", "transition"]):
            ax = axes[i, j]
            for k, name in enumerate(TRACKING):
                for n, speed in enumerate(speeds):
                    phase = "outage_window" if case == "transition" else "flight"
                    values = [
                        x[phase]["horizontal_position_rmse_m"]
                        for x in groups[profile, speed, case, name]
                    ]
                    mid = median(values)
                    xloc = k + (n - 0.5) * 0.32
                    ax.bar(
                        xloc, mid, width=0.3, color=COLORS[name], alpha=1 if n else 0.55
                    )
                    ax.errorbar(
                        xloc,
                        mid,
                        yerr=[[mid - min(values)], [max(values) - mid]],
                        color="black",
                        capsize=2,
                        linewidth=1,
                    )
            ax.set_xticks(range(4), ["ESKF", "UKF", "EKF2", "EKF3"])
            ax.set_title(f"{case}: {profile}")
            ax.set_ylabel("Horizontal RMSE (m)")
            ax.grid(axis="y", alpha=0.2)
    fig.suptitle(
        "Same measured captures; pale = 0.25 rad/s, solid = 0.60 rad/s motion frequency\n"
        "Median and min–max of three seeds; high-rate denied UKF includes rejected predictions"
    )
    fig.savefig(output / "horizontal-comparison.svg")
    plt.close(fig)
    figures.append("horizontal-comparison.svg")
    for case in ["denied", "transition"]:
        fig, axes = plt.subplots(
            3, 2, figsize=(12, 8), constrained_layout=True, sharex=True
        )
        for col, profile in enumerate(profiles):
            low = profile.startswith("25/")
            truth = read(
                args.data_root / ("lower_0.6_7" if low else "0.6_7") / "truth.csv"
            )
            results = args.lower_results if low else args.results
            for name in TRACKING:
                record = next(
                    x for x in groups[profile, 0.6, case, name] if x["seed"] == 7
                )
                d = read(results / record["csv"])
                d = d[(d["t_s"] >= 13) & (d["t_s"] < 60)]
                p, _, _, yaw = errors(d, truth)
                for row, values in enumerate(
                    [np.linalg.norm(p[:, :2], axis=1), p[:, 2], yaw]
                ):
                    axes[row, col].plot(
                        d["t_s"][::5],
                        values[::5],
                        color=COLORS[name],
                        linestyle="--" if name == "ukf" else "-",
                        label=NAMES[name],
                        linewidth=1,
                    )
            axes[0, col].set_title(profile)
            for row, label in enumerate(
                ["Horizontal error (m)", "Vertical error (m)", "Yaw error (deg)"]
            ):
                axes[row, col].set_ylabel(label)
                axes[row, col].grid(alpha=0.2)
                if case == "transition":
                    axes[row, col].axvspan(25, 40, color="gray", alpha=0.15)
            axes[2, col].set_xlabel("Capture time (s)")
        axes[0, 0].legend(fontsize=8)
        fig.suptitle(
            f"{case}: motion frequency 0.60 rad/s, seed 7; gray band = GPS absent"
        )
        name = case + "-errors.svg"
        fig.savefig(output / name)
        plt.close(fig)
        figures.append(name)
    parts = [INTRO]
    for figure in figures:
        parts.append(f'<figure><img src="{figure}" alt="{figure}"></figure>')
    parts.append(
        "<h2>Accuracy by phase</h2><p>Every cell is median [minimum, maximum] across seeds 7, 19, 41. "
        "There are only three seeds: these ranges are not confidence intervals. GPS and denied summaries "
        "use 13–60 s; transition summaries below use the 25–40 s outage. Full-flight, pre-outage and "
        "post-return results for every run are in the linked JSON and CSV.</p>"
    )
    for profile in profiles:
        for speed in speeds:
            parts.append(
                f"<h3>Flow / mag / barometer {profile}; motion frequency {speed:.2f} rad/s</h3>"
            )
            parts.append(
                "<table><thead><tr><th>Scenario</th><th>Estimator</th>"
                + "".join(f"<th>{v}</th>" for v in KEYS.values())
                + "<th>Position valid</th><th>Prediction accepted</th></tr></thead><tbody>"
            )
            for case in ["gps", "denied", "transition"]:
                phase = "outage_window" if case == "transition" else "flight"
                for name in NAMES:
                    a = groups[profile, speed, case, name]
                    cells = [case, NAMES[name]] + [
                        cell([x[phase].get(k) for x in a]) for k in KEYS
                    ]
                    cells += [
                        cell([x[phase]["position_valid_fraction"] for x in a]),
                        cell([x[phase].get("prediction_accepted_fraction") for x in a])
                        if "prediction_accepted_fraction" in a[0][phase]
                        else "not exported",
                    ]
                    parts.append(
                        "<tr>"
                        + "".join(f"<td>{html.escape(v)}</td>" for v in cells)
                        + "</tr>"
                    )
            parts.append("</tbody></table>")
    parts.append(
        "<h2>GPS reacquisition</h2><p>GPS resumes at 40 s. Time to error below 0.25 m for one second can "
        "be zero when flow already keeps the estimate within tolerance. It is not a GPS-fusion latency. "
        "Native GPS flags denote active aiding; Modelica flags denote an accepted correction. "
        "Error jump is max norm of successive position-error changes in 39–45 s; output times differ "
        "slightly, so it is an operational discontinuity metric, not an instantaneous reset magnitude.</p>"
    )
    parts.append(
        "<table><tr><th>Rates</th><th>Motion</th><th>Estimator</th><th>First GPS use after return (s)</th>"
        "<th>Below 0.25 m for 1 s (s)</th><th>Max error jump (m)</th></tr>"
    )
    for profile in profiles:
        for speed in speeds:
            for name in NAMES:
                a = groups[profile, speed, "transition", name]
                cells = [profile, str(speed), NAMES[name]] + [
                    cell([x["transition"][k] for x in a])
                    for k in [
                        "first_gps_use_after_return_s",
                        "recovery_to_025m_for_1s_s",
                        "max_return_error_jump_m",
                    ]
                ]
                parts.append(
                    "<tr>"
                    + "".join(f"<td>{html.escape(v)}</td>" for v in cells)
                    + "</tr>"
                )
    parts.append(
        "</table><h2>Fusion activity and coverage</h2><p>All values below cover 13–60 s. "
        "Native active fractions do not prove individual measurement acceptance. Modelica accepted Hz "
        "counts per-step acceptances. Finite invalid rows remain in all accuracy metrics; a nonfinite "
        "row suppresses the accuracy metric for that window. Missing timestamps are exposed by "
        "coverage and maximum output gap. The UKF's valid flag stays true during rejected predictions.</p>"
    )
    parts.append(
        "<table><tr><th>Rates</th><th>Motion</th><th>Scenario</th><th>Estimator</th><th>GPS</th>"
        "<th>Flow</th><th>Mag</th><th>Baro</th><th>Coverage</th><th>Finite</th><th>Max gap (s)</th></tr>"
    )
    for key, a in groups.items():
        profile, speed, case, name = key
        cells = [profile, str(speed), case, NAMES[name]]
        for sensor in ["gps", "flow", "mag", "baro"]:
            k = sensor + (
                "_fused_accepted_hz"
                if name in ["modelica", "ukf"]
                else "_active_fraction"
            )
            cells.append(
                cell([x["flight"].get(k) for x in a])
                + (" Hz" if name in ["modelica", "ukf"] else " fraction")
                if k in a[0]["flight"]
                else "not exported"
            )
        cells += [
            cell([x["flight"][k] for x in a])
            for k in ["row_coverage", "finite_fraction", "max_output_gap_s"]
        ]
        parts.append(
            "<tr>" + "".join(f"<td>{html.escape(v)}</td>" for v in cells) + "</tr>"
        )
    parts.append(OUTRO)
    (output / "estimator-comparison.html").write_text("\n".join(parts))
    for figure in figures:
        path = output / figure
        path.write_text(
            "\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n"
        )


INTRO = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Estimator comparison — 7 October 2026</title>
<style>body{font:16px/1.55 system-ui,sans-serif;max-width:1350px;margin:35px auto;padding:0 22px;color:#172334}
h1,h2,h3{line-height:1.25}p,li{max-width:1000px}table{border-collapse:collapse;font-size:12px;width:100%;margin:18px 0 35px}
td,th{padding:8px;border-bottom:1px solid #d9e1ec;text-align:left}th{background:#edf2f7}tr:nth-child(even){background:#f7f9fc}
figure{margin:25px 0}img{width:100%;height:auto}code{background:#eef2f6;padding:2px 4px}a{color:#1659b7}
@media print{body{margin:0;font-size:11px}table{font-size:9px}figure,table{break-inside:avoid}}</style>
<h1>GPS, GPS-denied, and GPS loss/recovery estimation</h1><p>7 October 2026 · 144 native/generated-code replays ·
two trajectories · three noise seeds · two aiding-rate profiles · independent analytic truth.</p>
<p><strong>Finding:</strong> native PX4 EKF2 has the lowest velocity error and usually the lowest GPS-denied horizontal
error in these controlled captures. The Modelica ESKF and UKF are competitive with GPS and sometimes have lower
horizontal error. Both Modelica filters' single-correction priority chains starve magnetic yaw updates at 100 Hz
optical flow. At 25 Hz aiding, magnetic fusion resumes and GPS-denied performance improves substantially.
Altitude performance depends on the profile; PX4 has larger vertical errors at the lower rates.
These are as-configured stack results, not a universal algorithm ranking.</p>
<p>For the faster trajectory, medians at 100/50/50 Hz: GPS-denied horizontal RMSE is approximately 0.10 m for EKF2,
0.20 m for EKF3, 2.58 m for ESKF and 4.62 m for UKF. The high-rate denied UKF rejects some predictions, making its
error a runtime/configuration failure metric. At 25/25/25 Hz the corresponding errors are approximately 0.10 m,
0.19 m, 0.38 m and 0.45 m, with uninterrupted prediction. PX4 altitude RMSE worsens at the lower rates
(about 0.63–0.82 m); there is no single best stack across all axes.</p>
<h2>What was compared</h2>
<table><tr><th>Stack</th><th>Executable implementation</th><th>Configuration and interpretation</th></tr>
<tr><td>Modelica ESKF</td><td>Vehicles.Rdd2.NavigationEstimator, generated C99 by released Rumoca 0.10.2</td>
<td>Actual 800 Hz FOH preintegration and analytic bias Jacobians, 100 Hz filter; RDD2 process noise/initial variances.
Pseudo-position and zero-velocity corrections disabled to avoid a trajectory-dependent motion oracle.</td></tr>
<tr><td>Modelica UKF</td><td>Estimation.StrapdownINS.UKF.Estimator, same compiler/preintegrator</td>
<td>Declared UKF initial variances; RDD2 process noise. An explicit per-sigma intermediate avoids a released compiler
reduction bug. Prediction is continuous for GPS, transition and lower-rate denied runs. High-rate denied runs
still reject some predictions; all finite outputs remain scored, and prediction acceptance is reported separately.</td></tr>
<tr><td>PX4 EKF2</td><td>Unmodified native EKF source at
<a href="https://github.com/PX4/PX4-Autopilot/tree/f1c0a1f794edf8e5e974b6ed96df3f95eda0df39/src/modules/ekf2/EKF">f1c0a1f794ed</a></td>
<td>GPS, flow, range, magnetometer and barometer enabled; barometer height reference, conditional range aid,
declared magnetic declination. Other native noise defaults retained.</td></tr>
<tr><td>ArduPilot EKF3</td><td>Unmodified Tools/Replay / AP_NavEKF3 at
<a href="https://github.com/ArduPilot/ardupilot/tree/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AP_NavEKF3">1511f27194f1</a> (Copter 4.7.0)</td>
<td>One IMU/mag/barometer; GPS position/velocity, flow fallback/parallel velocity, compass yaw, barometer height.
Other native noise defaults retained. Relative position validity is accepted during GPS denial.</td></tr></table>
<p>PX4's public guidance describes its delayed fusion horizon and optical flow requirements;
<a href="https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf">PX4 navigation filter documentation</a>.
ArduPilot documents configurable source sets and optical-flow operation;
<a href="https://ardupilot.org/copter/docs/common-ekf-sources.html">source selection</a> and
<a href="https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html">flow setup</a>.
The measured results below come from the pinned implementations and harness configuration, not these moving guides.</p>
<p>Trace figures display every fifth output sample (approximately 20 Hz); all metrics use every output sample.</p>
<h2>Capture and scoring contract</h2><ul>
<li>60 s analytic ENU/FLU trajectory with independent position, velocity, attitude, angular rate and specific force.
13 s stationary noisy startup, then a 4 m × 3 m periodic horizontal path and 2 m climb. Horizontal frequencies
0.25 and 0.60 rad/s; roll/pitch/yaw vary independently. This is kinematic replay, not an airframe/closed-loop simulation.</li>
<li>800 Hz IMU; 10 Hz GPS; high profile flow/mag/barometer 100/50/50 Hz, lower profile 25/25/25 Hz.
Lower-rate captures downsample the same noise realizations. Seeds 7, 19, 41. Sensor latency is zero in this baseline.</li>
<li>Gyro bias [0.0008, −0.0005, 0.0004] rad/s and accelerometer bias [0.02, −0.01, 0.015] m/s².
Per-sample white standard deviations: gyro 0.0015 rad/s, accelerometer 0.03 m/s²; GPS position
[0.2, 0.2, 0.35] m and velocity 0.05 m/s; calibrated flow velocity 0.03 m/s, range 0.02 m;
magnetometer 0.3 µT; barometer 0.1 m plus 0.3 m offset and 0.05 sin(0.03t) m drift.</li>
<li>Common barometer frontend subtracts the first five seconds' mean for all stacks; raw pressure-derived altitude
is retained separately. Common known starting origin and flat ground 1 m below startup. No in-flight truth is
provided to estimators. Native drivers transform ENU/FLU to NED/FRD and convert magnetic/flow units/signs.</li>
<li>GPS scenario retains all sensors; denied scenario withholds GPS for the entire capture; transition withholds it
from 25 to 40 s. Flow/range/mag/barometer remain available throughout. No GPS-based trajectory or attitude truth
is used as reference. GPS-denied position is relative to the declared startup origin, not globally observable position.</li>
<li>Scoring interpolates independent truth to each estimator's output timestamps, forbids extrapolation, includes
finite invalid outputs, and reports validity/coverage. Horizontal and 3D errors are Euclidean RMSE, without dividing
by axis count. Attitude uses quaternion geodesic distance and yaw uses wrapped heading error, not course.</li></ul>
<p>Recorded wall times include capture I/O, DataFlash conversion, startup and logging; they do not compare filter CPU
cost. This study measures neither worst-case execution time nor flight-computer resource budgets.</p>
"""

OUTRO = """<h2>Implementation findings and next experiments</h2><ol>
<li><strong>Modelica sensor scheduling:</strong> GPS → barometer → optical flow → magnetometer dispatch permits at most
one correction per filter tick. High-rate flow uses remaining ticks and prevents magnetic yaw updates. The 25 Hz
ablation restores approximately 25 accepted mag corrections/s, reducing denied yaw RMSE from about 30° for ESKF
and 56–60° for UKF to below 1°. This existing scheduling limitation is separate from preintegration accuracy.</li>
<li><strong>UKF generated-code correction:</strong> Rumoca 0.10.2 hoisted an indexed function call out of the mean-error
reduction and repeated sigma point 2 thirty times. A noiseless hover moved 3.87298 m on its first prediction.
Materializing each error vector before accumulation preserves the loop and fixes raw/preintegrated prediction.
The native C hover test now runs in CI; the previous source fails it. The old failed replay scores are retained
as <a href="ukf-before-workaround-scores.json">negative evidence</a>, separate from the corrected tables here.</li>
<li><strong>UKF covariance admission:</strong> matching RDD2's 1e−6 gyro-bias initial variance makes the float32
Cholesky threshold reject the initial prior: 15 × epsilon × max diagonal ≈ 1.79e−6 exceeds that variance.
The comparison retains the declared UKF 1e−4 initial bias variance. After the compiler workaround, high-rate
denied runs still reject some predictions as covariance evolves. This residual failure needs a separate
conditioning investigation. The validity flag alone does not expose it. Do not rank unscented-filter theory
using a configuration that rejects prediction.</li>
<li><strong>GPS return:</strong> inspect all seeds, not only the median. High-rate ESKF has one delayed-recovery seed
with a metre-scale correction jump. PX4 continues flow tracking through the outage and resumes GPS later; EKF3
resumes GPS sooner but can produce a larger position correction. The tables separate recovery from source activation.</li>
<li><strong>Future ranking:</strong> normalize measurement noise/floors and initial uncertainty where feasible; add
nonzero latency, actual camera integration/texture limits, range dropout/terrain changes, magnetic disturbance,
vibration, GPS outliers and longer outages. Then run an independently measured real-flight capture and closed-loop
trials. This baseline uses calibrated body flow velocity to manufacture native camera observations; it does not
test image formation or visual odometry.</li></ol>
<h2>Reproduction and paper review</h2><p>
<a href="estimator-scores.json">High-rate per-run scores</a> ·
<a href="estimator-scores-lower-rates.json">Lower-rate per-run scores</a> ·
<a href="estimator-metrics.csv">All scored phases CSV</a> ·
<a href="capture-manifest.json">Input/output and implementation hashes</a> ·
<a href="preintegration-paper-review.txt">Paper review</a> ·
<a href="rumoca-codegen-review.txt">Compiler compatibility findings</a> ·
<a href="validation.txt">Validation evidence</a> ·
<a href="../../../tools/estimator_comparison/README.txt">Reproduction instructions</a>.</p>
<p>Modelica sources and replay generator/scorer are original project code. Native replay harness changes live
in the separate estimator-comparison repository branch workspace/matched-sensor-replay. Its EKF3 Modelica
directory contains scaffolding only; no completed EKF3 Modelica port was found or published. The existing
private PX4 Modelica port covers a smaller subset than this native full-sensor replay and was not substituted
for the native EKF2 comparator.</p></html>"""


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--scores", type=Path, nargs="+", required=True)
    p.add_argument("--data-root", type=Path, required=True)
    p.add_argument("--results", type=Path, required=True)
    p.add_argument("--lower-results", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    render(p.parse_args())
