#!/usr/bin/env python3
"""Report the altitude experiment without collapsing it into an algorithm ranking."""

import argparse
import csv
import itertools
import json
from statistics import median


METRICS = {
    "horizontal_position_rmse_m": "Horizontal position (m)",
    "vertical_position_rmse_m": "Vertical position (m)",
    "velocity_3d_rmse_m_s": "3D velocity (m/s)",
    "yaw_rmse_deg": "Yaw (degrees)",
}
NAMES = {
    "horizon": "ESKF horizon",
    "retrodiction": "ESKF retrodiction",
    "px4": "PX4 EKF2",
    "ekf3": "ArduPilot EKF3",
}
WINDOWS = ("flight", "outage_window", "after_return")


def report(args):
    paths = [args.output.with_suffix(suffix) for suffix in (".html", ".csv", ".txt")]
    if any(path.exists() for path in paths):
        raise ValueError("Choose new report paths")
    points = []
    activation = []
    profile = None
    primary_hashes = {}
    for dataset, path in (("original", args.original), ("held-out", args.held_out)):
        evidence = json.loads(path.read_text())
        if (
            len(evidence["seeds"]) != 3
            or evidence["aiding_profile"] != "100/50/50 Hz"
            or evidence["delay_profile"] != "nominal"
            or evidence["takeoff_expectation_clear_s"] != 18
        ):
            raise ValueError("Unexpected altitude experiment configuration")
        configuration = (
            evidence["noise_density"],
            evidence["native_source_pins"],
            {
                key: value
                for key, value in evidence["binary_sha256"].items()
                if key != "px4_adapter"
            },
        )
        if profile is not None and profile != configuration:
            raise ValueError("Estimator configuration differs between motion sets")
        profile = configuration
        expected = set(
            itertools.product(
                evidence["seeds"], (2, 4), ("gps", "denied", "transition"), NAMES
            )
        )
        actual = {
            (row["seed"], row["climb_height_m"], row["scenario"], row["name"])
            for row in evidence["scores"]
        }
        if len(actual) != len(evidence["scores"]) or actual != expected:
            raise ValueError("Altitude result matrix is incomplete or duplicated")
        for row in evidence["scores"]:
            primary_hashes[
                (
                    dataset,
                    row["tag"],
                    row["climb_height_m"],
                    row["scenario"],
                    row["name"],
                )
            ] = row["output_sha256"]
            for window in WINDOWS:
                if row[window]["finite_fraction"] != 1:
                    raise ValueError("Nonfinite result")
                points.append(
                    dict(
                        dataset=dataset,
                        seed=row["seed"],
                        climb_height_m=row["climb_height_m"],
                        scenario=row["scenario"],
                        estimator=row["name"],
                        window=window,
                        **{key: row[window][key] for key in METRICS},
                    )
                )
            if row["name"] == "ekf3":
                modes = {
                    item["mag_fusion"]: item["rows"]
                    for item in row["magnetic_selection_rows"]
                }
                activation.append(
                    f"  {dataset} seed {row['seed']} {row['climb_height_m']} m {row['scenario']}: "
                    f"heading {modes.get(1, 0)}, three-axis {modes.get(2, 0)} selection rows"
                )
    with paths[1].open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(points[0]))
        writer.writeheader()
        writer.writerows(points)
    text = """Altitude, magnetic activation and configured-estimator comparison
=================================================================

The native EKF3 magnetic activation difference is caused by mission height:
its unchanged Copter MAG_CAL=3 policy remains heading-only at 2 m and switches
naturally to three-axis field fusion after climbing above its threshold at 4 m.
No magnetic mode is forced in this experiment. The persistent takeoff flag is
corrected for both heights using a declared five-second phase (13 to 18 s).
This approximates the vehicle hint; it is not the full Copter detector.

144 replays cover two heights, original/held-out motion, three seeds per motion,
three GPS scenarios, ESKF retrodiction/horizon and native PX4 EKF2/ArduPilot EKF3.
Only high-rate aiding and nominal transport delays are included here. The 4 m
case preserves horizontal and attitude motion and noise draws, but changes
vertical acceleration, range and flow geometry. Differences cannot be attributed
solely to magnetic activation. The original 2 m mission remains a valid case.

ESKF uses the previous frozen stationary IMU calibration and magnetic-vector
option. Production Modelica defaults are unchanged. Native noise settings,
internal rates, observation/source policies and averaging/delay semantics remain
stack-specific. Neither this experiment nor its plots establish a general
algorithm ranking or universal ESKF superiority. See native-configuration-audit
for the remaining fidelity gaps. Native cores remain unmodified validation
submodules; partial Modelica ports are not used.

Open the sibling HTML for interactive metrics/windows and seed ranges. The CSV
contains every seed/window value. GPS and denied discussion below uses full
flight (13 <= t < 60); transition uses the outage window (25 <= t < 40).
Values below are medians of three seeds, in horizontal m / vertical m /
3D velocity m/s / yaw degrees order. All finite invalid outputs remain scored.

"""
    for dataset, scenario, height in itertools.product(
        ("original", "held-out"), ("gps", "denied", "transition"), (2, 4)
    ):
        window = "outage_window" if scenario == "transition" else "flight"
        text += f"{dataset}, {scenario}, climb {height} m ({window})\n"
        for name, label in NAMES.items():
            selected = [
                row
                for row in points
                if row["dataset"] == dataset
                and row["scenario"] == scenario
                and row["climb_height_m"] == height
                and row["window"] == window
                and row["estimator"] == name
            ]
            text += (
                f"  {label:22s}"
                + " ".join(
                    f"{median(row[key] for row in selected):10.5f}" for key in METRICS
                )
                + "\n"
            )
        text += "\n"
    text += (
        "EKF3 magnetic selection diagnostics (includes the 60 s endpoint)\n"
        + "\n".join(activation)
        + "\n"
    )
    if getattr(args, "covariance", None):
        covariance = json.loads(args.covariance.read_text())["scores"]
        if len(covariance) != 6:
            raise ValueError("Incomplete covariance experiment")
        for row in covariance:
            key = (row["dataset"], row["tag"], 4, "denied", "retrodiction")
            if (
                row["output_sha256"] != primary_hashes[key]
                or row["consistency"]["rows"] != 4700
            ):
                raise ValueError("Covariance probe differs from primary replay")
        text += """
Covariance diagnostic
---------------------

Six additional 4 m denied-flight retrodiction replays have symmetric,
Cholesky-positive covariance at all 4700 flight epochs; their output hashes
match the primary replays. This does not prove statistical consistency. Mean
15-dimensional NEES ranges from 6.39 to 141.61. Original seed 41 is a clear
overconfidence case (141.61, versus 137.42 in the earlier 2 m result), especially
in velocity, attitude and accelerometer bias. It is retained in the comparison.
The samples are time-correlated; NEES is descriptive. Current horizon/native
covariance comparisons remain unavailable. See eskf-altitude-covariance.json.
"""
    text += """
Validation and reproduction
---------------------------

The default generator's nine files match its baseline version byte for byte.
54 unchanged low-altitude ESKF/PX4 outputs match earlier score hashes; six
corrected low-altitude EKF3 outputs match the previous finite-takeoff ablation.
All ESKF/native packet delivery counts and payload digests agree. ESKF horizon
queues report no late/stale/overflow packets. Native logs verify actual magnetic
selection, and native sources remain at their unchanged pins. The manifest
records exact sources, binaries, captures, controls and any additional checks.

Use compare_altitude.py --help with the previous frozen calibration, vector
retrodiction/horizon binaries, pinned native cores, external harness and native/
ESKF/audit score references. Run once for .6 rad/s seeds 7 19 41 and once for
.85 rad/s seeds 101 307 911. Set work/cache paths under HOME/scratch when writable.
The runner generates 4 m captures, reads existing 2 m captures, checks baseline
hashes and refuses old output/work paths. report_altitude.py regenerates this
text, CSV and interactive HTML. Existing native-delay/ESKF reports remain
historical and are not rewritten. Full CI, the full delay/rate matrix, native
retuning, real flight disturbance coverage and target efficiency remain open.
"""
    paths[2].write_text(text)
    html = r"""<!doctype html><html lang="en"><meta charset="utf-8"><title>Altitude and estimator comparison</title>
<style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:0 1rem;color:#192734}select{font:inherit;margin:.5rem;padding:.3rem}svg{width:100%;height:auto}table{border-collapse:collapse;width:100%}th,td{text-align:left;padding:.6rem;border-bottom:1px solid #ddd}.note{background:#fff3ce;padding:1rem}small{color:#4c5964}</style>
<h1>Altitude and magnetic activation</h1><p>2 m and 4 m climbs · original and held-out motion · three seeds · nominal delays</p>
<p class="note">Configured stacks with unequal native tuning, sensor rates and averaging/time semantics. This is not an intrinsic algorithm ranking. The takeoff expectation clears at 18 s for both heights; EKF3 MAG_CAL=3 remains unchanged.</p>
__COVARIANCE__
<label>Motion <select id="dataset"><option>original</option><option>held-out</option></select></label>
<label>Scenario <select id="scenario"><option>gps</option><option>denied</option><option>transition</option></select></label>
<label>Window <select id="window"><option value="flight">Flight 13–60 s</option><option value="outage_window">Outage 25–40 s</option><option value="after_return">After return 40–60 s</option></select></label>
<label>Metric <select id="metric">__OPTIONS__</select></label>
<p><span style="color:#2778b8">■ 2 m climb</span> · <span style="color:#b15418">■ 4 m climb</span> · bars: three-seed median · whiskers: min–max</p>
<svg id="plot" viewBox="0 0 1000 360" role="img" aria-label="Estimator error by climb height"></svg>
<table><thead><tr><th>Estimator</th><th>2 m: median [min, max]</th><th>4 m: median [min, max]</th></tr></thead><tbody id="values"></tbody></table>
<p><small>Altitude also changes vertical acceleration and optical-flow geometry. EKF3 magnetic selection switches naturally at 4 m; the low-altitude case remains heading-only. Native selection flags do not count all accepted updates.</small></p>
<p><a href="__TXT__">Detailed review</a> · <a href="__CSV__">All seed/window scores</a> · <a href="native-configuration-audit.txt">Fidelity audit</a></p>
<script>
const data=__DATA__, names=__NAMES__;
const ids=['dataset','scenario','window','metric'], fields=Object.fromEntries(ids.map(id=>[id,document.getElementById(id)]));
function selection(){return data.filter(row=>row.dataset===fields.dataset.value&&row.scenario===fields.scenario.value&&row.window===fields.window.value)}
function stats(rows,name,height,key){const x=rows.filter(row=>row.estimator===name&&row.climb_height_m===height).map(row=>row[key]).sort((a,b)=>a-b);if(x.length!==3)throw Error('Incomplete seed group');return {mid:x[1],low:x[0],high:x[2]}}
function draw(){const rows=selection(),key=fields.metric.value, groups=Object.keys(names).map(name=>[name,[2,4].map(height=>stats(rows,name,height,key))]);
const top=Math.max(...groups.flatMap(g=>g[1].map(s=>s.high)))*1.12||1, y=v=>300-250*v/top, fmt=v=>v.toPrecision(4);
let svg='',table='';for(let i=0;i<=5;i++){const value=top*i/5;svg+=`<line x1="75" y1="${y(value)}" x2="980" y2="${y(value)}" stroke="#ddd"/><text x="65" y="${y(value)+5}" text-anchor="end" font-size="13">${fmt(value)}</text>`}
groups.forEach(([name,pair],index)=>{const center=185+index*225;svg+=`<text x="${center}" y="330" text-anchor="middle" font-size="15">${names[name]}</text>`;
pair.forEach((s,j)=>{const x=center-63+j*66,cx=x+28;svg+=`<rect x="${x}" y="${y(s.mid)}" width="56" height="${300-y(s.mid)}" fill="${j?'#b15418':'#2778b8'}"/><path d="M${cx},${y(s.low)}V${y(s.high)}M${cx-7},${y(s.low)}h14M${cx-7},${y(s.high)}h14" stroke="#172d3e" fill="none"/>`});
table+=`<tr><td>${names[name]}</td>${pair.map(s=>`<td>${fmt(s.mid)} [${fmt(s.low)}, ${fmt(s.high)}]</td>`).join('')}</tr>`});document.getElementById('plot').innerHTML=svg;document.getElementById('values').innerHTML=table}
ids.forEach(id=>fields[id].onchange=()=>{if(id==='scenario')fields.window.value=fields.scenario.value==='transition'?'outage_window':'flight';draw()});draw();
</script></html>"""
    options = "".join(
        f'<option value="{key}">{label}</option>' for key, label in METRICS.items()
    )
    for token, value in {
        "__OPTIONS__": options,
        "__DATA__": json.dumps(points, allow_nan=False),
        "__NAMES__": json.dumps(NAMES),
        "__TXT__": paths[2].name,
        "__CSV__": paths[1].name,
        "__COVARIANCE__": (
            "<p>Retrodiction covariance remains overconfident in original seed 41 "
            "(mean 15D NEES 141.61). Positive covariance alone does not establish "
            "consistency. See the detailed review.</p>"
            if getattr(args, "covariance", None)
            else ""
        ),
    }.items():
        html = html.replace(token, value)
    paths[0].write_text(html)


if __name__ == "__main__":
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("original", "held-out", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--covariance", type=Path)
    report(parser.parse_args())
