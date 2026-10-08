#!/usr/bin/env python3
"""Compare ESKF modes and native EKF2/EKF3 on shared delayed sensor captures."""

import argparse
import csv
import html
import json
from pathlib import Path
from statistics import median


METRICS = {
    "horizontal_position_rmse_m": "Horizontal position RMSE (m)",
    "vertical_position_rmse_m": "Vertical position RMSE (m)",
    "velocity_3d_rmse_m_s": "Velocity RMSE (m/s)",
    "attitude_rmse_deg": "Attitude RMSE (deg)",
    "yaw_rmse_deg": "Yaw RMSE (deg)",
    "position_valid_fraction": "Position valid fraction",
    "attitude_valid_fraction": "Attitude valid fraction",
}
NAMES = {
    "retrodiction": "ESKF retrodiction",
    "horizon": "ESKF buffered horizon",
    "px4": "Native PX4 EKF2",
    "ekf3": "Native ArduPilot EKF3",
}
FILTERS = ("dataset", "profile", "scenario", "delay_profile", "window")


def render(args):
    groups, flat = {}, []
    stationary_intervals = set()
    imu_noise_profiles = set()
    for dataset in ("original", "held-out"):
        transports = {}
        for prefix in (args.eskf_prefix, "native-delay"):
            path = args.directory / f"{prefix}-{dataset}.json"
            evidence = json.loads(path.read_text())
            if evidence.get("valid_for_comparison") is False:
                raise ValueError(
                    "Invalidated evidence cannot enter the native comparison"
                )
            if prefix == args.eskf_prefix:
                stationary_intervals.add(evidence.get("stationary_until_s", 0))
                imu_noise_profiles.add(tuple(evidence.get("imu_noise_density", ())))
            for score in evidence["scores"]:
                case = (score["tag"], score["scenario"], score["delay_profile"])
                trace = score.get("transport", score.get("timing", [{}])[0])
                packets = (trace["transport_delivered"], trace["transport_digest"])
                if case in transports and packets != transports[case]:
                    raise ValueError("Compared estimators received different packets")
                transports[case] = packets
                for window in (
                    "flight",
                    "before_outage",
                    "outage_window",
                    "after_return",
                ):
                    row = dict(
                        dataset=dataset,
                        profile=score["profile"],
                        frequency=score["frequency"],
                        seed=score["seed"],
                        scenario=score["scenario"],
                        delay_profile=score["delay_profile"],
                        name=score["name"],
                        window=window,
                        **score[window],
                    )
                    flat.append(row)
                    key = tuple(row[field] for field in (*FILTERS, "frequency", "name"))
                    groups.setdefault(key, []).append(row)
    if len(stationary_intervals) != 1:
        raise ValueError("ESKF datasets use inconsistent stationary intervals")
    stationary_until = stationary_intervals.pop()
    if len(imu_noise_profiles) != 1:
        raise ValueError("ESKF datasets use inconsistent IMU noise densities")
    imu_noise_density = imu_noise_profiles.pop()
    if len(flat) != 1728:
        raise ValueError(
            "The full original/held-out four-estimator matrix is incomplete"
        )
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="") as output:
        writer = csv.DictWriter(
            output, fieldnames=sorted(set().union(*(row.keys() for row in flat)))
        )
        writer.writeheader()
        writer.writerows(flat)
    aggregates, table = [], []
    for key, rows in sorted(groups.items()):
        if len(rows) != 3 or len({row["seed"] for row in rows}) != 3:
            raise ValueError(
                "An aggregate lacks its three independent seed-specific runs"
            )
        row = dict(zip((*FILTERS, "frequency", "name"), key))
        for metric in METRICS:
            samples = [record[metric] for record in rows]
            row[metric] = [median(samples), min(samples), max(samples)]
        aggregates.append(row)
        attributes = " ".join(
            f'data-{field.replace("_", "-")}="{html.escape(str(row[field]))}"'
            for field in FILTERS
        )
        cells = [row[field] for field in FILTERS] + [
            row["frequency"],
            NAMES[row["name"]],
        ]
        cells += [
            f"{row[metric][0]:.4f} [{row[metric][1]:.4f}, {row[metric][2]:.4f}]"
            for metric in METRICS
        ]
        table.append(
            f"<tr {attributes}>"
            + "".join(f"<td>{html.escape(str(cell))}</td>" for cell in cells)
            + "</tr>"
        )
    controls = []
    for field in FILTERS:
        choices = sorted({row[field] for row in aggregates})
        default = {
            "dataset": "original",
            "profile": "100/50/50 Hz",
            "scenario": "denied",
            "delay_profile": "nominal",
            "window": "flight",
        }[field]
        options = "".join(
            f"<option{' selected' if choice == default else ''}>{html.escape(choice)}</option>"
            for choice in choices
        )
        controls.append(
            f'<label>{field.replace("_", " ")} <select data-filter="{field}">{options}</select></label>'
        )
    headers = [*FILTERS, "motion (rad/s)", "estimator", *METRICS.values()]
    page = """<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESKF, native PX4 EKF2 and ArduPilot EKF3: delayed sensor comparison</title><style>
body{font:16px system-ui;line-height:1.5;margin:2rem;color:#172b4d;background:#f6f8fb}main{max-width:1600px;margin:auto}
label{display:inline-block;margin:0 .7rem .8rem 0}select{padding:.4rem}svg{width:100%;max-width:1000px;background:white;border:1px solid #ddd}
.scroll{overflow:auto}table{border-collapse:collapse;background:white;font-size:13px}th,td{padding:.6rem;border:1px solid #ddd;white-space:nowrap}th{background:#e5edf8}
</style><main><h1>ESKF, native PX4 EKF2 and ArduPilot EKF3</h1>
<p>Shared noisy sensor captures, packet arrival schedules and 100 Hz scoring epochs. Each value is the median [minimum, maximum] of three seed-specific results.
Original motion is .6 rad/s; held-out motion is .85 rad/s. GPS is lost at 25 s and returns at 40 s. Flight scoring uses 13–60 s.</p>
<p>Delays in GPS/flow/magnetometer/barometer order: zero 0/0/0/0 ms; nominal 110/30/20/20 ms plus 0–10 ms positive jitter;
stress 160/60/50/70 ms plus 0–20 ms jitter. Every native arrival count and payload digest matches its ESKF reference.</p>
<p>Native filter cores are unchanged. PX4 receives original measurement timestamps; EKF3 uses its driver-lag and fixed-delay rules, including its fixed 60 ms magnetic delay.
@REST@ Priors, tuning, source selection and output observers remain stack-specific.
This measures configured estimator stacks and does not establish a universal Kalman-filter ranking.</p>
<p>Native state outputs are linearly interpolated to the common scoring epochs; quaternions use sign-continuous normalized interpolation.
Flags hold the preceding native value. Missing intervals and extrapolation are refused. Invalid finite states stay in the error metrics.
Native source-active flags do not prove individual measurement acceptance.</p>
<p><a href="@REVIEW@.txt">Method, findings and remaining gaps</a> · <a href="@REVIEW@-manifest.json">Provenance and validation</a> ·
<a href="@CSV@">Per-seed CSV</a> · <a href="native-delay-original.json">Original native results</a> ·
<a href="native-delay-held-out.json">Held-out native results</a> · <a href="eskf-delay.html">ESKF timing and memory comparison</a></p>
<div>@CONTROLS@<label>Metric <select id="metric">@METRICS@</select></label><label><input id="log" type="checkbox" checked> Log scale</label></div>
<svg id="plot" viewBox="0 0 1000 380" role="img" aria-label="Four estimator comparison with seed ranges"></svg>
<p>Vertical bars show seed ranges. The ESKF vehicle default remains retrodiction; composed horizon export and flight timing qualification are outstanding.</p>
<div class="scroll"><table><thead><tr>@HEADERS@</tr></thead><tbody>@TABLE@</tbody></table></div></main>
<script>const data=@DATA@,names=@NAMES@;const filters=[...document.querySelectorAll('[data-filter]')],svg=document.querySelector('#plot'),metric=document.querySelector('#metric'),log=document.querySelector('#log');
function draw(){const selected=data.filter(row=>filters.every(control=>String(row[control.dataset.filter])===control.value));
for(const row of document.querySelectorAll('tbody tr'))row.hidden=filters.some(control=>row.dataset[control.dataset.filter]!==control.value);
const key=metric.value,series=Object.keys(names),colors=['#64748b','#b45309','#2563eb','#15803d'];
const transform=value=>log.checked?Math.log10(Math.max(value,1e-9)):value,samples=selected.flatMap(row=>row[key]);
let low=Math.min(...samples.map(transform)),high=Math.max(...samples.map(transform));if(low===high)high=low+1;const padding=(high-low)*.1;low-=padding;high+=padding;
const y=value=>300-(transform(value)-low)/(high-low)*250,x=index=>175+index*235;
let content='<text x="80" y="24">'+metric.selectedOptions[0].textContent+'</text>';
for(let tick=0;tick<=4;tick++){const value=low+(high-low)*tick/4,position=300-tick*250/4;
content+='<line x1="80" y1="'+position+'" x2="960" y2="'+position+'" stroke="#ddd"/><text x="6" y="'+(position+5)+'">'+(log.checked?10**value:value).toPrecision(3)+'</text>';}
series.forEach((name,index)=>{const row=selected.find(row=>row.name===name);if(!row)return;const values=row[key],xx=x(index);
content+='<line x1="'+xx+'" x2="'+xx+'" y1="'+y(values[1])+'" y2="'+y(values[2])+'" stroke="'+colors[index]+'" stroke-width="4"/>';
content+='<circle cx="'+xx+'" cy="'+y(values[0])+'" r="7" fill="'+colors[index]+'"/><text x="'+(xx-40)+'" y="'+(y(values[0])-12)+'">'+values[0].toPrecision(4)+'</text>';
content+='<text x="'+(xx-85)+'" y="335" fill="'+colors[index]+'">'+names[name]+'</text>';});svg.innerHTML=content;}
[...filters,metric,log].forEach(control=>control.addEventListener('change',draw));draw();</script></html>"""
    replacements = {
        "@CSV@": html.escape(csv_path.name),
        "@REVIEW@": html.escape(args.review_prefix or args.output.stem),
        "@REST@": (
            f"Native adapters receive the declared stationary/armed phase ending at 13 s; ESKF receives an explicit at-rest velocity constraint until {stationary_until:g} s."
            if stationary_until
            else "Both native adapters receive the declared stationary/armed phase; ESKF has no equivalent explicit flag."
        ),
        "@CONTROLS@": "".join(controls),
        "@METRICS@": "".join(
            f'<option value="{key}">{label}</option>' for key, label in METRICS.items()
        ),
        "@HEADERS@": "".join(
            f"<th>{html.escape(label.replace('_', ' '))}</th>" for label in headers
        ),
        "@TABLE@": "".join(table),
        "@DATA@": json.dumps(aggregates, allow_nan=False),
        "@NAMES@": json.dumps(NAMES),
    }
    if imu_noise_density:
        gyro, accel = imu_noise_density
        replacements["@REST@"] += (
            f" ESKF white IMU noise densities are {gyro:.6g} rad/sqrt(s) and "
            f"{accel:.6g} m/(s sqrt(s)); native noise tuning is unchanged from the "
            "earlier native comparison. These are different configured stacks, "
            "not identically tuned filter algorithms."
        )
    for marker, value in replacements.items():
        page = page.replace(marker, value)
    args.output.write_text(page)
    print(f"Wrote {len(flat)} per-seed rows and {len(aggregates)} aggregate rows")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--eskf-prefix", default="eskf-delay")
    parser.add_argument("--review-prefix")
    render(parser.parse_args())
