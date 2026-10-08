#!/usr/bin/env python3
"""Render delayed-fusion scores as a standalone interactive report and CSV."""

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
    "yaw_rmse_deg": "Yaw RMSE (deg)",
    "cpu_ms": "Generated component CPU time per 60 s replay (ms)",
}
COMPONENTS = ("filter", "preintegration", "predictor", "queues")


def render(args):
    flat, groups = [], {}
    for dataset, path in (("original", args.original), ("held-out", args.held_out)):
        evidence = json.loads(path.read_text())
        if evidence.get("valid_for_comparison") is False:
            raise ValueError("Refusing invalidated adapter results")
        for score in evidence["scores"]:
            cpu_ms = median(
                sum(timing[part]["total_ns"] for part in COMPONENTS) / 1e6
                for timing in score["timing"]
            )
            for window in ("flight", "before_outage", "outage_window", "after_return"):
                record = dict(
                    dataset=dataset,
                    profile=score["profile"],
                    frequency=score["frequency"],
                    seed=score["seed"],
                    scenario=score["scenario"],
                    delay_profile=score["delay_profile"],
                    name=score["name"],
                    window=window,
                    cpu_ms=cpu_ms,
                    instance_bytes=score["timing"][0]["state_bytes"],
                    **score[window],
                )
                flat.append(record)
                key = tuple(
                    record[field]
                    for field in (
                        "dataset",
                        "profile",
                        "frequency",
                        "scenario",
                        "delay_profile",
                        "name",
                        "window",
                    )
                )
                groups.setdefault(key, []).append(record)
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="") as output:
        writer = csv.DictWriter(
            output, fieldnames=sorted(set().union(*(row.keys() for row in flat)))
        )
        writer.writeheader()
        writer.writerows(flat)
    table, chart = [], []
    for key, records in sorted(groups.items()):
        row = dict(
            zip(
                (
                    "dataset",
                    "profile",
                    "frequency",
                    "scenario",
                    "delay_profile",
                    "name",
                    "window",
                ),
                key,
            )
        )
        values = {}
        for metric in METRICS:
            samples = [record[metric] for record in records]
            values[metric] = [median(samples), min(samples), max(samples)]
        row.update(values)
        chart.append(row)
        attributes = " ".join(
            f'data-{field.replace("_", "-")}="{html.escape(str(row[field]))}"'
            for field in (
                "dataset",
                "profile",
                "frequency",
                "scenario",
                "delay_profile",
                "name",
                "window",
            )
        )
        cells = [
            row[field]
            for field in (
                "dataset",
                "profile",
                "frequency",
                "scenario",
                "delay_profile",
                "name",
                "window",
            )
        ]
        cells += [
            f"{values[metric][0]:.4f} [{values[metric][1]:.4f}, {values[metric][2]:.4f}]"
            for metric in METRICS
        ]
        cells += [f"{records[0]['instance_bytes'] / 1024:.2f}"]
        table.append(
            f"<tr {attributes}>"
            + "".join(f"<td>{html.escape(str(cell))}</td>" for cell in cells)
            + "</tr>"
        )
    controls = []
    for field in ("dataset", "profile", "frequency", "scenario", "window"):
        choices = sorted({row[field] for row in chart})
        default = {
            "dataset": "original",
            "profile": "100/50/50 Hz",
            "scenario": "denied",
            "window": "flight",
            "frequency": choices[0],
        }[field]
        options = "".join(
            f"<option{(' selected' if choice == default else '')}>{html.escape(str(choice))}</option>"
            for choice in choices
        )
        controls.append(
            f'<label>{field.replace("_", " ")} <select data-filter="{field}">{options}</select></label>'
        )
    metric_options = "".join(
        f'<option value="{key}">{value}</option>' for key, value in METRICS.items()
    )
    headers = [
        "Dataset",
        "Flow/mag/baro rate",
        "Motion (rad/s)",
        "Scenario",
        "Delay",
        "Estimator",
        "Window",
        *METRICS.values(),
        "Instance including scratch (KiB)",
    ]
    page = """<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESKF fusion horizon versus retrodiction</title><style>
body{font:16px system-ui;line-height:1.5;margin:2rem;color:#172b4d;background:#f6f8fb}main{max-width:1500px;margin:auto}
label{display:inline-block;margin:0 .8rem .8rem 0}select{padding:.4rem}svg{width:100%;max-width:960px;background:white;border:1px solid #ddd}
.scroll{overflow:auto}table{border-collapse:collapse;background:white;font-size:13px}th,td{padding:.6rem;border:1px solid #ddd;white-space:nowrap}th{background:#e5edf8}
</style><main><h1>ESKF fusion horizon versus retrodiction</h1>
<p>Identical sensor packets, an 800 Hz IMU and a 100 Hz filter. Each value is the median [minimum, maximum] of three seed-specific results.
GPS loss occurs at 25 s and GPS returns at 40 s. Flight scoring uses 13–60 s. Both geometric experimental options are disabled.</p>
<p>The horizon filters approximately 200 ms behind now, then reconstructs the current state with buffered SE₂(3) preintegrals.
The accuracy chart scores that current prediction. Filter-state errors use the separate fusion timestamp in the raw JSON evidence.</p>
<p>Delay profiles list GPS/flow/magnetometer/barometer: zero 0/0/0/0 ms; nominal 110/30/20/20 ms plus 0–10 ms seeded jitter;
stress 160/60/50/70 ms plus 0–20 ms jitter. Arrivals are released on the common 100 Hz sensor schedule, in source order.</p>
<p>CPU is generated numerical component thread time for the entire 60 s replay, including timer overhead and excluding CSV, transport simulation and adapter copies.
These host results are not flight-target WCET. Instance bytes include generated scratch; the common transport simulator is excluded.
The composed Modelica export remains blocked by Rumoca 0.10.2; this comparison explicitly schedules generated components.
Queue work runs on aiding arrival or fusion release. This differs from the composed model's unconditional 800 Hz queue calls.</p>
<p><a href="eskf-delay.txt">Method, findings and limitations</a> · <a href="@CSV@">Per-seed CSV</a> ·
<a href="eskf-delay-original.json">Original scores</a> · <a href="eskf-delay-held-out.json">Held-out scores</a> ·
<a href="eskf-delay-manifest.json">Source and validation evidence</a></p>
<div>@CONTROLS@<label>Chart metric <select id="metric">@METRICS@</select></label><label><input id="log" type="checkbox" checked> Log scale</label></div>
<svg id="plot" viewBox="0 0 960 370" role="img" aria-label="Error and CPU comparison by delay profile"></svg>
<p>Table cells show median [minimum, maximum]; CPU always covers the full replay. The retrodiction path is still the vehicle default.</p>
<div class="scroll"><table><thead><tr>@HEADERS@</tr></thead><tbody>@TABLE@</tbody></table></div></main>
<script>const data=@DATA@;const filters=[...document.querySelectorAll('[data-filter]')];
const svg=document.querySelector('#plot'),metric=document.querySelector('#metric'),log=document.querySelector('#log');
function draw(){const dataset=filters.find(control=>control.dataset.filter==='dataset').value;
const frequency=filters.find(control=>control.dataset.filter==='frequency');
const available=[...new Set(data.filter(row=>row.dataset===dataset).map(row=>String(row.frequency)))];
if(!available.includes(frequency.value))frequency.value=available[0];
for(const option of frequency.options)option.disabled=!available.includes(option.value);
const selected=data.filter(row=>filters.every(control=>String(row[control.dataset.filter])===control.value));
for(const row of document.querySelectorAll('tbody tr'))row.hidden=filters.some(control=>row.dataset[control.dataset.filter]!==control.value);
const key=metric.value,series=['retrodiction','horizon'],delays=['zero','nominal','stress'],colors=['#2563eb','#b45309'];
const samples=selected.flatMap(row=>row[key]), transform=value=>log.checked?Math.log10(Math.max(value,1e-9)):value;
let low=Math.min(...samples.map(transform)),high=Math.max(...samples.map(transform));if(high===low)high=low+1;
const pad=(high-low)*.1;low-=pad;high+=pad;const y=value=>300-(transform(value)-low)/(high-low)*250,x=index=>130+index*340;
let content='<text x="80" y="23" fill="#172b4d">'+metric.selectedOptions[0].textContent+'</text>';
for(let tick=0;tick<=4;tick++){const value=low+(high-low)*tick/4,position=300-tick*250/4;
content+='<line x1="80" y1="'+position+'" x2="890" y2="'+position+'" stroke="#ddd"/><text x="6" y="'+(position+5)+'">'+(log.checked?10**value:value).toPrecision(3)+'</text>';}
delays.forEach((delay,index)=>content+='<text x="'+(x(index)-24)+'" y="330">'+delay+'</text>');
series.forEach((name,index)=>{const points=delays.map((delay,position)=>{const row=selected.find(row=>row.name===name&&row.delay_profile===delay);if(!row)return'';
const values=row[key],xx=x(position)+index*5;content+='<line x1="'+xx+'" y1="'+y(values[1])+'" x2="'+xx+'" y2="'+y(values[2])+'" stroke="'+colors[index]+'" stroke-width="3"/>';
content+='<circle cx="'+xx+'" cy="'+y(values[0])+'" r="5" fill="'+colors[index]+'"/>';return xx+','+y(values[0]);}).filter(Boolean);
content+='<polyline points="'+points.join(' ')+'" fill="none" stroke="'+colors[index]+'" stroke-width="2"/><text x="'+(150+index*280)+'" y="360" fill="'+colors[index]+'">'+name+'</text>';});svg.innerHTML=content;}
[...filters,metric,log].forEach(control=>control.addEventListener('change',draw));draw();</script></html>"""
    replacements = {
        "@CSV@": html.escape(csv_path.name),
        "@CONTROLS@": "".join(controls),
        "@METRICS@": metric_options,
        "@HEADERS@": "".join(f"<th>{html.escape(header)}</th>" for header in headers),
        "@TABLE@": "".join(table),
        "@DATA@": json.dumps(chart, allow_nan=False).replace("</", "<\\/"),
    }
    for marker, replacement in replacements.items():
        page = page.replace(marker, replacement)
    args.output.write_text(page)
    print(f"Wrote {len(flat)} per-seed metric rows and {len(chart)} aggregate rows.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original", type=Path, required=True)
    parser.add_argument("--held-out", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    render(parser.parse_args())
