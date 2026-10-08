"""Summarize frozen native comparisons, new flight controls and formal contracts."""

import argparse
import csv
import itertools
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


KEYS = ("seed", "frequency", "climb_height_m", "scenario", "explicit_exposure")
METRICS = (
    "horizontal_position_rmse_m",
    "vertical_position_rmse_m",
    "velocity_3d_rmse_m_s",
    "attitude_rmse_deg",
    "yaw_rmse_deg",
)
LABELS = {
    "horizon-dense": "ESKF horizon",
    "retrodiction-dense": "ESKF retrodiction",
    "horizon-root": "ESKF horizon root",
    "retrodiction-root": "ESKF retrodiction root",
    "px4": "PX4 EKF2",
    "ekf3": "ArduPilot EKF3",
}


def key(row):
    return tuple(row[name] for name in KEYS)


def cpu(timing):
    return sum(
        timing[name]["total_ns"]
        for name in ("filter", "preintegration", "predictor", "queues")
    )


def report(args):
    native, flights, proofs = [
        json.loads(path.read_text())
        for path in (args.native, args.flights, args.proofs)
    ]
    conditions = set(
        (seed, frequency, height, scenario, exposure)
        for (seed, frequency), height, scenario, exposure in itertools.product(
            ((7, 0.6), (101, 0.85)),
            (2, 4),
            ("gps", "denied", "transition"),
            (False, True),
        )
    )
    groups = {}
    for name in LABELS:
        source = (
            flights["scores"]
            if name.startswith(("horizon", "retro"))
            else native["scores"]
        )
        rows = [
            row
            for row in source
            if (
                row["name"] + "-" + row["representation"]
                if "representation" in row
                else row["name"]
            )
            == name
        ]
        groups[name] = {key(row): row for row in rows}
        if len(rows) != 24 or set(groups[name]) != conditions:
            raise ValueError(f"Incomplete or duplicated comparison: {name}")
    if len(flights["scores"]) != 96:
        raise ValueError("Expected every one of the 96 ESKF flight results")
    if any(
        row[window]["finite_fraction"] != 1
        or row[window]["row_coverage"] < 0.999
        or row[window]["position_valid_fraction"] != 1
        or row[window]["attitude_valid_fraction"] != 1
        for group in groups.values()
        for row in group.values()
        for window in ("flight", "outage_window", "after_return")
    ):
        raise ValueError("Invalid comparison coverage; retain and investigate failures")
    if set(proofs["audit"]["axioms"]) != {"Classical.choice", "Quot.sound", "propext"}:
        raise ValueError("Unexpected formal proof assumptions")

    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "estimator-theory-comparison.csv").open("w") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=["estimator", *KEYS, "window", *METRICS, "output_sha256"],
        )
        writer.writeheader()
        for name, group in groups.items():
            for identifier, row in sorted(group.items()):
                for window in ("flight", "outage_window", "after_return"):
                    writer.writerow(
                        dict(
                            estimator=name,
                            **dict(zip(KEYS, identifier)),
                            window=window,
                            **{metric: row[window][metric] for metric in METRICS},
                            output_sha256=row["output_sha256"],
                        )
                    )

    lines = [
        "ESKF / native EKF2 / native EKF3: measured and formal comparison, 7 October 2026",
        "",
        "This is a two-seed synthetic measurement-contract study. It does not establish",
        "universal superiority. ESKF has lower horizontal and velocity errors in the",
        "explicit-exposure cases; ArduPilot still has lower vertical error in every case.",
        "The new joint-covariance guard is a robustness change, not an accuracy gain.",
        "Raw fallback prediction also restores its previously unassigned transition.",
        "",
        "Evidence and versions",
        f"Native flight source pins: {json.dumps(native['native_source_pins'], sort_keys=True)}",
        "Native flight results are frozen, actual native cores, not Modelica ports.",
        "Separately refreshed port kernels target PX4 v1.17.0 and Copter-4.7.1.",
        "Those release-kernel tests do not upgrade or rerun these native flight cores.",
        "See release-port-refresh.txt for release source hashes and precise port scope.",
        "ESKF uses fresh Rumoca 0.10.2 generated C, FOH and bias Jacobians, geometric",
        "alignment and vector magnetic fusion; stationary IMU aiding is disabled.",
        "Fusion horizon is 200 ms, followed by the output predictor; retrodiction is",
        "also scored on the same inputs. Root covariance is an optional representation.",
        "",
        "Common data and remaining mismatches",
        "Seeds 7/101 also select motion frequencies 0.6/0.85; motion and seed are coupled.",
        "Each has 2/4 m climbs and GPS, denied, and GPS loss 25–40 s then return.",
        "Every method uses the same frozen physical capture and delivered packet trace.",
        "100 ms optical-flow exposure at 10 Hz, midpoint timestamp, available at its end.",
        "Independent unbiased camera gyro; no noise construction followed by cancellation.",
        "GPS/flow/mag/barometer transport delays are 110/30/60/20 ms; no jitter.",
        "Stack-specific priors, effective R/Q, magnetic/height policies and sensor",
        "acceptance remain unequal. EKF3 retains its range delay and median filtering.",
        "ESKF uses a calibrated fixed field; native filters estimate magnetic states.",
        "No image texture, rolling shutter, magnetic faults, vibration or real flights.",
        "The legacy flow mapping is retained in the CSV, including catastrophic drift.",
        "",
        "Published-state RMSE 13–60 s, medians of two seeds, explicit exposure only",
        "height scenario   estimator                  horizontal m vertical m velocity m/s attitude deg yaw deg",
    ]
    for height, scenario in itertools.product((2, 4), ("gps", "denied", "transition")):
        for name, group in groups.items():
            rows = [r for k, r in group.items() if k[2:] == (height, scenario, True)]
            values = [np.median([r["flight"][m] for r in rows]) for m in METRICS]
            lines.append(
                f"{height:4} m {scenario:10} {LABELS[name]:26}"
                + " ".join(f"{v:12.5f}" for v in values)
            )
    lines.extend(
        [
            "",
            "Native configured-stack comparison, explicit-exposure flight RMS",
            "Counts where PX4 has lower RMS than ArduPilot; four pairs per scenario.",
            "Different policies and effective noise prevent an intrinsic algorithm ranking.",
            "scenario    horizontal vertical velocity attitude yaw",
        ]
    )
    for scenario in ("gps", "denied", "transition"):
        identifiers = [k for k in groups["px4"] if k[3:] == (scenario, True)]
        wins = [
            sum(
                groups["px4"][k]["flight"][metric] < groups["ekf3"][k]["flight"][metric]
                for k in identifiers
            )
            for metric in METRICS
        ]
        lines.append(f"{scenario:11}" + " ".join(f"{value:2}/4     " for value in wins))
    lines.extend(
        [
            "",
            "Paired cases with lower RMSE than BOTH native cores, explicit exposure",
            "Twelve cases per method; no statistical confidence interval is implied.",
            "estimator                    window         horizontal vertical velocity attitude yaw",
        ]
    )
    comparisons = {}
    for name in list(LABELS)[:4]:
        comparisons[name] = {}
        for window in ("flight", "outage_window", "after_return"):
            identifiers = [
                k
                for k in groups[name]
                if k[-1] and (window == "flight" or k[3] == "transition")
            ]
            wins = {
                metric: sum(
                    groups[name][k][window][metric]
                    < min(
                        groups[native_name][k][window][metric]
                        for native_name in ("px4", "ekf3")
                    )
                    for k in identifiers
                )
                for metric in METRICS
            }
            comparisons[name][window] = dict(cases=len(identifiers), wins=wins)
            lines.append(
                f"{LABELS[name]:28} {window:14}"
                + " ".join(f"{wins[m]:2}/{len(identifiers):2}   " for m in METRICS)
            )
    lines.extend(
        [
            "",
            "GPS loss-and-return windows, median over two seeds and two heights",
            "window         estimator                  horizontal m vertical m velocity m/s attitude deg yaw deg",
        ]
    )
    for window in ("outage_window", "after_return"):
        for name, group in groups.items():
            rows = [r for k, r in group.items() if k[3:] == ("transition", True)]
            values = [np.median([r[window][m] for r in rows]) for m in METRICS]
            lines.append(
                f"{window:14} {LABELS[name]:26}"
                + " ".join(f"{v:12.5f}" for v in values)
            )
    lines.extend(
        [
            "",
            "ESKF GPS-return indicators, four explicit-exposure cases per method",
            "First accepted-GPS output indicator latency is measured from 40 s;",
            "it is not a native per-sensor fusion-epoch diagnostic.",
            "estimator                    median GPS indicator s  median max error jump m",
        ]
    )
    for name in list(LABELS)[:4]:
        rows = [r for k, r in groups[name].items() if k[3:] == ("transition", True)]
        indicators = [r["transition"]["first_gps_use_after_return_s"] for r in rows]
        jumps = [r["transition"]["max_return_error_jump_m"] for r in rows]
        if any(value is None for value in indicators):
            raise ValueError(
                "Missing ESKF GPS return; retain and investigate the failure"
            )
        lines.append(
            f"{LABELS[name]:28} {np.median(indicators):22.5f} {np.median(jumps):24.5f}"
        )
    lines.extend(
        [
            "The 0.25 m / one-second recovery criterion is weak for these small-error",
            "runs. Do not interpret an immediate threshold pass as demonstrated fast",
            "reacquisition from a large outage error. Comparable native latency is not",
            "available in the frozen score summaries.",
            "",
            "Consistency and numerical validity",
            "15D NEES uses local right-error covariance at its own state epoch: fusion",
            "time for horizon, publication time for retrodiction. It is descriptive:",
            "samples are correlated, and two seeds do not establish chi-square coverage.",
            "A smaller NEES is not automatically better calibration; 15 is the modeled",
            "expectation only under the correct centered joint error distribution.",
            "Exact native full-covariance NEES and per-sensor NIS are unavailable here.",
            "Native gate-test ratios and quantized logs cannot replace those quantities.",
        ]
    )
    for name in list(LABELS)[:4]:
        rows = [r for k, r in groups[name].items() if k[-1]]
        valid = [
            r["consistency"]["flight"]
            for r in rows
            if r["consistency"]["flight"].get("valid", True)
        ]
        lines.append(
            f"{LABELS[name]}: valid {len(valid)}/12, median mean NEES {np.median([r['mean_nees_15d'] for r in valid]):.5f}."
        )
    lines.extend(
        [
            "",
            "Joint-covariance safety improvement",
            "Previously dense correction could accept positive innovation covariance",
            "while the joint state/noise covariance was indefinite, producing negative P.",
            "The guard factors P=L L^T and solves L U=C, then factors R-U^T U.",
            "It rejects impossible joint covariance and preserves the complete prior.",
            "A first guard using a global pivot threshold rejected valid mixed-unit GPS",
            "updates. That candidate was rejected; its full 96-case evidence is retained.",
            "Final adversarial tests include that scale disparity and valid cross noise.",
            "An explicit array copy in the QR root also fixes OpenModelica's constant",
            "evaluation discrepancy while retaining the fast loop structure. A scalar",
            "rewrite was rejected after a large flight CPU regression; its partial",
            "evidence is retained in eskf-theory-rejected-scalar-replays.json.",
            "The aggregate test also exposed an unassigned raw-IMU transition matrix.",
            "Its nominal prediction agreed with packet integration, but covariance",
            "differed by 0.040001. The transition is now computed before propagation.",
            "Tests.All passes with OpenModelica cflags=-O0; default -Os compilation",
            "was stopped during a long build. This is not a completed default-CI pass.",
            "The new -O2 deployment probe passes 640 raw-prediction kinematic cases.",
            "",
            "Final fresh flight controls versus saved outputs, all flow mappings",
            "estimator                    identical  median filter CPU ratio  total CPU ratio  state bytes",
        ]
    )
    controls = {}
    for name in list(LABELS)[:4]:
        rows = list(groups[name].values())
        values = dict(
            identical=sum(r["output_identical"] for r in rows),
            cases=len(rows),
            median_filter_cpu_ratio=float(
                np.median(
                    [
                        r["timing"]["filter"]["total_ns"]
                        / r["previous_timing"]["filter"]["total_ns"]
                        for r in rows
                    ]
                )
            ),
            median_total_cpu_ratio=float(
                np.median([cpu(r["timing"]) / cpu(r["previous_timing"]) for r in rows])
            ),
            state_bytes=sorted({r["timing"]["state_bytes"] for r in rows}),
        )
        controls[name] = values
        lines.append(
            f"{LABELS[name]:28} {values['identical']:2}/24        {values['median_filter_cpu_ratio']:12.3f}       {values['median_total_cpu_ratio']:12.3f}   {values['state_bytes']}"
        )
    lines.extend(
        [
            "Host thread CPU measurements include compile/run variability and are not",
            "paired hardware WCET or a native CPU comparison. Root representation's",
            "historical filter cost is about 3.4–3.6x dense; do not infer a free accuracy gain.",
            "",
            "Formal verification",
            f"Ten selected GNC modules checked from source; {proofs['audit']['project_declarations']} project declarations in the imported environment.",
            "Kernel axiom audit permits only propext, Classical.choice and Quot.sound.",
            "See estimator-theory-contracts.txt for theorem names, hypotheses and limits.",
            "Exact-real factor identities prove covariance PSD under a valid joint noise",
            "model. They do not prove IEEE arithmetic or Modelica-to-Lean refinement.",
            "Matched optimal Kalman gains have equal modeled linear covariance; ESKF",
            "cannot strictly beat every correctly modeled EKF2/EKF3 case by coordinates alone.",
            "Horizontal translation remains unobservable with flat-ground flow, range and",
            "a constant magnetic field. Lie-group geometry cannot manufacture GPS data.",
            "The separate 288-replay FOH/ZOH study (eskf-hold-stationary.txt) also does",
            "not establish uniform stochastic FOH superiority: with stationary IMU off,",
            "FOH wins horizontal RMS in 11/36 horizon and 17/36 retrodiction cases.",
            "Those historical captures differ from the explicit-exposure study here.",
            "",
            "Remaining work before any broad superiority claim",
            "1. Instrument native full covariance and innovations at actual fusion epochs;",
            "   rebuild whole native cores at the verified release pins, then rerun parity.",
            "2. Match observable priors, transformed R/Q, sensor epochs and supported",
            "   source policies. Keep default-stack and calibrated comparisons separate.",
            "3. Improve vertical/pressure-datum estimation with modeled cross covariance,",
            "   qualified against withheld datum drift and independent range observations.",
            "4. Derive FOH process noise with shared-endpoint correlations; the retained",
            "   Magnus mean and bias Jacobians still use approximate Simpson noise.",
            "5. Use independently varied motion/noise seeds, longer outages and realistic",
            "   disturbances; count invalid runs, recovery latency and target CPU/memory.",
        ]
    )
    (args.output / "estimator-theory-comparison.txt").write_text(
        "\n".join(lines) + "\n"
    )
    (args.output / "estimator-theory-summary.json").write_text(
        json.dumps(dict(comparisons=comparisons, controls=controls), indent=2) + "\n"
    )

    colors = ("#0072B2", "#56B4E9", "#D55E00", "#009E73")
    names = ("horizon-dense", "retrodiction-dense", "px4", "ekf3")
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), constrained_layout=True)
    for column, scenario in enumerate(("gps", "denied", "transition")):
        for row, metric in enumerate(METRICS[:2]):
            ax = axes[row, column]
            for index, (name, color) in enumerate(zip(names, colors)):
                values = [
                    r["flight"][metric]
                    for k, r in groups[name].items()
                    if k[3:] == (scenario, True)
                ]
                median = np.median(values)
                ax.bar(index, median, color=color, width=0.7)
                ax.errorbar(
                    index,
                    median,
                    yerr=[[median - min(values)], [max(values) - median]],
                    color="#333333",
                    capsize=4,
                )
            ax.set_xticks(range(4), ("Horizon", "Retro", "EKF2", "EKF3"))
            ax.set_ylabel(("Horizontal" if row == 0 else "Vertical") + " RMSE (m)")
            ax.grid(axis="y", alpha=0.25)
            ax.set_axisbelow(True)
            if row == 0:
                ax.set_title(
                    dict(
                        gps="GPS", denied="GPS denied", transition="GPS loss and return"
                    )[scenario]
                )
    fig.suptitle(
        "Frozen common captures: ESKF leads horizontally; vertical gap remains\nMedians and ranges of 4 cases (2 heights × 2 coupled motion/seed choices)",
        fontsize=12,
    )
    fig.supxlabel(
        "13–60 s; explicit flow exposure; unequal effective R/Q and policies\nNative flight pins: PX4 f1c0a1f / ArduPilot 1511f27; release port kernels tested separately",
        fontsize=9,
    )
    fig.savefig(
        args.output / "estimator-theory-comparison.svg", metadata={"Date": None}
    )
    fig.savefig(args.output / "estimator-theory-comparison.png", dpi=160)
    plt.close(fig)
    print(
        json.dumps(
            dict(
                rows=432,
                identical=sum(r["output_identical"] for r in flights["scores"]),
                controls=controls,
            )
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("native", "flights", "proofs", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
