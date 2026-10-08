#!/usr/bin/env python3
"""Run external native cores and generated Modelica on identical captures."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from generate import generate


def run(args):
    root = args.output.resolve()
    results = root / ("results-lower-rates" if args.lower_rates else "results")
    results.mkdir(parents=True, exist_ok=True)
    records = []
    for speed in args.speeds:
        for seed in args.seeds:
            tag = f"{speed}_{seed}"
            data = root / "data" / ("lower_" + tag if args.lower_rates else tag)
            generate(data, seed, speed, args.lower_rates)
            for case, flags in [
                ("gps", []),
                ("denied", ["--no-gps"]),
                ("transition", ["--gps-deny", "25:40"]),
            ]:
                for name in ["modelica", "px4", "ekf3"] + (
                    ["ukf"] if args.ukf_replay else []
                ):
                    stem = f"{name}_{case}_{tag}"
                    output = results / (stem + ".csv")
                    if name in ("modelica", "ukf"):
                        binary = (
                            args.ukf_replay if name == "ukf" else args.modelica_replay
                        )
                        cmd = [
                            str(binary),
                            str(data / "imu.csv"),
                            str(data / "modelica_input.csv"),
                            str(output),
                            case,
                        ]
                    elif name == "px4":
                        cmd = [
                            str(args.px4_replay),
                            "--input",
                            str(data),
                            "--output",
                            str(output),
                            *flags,
                        ]
                    else:
                        cmd = [
                            str(args.native_harness / "ardupilot/apenv"),
                            "python3",
                            str(args.native_harness / "ardupilot/run_ekf3.py"),
                            "--input",
                            str(data),
                            "--output",
                            str(output),
                            "--arm-after",
                            "13",
                            "--workdir",
                            str(
                                root
                                / "replay-runs"
                                / ("lower_" + stem if args.lower_rates else stem)
                            ),
                            *flags,
                        ]
                    env = os.environ.copy()
                    env["AP_REPLAY"] = str(args.ap_replay)
                    start = time.perf_counter()
                    with (results / (stem + ".log")).open("w") as log:
                        subprocess.run(
                            cmd,
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            env=env,
                            check=True,
                        )
                    records.append(
                        dict(
                            name=name,
                            case=case,
                            speed=speed,
                            seed=seed,
                            csv=output.name,
                            wall_s=time.perf_counter() - start,
                        )
                    )
                    print(stem, "done", flush=True)
                    (results / "runs.json").write_text(
                        json.dumps(records, indent=2) + "\n"
                    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--native-harness", type=lambda s: Path(s).resolve(), required=True)
    for flag in ["modelica-replay", "px4-replay", "ap-replay"]:
        p.add_argument("--" + flag, type=lambda s: Path(s).resolve(), required=True)
    p.add_argument("--seeds", nargs="+", type=int, default=[7, 19, 41])
    p.add_argument("--speeds", nargs="+", type=float, default=[0.25, 0.6])
    p.add_argument("--lower-rates", action="store_true")
    p.add_argument("--ukf-replay", type=lambda s: Path(s).resolve())
    run(p.parse_args())
