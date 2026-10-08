#!/usr/bin/env python3
"""Compare mean packets, 800 Hz ZOH, and 800 Hz FOH on frozen captures."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import time

from score import score


def replay(job):
    binary, data, output, mode, case, frequency, seed, diagnostics = job
    start = time.perf_counter()
    with output.with_suffix(".log").open("w") as log:
        subprocess.run(
            [
                str(binary),
                str(data / "imu.csv"),
                str(data / "modelica_input.csv"),
                str(output),
                case,
                mode,
                *([str(output.with_suffix(".covariance.csv"))] if diagnostics else []),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    return dict(
        name="modelica",
        integration=mode,
        case=case,
        speed=frequency,
        seed=seed,
        csv=output.name,
        wall_s=time.perf_counter() - start,
        output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
    )


def ablate(args):
    output = args.output.resolve()
    if output.exists():
        raise ValueError("Use a new output directory to preserve previous evidence")
    output.mkdir(parents=True)
    summary = []
    for lower_rates in (False, True):
        profile = "low" if lower_rates else "high"
        for mode in args.integrations:
            directory = output / profile / mode
            directory.mkdir(parents=True)
            jobs = []
            for frequency in args.frequencies:
                for seed in args.seeds:
                    tag = f"{frequency}_{seed}"
                    data = args.captures / ("lower_" + tag if lower_rates else tag)
                    for case in ("gps", "denied", "transition"):
                        jobs.append(
                            (
                                args.replay,
                                data,
                                directory / f"modelica_{case}_{tag}.csv",
                                mode,
                                case,
                                frequency,
                                seed,
                                args.diagnostics,
                            )
                        )
            with ThreadPoolExecutor(max_workers=args.workers) as pool:
                records = list(pool.map(replay, jobs))
            (directory / "runs.json").write_text(json.dumps(records, indent=2) + "\n")
            scores = score(directory, args.captures, lower_rates)
            for result in scores:
                flight = result["flight"]
                if flight["finite_fraction"] != 1 or flight["nonzero_step_status_rows"]:
                    raise ValueError(f"Invalid production replay: {result['csv']}")
            (directory / "scores.json").write_text(
                json.dumps(scores, indent=2, allow_nan=False) + "\n"
            )
            summary.extend(scores)
            print(f"{profile} {mode}: {len(scores)} valid replays", flush=True)
    (output / "scores.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--replay", type=lambda value: Path(value).resolve(), required=True
    )
    parser.add_argument("--captures", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seeds", nargs="+", type=int, default=[7, 19, 41])
    parser.add_argument("--frequencies", nargs="+", type=float, default=[0.25, 0.6])
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument(
        "--integrations",
        nargs="+",
        choices=["mean", "zoh", "foh"],
        default=["mean", "zoh", "foh"],
    )
    parser.add_argument("--diagnostics", action="store_true")
    ablate(parser.parse_args())
