"""Compare declared-rest pressure calibration with frozen ESKF/native replays."""

import argparse
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace


from compare_delay import PROFILES
from compare_exposure import digest, eskf
from score import read


WINDOWS = (
    ("before_takeoff", 10, 13),
    ("flight", 13, 59.7),
    ("outage", 25, 40),
    ("after_return", 40, 59.7),
)


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    reference = json.loads(args.reference.read_text())
    if reference.get("native_campaign") != "stable-releases":
        raise ValueError("Use the completed stable-release comparison")
    if digest(args.transport_trace) != reference["binary_sha256"]["transport_trace"]:
        raise ValueError("Frozen packet transport executable changed")
    binaries = {
        (method, variant): getattr(args, method + "_" + variant)
        for method in ("horizon", "retrodiction")
        for variant in ("control", "candidate")
    }
    result = dict(
        reference_sha256=digest(args.reference),
        binary_sha256={
            method + "_" + variant: digest(binary)
            for (method, variant), binary in binaries.items()
        },
        input_sha256=reference["input_sha256"],
        scores=[],
        scope="Optional declared startup rest calibration; identical physical captures and arrivals. Effective native R/Q and priors remain unequal. Native scores are frozen references, not freshly replayed here.",
    )
    args.work.mkdir(parents=True)
    cases = {}
    for previous in reference["scores"]:
        if (
            previous["name"] not in ("horizon", "retrodiction")
            or not previous["explicit_exposure"]
        ):
            continue
        source = (
            f"{previous['frequency']}_{previous['seed']}_{previous['climb_height_m']}m"
        )
        label = source + "_" + previous["scenario"] + "_explicit"
        if label not in cases:
            original = args.cases / label / "capture"
            for name, expected in reference["input_sha256"][source].items():
                if digest(original / name) != expected:
                    raise ValueError("Frozen physical capture changed")
            capture = args.work / label / "capture"
            capture.mkdir(parents=True)
            for path in original.iterdir():
                if path.is_file() and path.name != "arrivals.csv":
                    (capture / path.name).symlink_to(path.resolve())
            trace = subprocess.run(
                [
                    str(args.transport_trace),
                    str(capture / "modelica_input.csv"),
                    previous["scenario"],
                    *map(str, PROFILES["exposure"]),
                    str(previous["seed"]),
                ],
                text=True,
                capture_output=True,
                check=True,
            )
            (capture / "arrivals.csv").write_text(trace.stdout)
            if (
                digest(capture / "arrivals.csv") != previous["arrival_trace_sha256"]
                or json.loads(trace.stderr) != previous["transport"]
            ):
                raise ValueError("Frozen packet delivery differs")
            cases[label] = capture
        for variant in ("control", "candidate"):
            method = previous["name"]
            work = args.work / label / (method + "_" + variant)
            work.mkdir()
            options = SimpleNamespace(
                work=work,
                delay_profile="exposure",
                seed=previous["seed"],
                imu_noise_density=reference["imu_noise_density"],
                consistency_windows=WINDOWS,
                retain_covariance=True,
            )
            score = eskf(
                binaries[method, variant],
                cases[label],
                previous["scenario"],
                options,
                True,
                previous["transport"],
            )
            diagnostics = read(work / "covariance.csv")
            epoch = diagnostics["fusion_t_s" if method == "horizon" else "t_s"]
            datum = {}
            for window, start, end in WINDOWS:
                rows = diagnostics[(epoch >= start) & (epoch < end)]
                if not len(rows):
                    raise ValueError("Missing pressure-datum diagnostic window")
                datum[window] = dict(
                    calibration_samples=int(rows["barometer_calibration_samples"][-1]),
                    terminal_bias_m=float(rows["barometer_bias_m"][-1]),
                    terminal_bias_variance_m2=float(
                        rows["barometer_bias_variance_m2"][-1]
                    ),
                )
            identical = score["output_sha256"] == previous["output_sha256"]
            row = dict(
                **score,
                **{
                    key: previous[key]
                    for key in ("seed", "frequency", "climb_height_m", "scenario")
                },
                name=method,
                variant=variant,
                control_output_identical=identical if variant == "control" else None,
                arrival_trace_sha256=digest(cases[label] / "arrivals.csv"),
                datum=datum,
            )
            result["scores"].append(row)
            args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            if variant == "control" and not identical:
                raise ValueError("Fresh ESKF control changed; retain the failed run")
            print(
                label,
                method,
                variant,
                score["flight"]["vertical_position_rmse_m"],
                flush=True,
            )
    if len(result["scores"]) != 48:
        raise ValueError("Incomplete pressure-datum comparison")
    result["complete"] = True
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "reference",
        "cases",
        "horizon-control",
        "retrodiction-control",
        "horizon-candidate",
        "retrodiction-candidate",
        "transport-trace",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
