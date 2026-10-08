"""Replay sensor-informed native settings with full-covariance observer parity."""

import argparse
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np

from audit_native_configuration import prepare_px4, probe, probe_px4
from compare_delay import PROFILES
from compare_exposure import digest
from compare_native_consistency import verify_observers
from native_aiding_noise import sensor_informed_noise
from native_consistency import check


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose fresh owned work and evidence paths")
    reference = json.loads(args.reference.read_text())
    observed_reference = json.loads(args.observer_reference.read_text())
    if (
        reference.get("native_campaign") != "stable-releases"
        or observed_reference["reference_sha256"] != digest(args.reference)
        or digest(args.transport_trace) != reference["binary_sha256"]["transport_trace"]
    ):
        raise ValueError("Require the frozen stable-release campaign and its observers")
    observers = verify_observers(args)
    if observers != observed_reference["observer_manifests"]:
        raise ValueError("Read-only observer manifests changed")
    binaries = {
        "control": dict(px4=args.px4_library, ekf3=args.ap_replay),
        "observed": dict(px4=args.px4_observed_library, ekf3=args.ap_observed_replay),
    }
    for variant, paths in binaries.items():
        expected = (reference if variant == "control" else observed_reference)[
            "native_core_sha256"
        ]
        if any(digest(path) != expected[name] for name, path in paths.items()):
            raise ValueError("Audited stable native core binary changed")
    args.work.mkdir(parents=True)
    base_work = args.work
    shared = dict(
        **vars(args),
        sensor_informed_noise=True,
        exposure_flow=True,
        px4_release="v1.17.0",
        imu_noise_density=reference["imu_noise_density"],
        delay_profile="exposure",
        takeoff_clear_s=18,
        retain_output=True,
        retain_native_run=True,
    )
    px4 = {}
    for variant, paths in binaries.items():
        work = base_work / ("px4-adapter-" + variant)
        work.mkdir()
        px4[variant] = prepare_px4(
            SimpleNamespace(**(shared | dict(work=work, px4_library=paths["px4"])))
        )
    result = dict(
        reference_sha256=digest(args.reference),
        observer_reference_sha256=digest(args.observer_reference),
        profile=sensor_informed_noise(),
        observer_manifests=observers,
        native_core_sha256={
            variant: {name: digest(path) for name, path in paths.items()}
            for variant, paths in binaries.items()
        },
        adapter_sha256={variant: digest(binary) for variant, binary in px4.items()},
        scores=[],
        scope="Sensor-informed native configuration ablation on unchanged captures and arrivals. Native enforced floors, priors and source policies remain unequal to ESKF. Not a fully matched R/Q or NIS study.",
    )
    for previous in reference["scores"]:
        if previous["name"] not in ("px4", "ekf3") or not previous["explicit_exposure"]:
            continue
        name, scenario = previous["name"], previous["scenario"]
        source = (
            f"{previous['frequency']}_{previous['seed']}_{previous['climb_height_m']}m"
        )
        label = source + "_" + scenario + "_explicit"
        original = args.cases / label / "capture"
        for filename, expected in reference["input_sha256"][source].items():
            if digest(original / filename) != expected:
                raise ValueError("Frozen physical capture changed")
        case = base_work / (label + "-" + name)
        capture = case / "capture"
        capture.mkdir(parents=True)
        for path in original.iterdir():
            if path.is_file() and path.name != "arrivals.csv":
                (capture / path.name).symlink_to(path.resolve())
        trace = subprocess.run(
            [
                str(args.transport_trace),
                str(capture / "modelica_input.csv"),
                scenario,
                *map(str, PROFILES["exposure"]),
                str(previous["seed"]),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        (capture / "arrivals.csv").write_text(trace.stdout)
        if (
            digest(capture / "arrivals.csv") != previous["arrival_trace_sha256"]
            or json.loads(trace.stderr) != previous["transport"]
        ):
            raise ValueError("Frozen packet delivery changed")
        scores = {}
        covariance = case / "observed/native-covariance.csv"
        for variant, paths in binaries.items():
            work = case / variant
            work.mkdir()
            options = SimpleNamespace(
                **(shared | dict(work=work, capture=capture, ap_replay=paths["ekf3"]))
            )
            old_environment = os.environ.pop("NATIVE_COVARIANCE_PATH", None)
            if variant == "observed":
                os.environ["NATIVE_COVARIANCE_PATH"] = str(covariance.resolve())
            try:
                score = (
                    probe_px4(options, px4[variant], capture, scenario)
                    if name == "px4"
                    else probe(
                        options,
                        capture,
                        np.genfromtxt(
                            capture / "arrivals.csv", delimiter=",", names=True
                        ),
                        scenario,
                        "baseline",
                    )
                )
            finally:
                os.environ.pop("NATIVE_COVARIANCE_PATH", None)
                if old_environment is not None:
                    os.environ["NATIVE_COVARIANCE_PATH"] = old_environment
            if name == "ekf3":
                expected = result["profile"]["ekf3"]
                if any(
                    not np.isclose(
                        score["parameters"].get(key, np.nan), value, rtol=1e-6, atol=0
                    )
                    for key, value in expected.items()
                ):
                    raise ValueError(
                        "Logged EKF3 noise parameters differ from the declared profile"
                    )
            scores[variant] = score
        identical = (
            scores["control"]["output_sha256"] == scores["observed"]["output_sha256"]
        )
        row = dict(
            **{
                key: previous[key]
                for key in ("name", "seed", "frequency", "climb_height_m", "scenario")
            },
            native=scores["observed"],
            control_output_sha256=scores["control"]["output_sha256"],
            observer_output_identical=identical,
            arrival_trace_sha256=digest(capture / "arrivals.csv"),
        )
        result["scores"].append(row)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        if not identical:
            raise ValueError(
                "Read-only observer changed native outputs; retain the failed run"
            )
        evidence = case / "observed/consistency.json"
        try:
            check(
                SimpleNamespace(
                    filter=name,
                    covariance=covariance,
                    truth=capture / "truth.csv",
                    output=evidence,
                )
            )
        except np.linalg.LinAlgError as error:
            row["consistency"] = dict(
                valid=False, reason=str(error), covariance_sha256=digest(covariance)
            )
        else:
            row["consistency"] = dict(valid=True, **json.loads(evidence.read_text()))
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        print(
            label,
            name,
            "observer identical",
            score["flight"]["horizontal_position_rmse_m"],
            flush=True,
        )
    if len(result["scores"]) != 24:
        raise ValueError("Require all twenty-four native conditions")
    result["complete"] = True
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "reference",
        "observer-reference",
        "cases",
        "harness",
        "px4-source",
        "px4-library",
        "px4-observed-library",
        "ardupilot-source",
        "px4-observer",
        "ardupilot-observer",
        "ap-replay",
        "ap-observed-replay",
        "transport-trace",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", default="c++")
    run(parser.parse_args())
