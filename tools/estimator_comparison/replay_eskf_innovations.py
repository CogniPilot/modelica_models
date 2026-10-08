"""Qualify an ESKF innovation observer against a frozen matched replay."""

import argparse
import json
import os
from pathlib import Path
import subprocess

from manifest import digest


VARIANTS = {
    "horizon": "horizon-rest",
    "retrodiction": "retrodiction-rest",
    "horizon_joint": "horizon-rest-joint",
    "retrodiction_joint": "retrodiction-rest-joint",
}


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose fresh owned replay and evidence paths")
    reference = json.loads(args.reference.read_text())
    if not reference["complete"] or not reference["readiness_qualified"]:
        raise ValueError("Require a completed readiness-qualified reference")
    if reference["noise_profile"] != "common-native-floors-v1":
        raise ValueError("This qualification uses the common sensor noise contract")
    noise = json.loads(args.noise_reference.read_text())["imu_noise_density"]
    args.work.mkdir(parents=True)
    result = dict(
        complete=False,
        reference_sha256=digest(args.reference),
        noise_reference_sha256=digest(args.noise_reference),
        imu_noise_density=noise,
        source_sha256={
            name: digest(Path(__file__).with_name(name))
            for name in (
                "replay_eskf_innovations.py",
                "instrument_eskf_innovations.py",
                "eskf_innovation_dump.h",
            )
        },
        cases=[],
        scope="Observer qualification only: exact state and complete raw covariance CSV parity with the frozen replay. Timing is affected by logging. Per-sensor NIS requires separate independent reconstruction; no estimator ranking is inferred here.",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.output.write_text(json.dumps(result, indent=2) + "\n")

    save()
    try:
        for scenario in args.scenarios:
            capture = args.captures / scenario / "capture"
            inputs = {
                name: digest(capture / name) for name in reference["input_sha256"]
            }
            if inputs != reference["input_sha256"]:
                raise ValueError("Capture differs from the frozen physical inputs")
            for name in args.names:
                frozen = next(
                    row
                    for row in reference["scores"]
                    if row["name"] == name and row["scenario"] == scenario
                )
                baseline = args.baseline_dir / VARIANTS[name]
                observed = args.observed_dir / VARIANTS[name]
                if digest(baseline) != reference["binary_sha256"][name]:
                    raise ValueError("Baseline binary differs from the frozen replay")
                case = dict(
                    name=name,
                    scenario=scenario,
                    input_sha256=inputs,
                    baseline_binary_sha256=digest(baseline),
                    observer_binary_sha256=digest(observed),
                    executions={},
                )
                result["cases"].append(case)
                work = args.work / scenario / name
                work.mkdir(parents=True)
                for policy, binary in (("baseline", baseline), ("observer", observed)):
                    covariance = work / (policy + "-covariance.csv")
                    state = work / (policy + "-states.csv")
                    log = work / (policy + "-timing.json")
                    innovations = work / "innovations.csv"
                    command = [
                        str(binary),
                        str(capture / "imu.csv"),
                        str(capture / "modelica_input.csv"),
                        "-",
                        scenario,
                        "foh",
                        str(covariance),
                        "--stationary-until",
                        str(reference["mission"]["warmup_s"]),
                        "--delays",
                        "110",
                        "30",
                        "60",
                        "20",
                        "0",
                        str(reference["origin"]["seed"]),
                        "--imu-noise-density",
                        *map(str, noise),
                        "--timing",
                        "-",
                        "--mission-offset",
                        str(reference["mission"]["warmup_s"] - 13),
                        "--measurement-noise",
                        "1e-6",
                        ".075",
                        "--flow-packets",
                        str(capture / "flow.csv"),
                    ]
                    environment = os.environ.copy()
                    environment.pop("ESKF_INNOVATION_PATH", None)
                    if policy == "observer":
                        environment["ESKF_INNOVATION_PATH"] = str(innovations)
                    with state.open("xb") as stdout, log.open("xb") as stderr:
                        subprocess.run(
                            command,
                            stdout=stdout,
                            stderr=stderr,
                            env=environment,
                            check=True,
                        )
                    timing = json.loads(log.read_text())
                    if any(timing[k] != v for k, v in frozen["transport"].items()):
                        raise ValueError("Packet delivery differs from the reference")
                    state_sha = digest(state)
                    if state_sha != frozen["output_sha256"]:
                        raise ValueError(
                            "Published states differ from the frozen replay"
                        )
                    case["executions"][policy] = dict(
                        command=command,
                        states_sha256=state_sha,
                        covariance_sha256=digest(covariance),
                        timing_sha256=digest(log),
                    )
                    save()
                baseline_cov = case["executions"]["baseline"]["covariance_sha256"]
                observer_cov = case["executions"]["observer"]["covariance_sha256"]
                if baseline_cov != observer_cov:
                    raise ValueError("Observer changed the complete covariance output")
                if not innovations.is_file() or innovations.stat().st_size == 0:
                    raise ValueError("Missing per-update observations")
                case.update(
                    state_parity=True,
                    full_covariance_parity=True,
                    innovations_sha256=digest(innovations),
                )
                save()
                print(f"Qualified {scenario} {name}", flush=True)
        result["complete"] = True
    except Exception as error:
        result["error"] = repr(error)
        raise
    finally:
        save()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for option in (
        "reference",
        "noise-reference",
        "captures",
        "baseline-dir",
        "observed-dir",
        "work",
        "output",
    ):
        parser.add_argument("--" + option, type=Path, required=True)
    parser.add_argument("--names", choices=VARIANTS, nargs="+", default=list(VARIANTS))
    parser.add_argument(
        "--scenarios",
        choices=("gps", "denied", "transition"),
        nargs="+",
        default=["gps", "denied", "transition"],
    )
    run(parser.parse_args())
