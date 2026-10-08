"""Replay the stable native releases and ESKF on frozen exposure captures."""

import argparse
import json
from pathlib import Path
import subprocess

from audit_native_configuration import prepare_px4
from compare_exposure import digest, run
from native_release import SOURCE_PINS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "captures",
        "references",
        "harness",
        "horizon",
        "retrodiction",
        "px4-source",
        "px4-library",
        "ap-source",
        "ap-replay",
        "transport-trace",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    for name, source in (("px4", args.px4_source), ("ardupilot", args.ap_source)):
        revision = subprocess.check_output(
            ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
        ).strip()
        if revision != SOURCE_PINS[name]:
            raise ValueError(f"Wrong stable {name} source revision")
        changes = subprocess.check_output(
            ["git", "-C", str(source), "status", "--porcelain", "--untracked-files=no"],
            text=True,
        )
        if changes:
            raise ValueError(f"Native {name} sources must remain unmodified")
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    reference = json.loads((args.references / "native-exposure.json").read_text())
    for label, files in reference["input_sha256"].items():
        for name, expected in files.items():
            if digest(args.captures / label / name) != expected:
                raise ValueError(f"Frozen capture changed: {label}/{name}")
    if digest(args.transport_trace) != reference["binary_sha256"]["transport_trace"]:
        raise ValueError("Use the validated frozen transport trace executable")
    args.native_campaign = "stable-releases"
    args.native_source_pins = SOURCE_PINS
    args.native_core_sha256 = {
        "px4": digest(args.px4_library),
        "ekf3": digest(args.ap_replay),
    }
    args.px4_release = "v1.17.0"
    args.imu_noise_density = json.loads(
        (args.references / "eskf-noise-calibration.json").read_text()
    )["imu_noise_density"]
    run_work = args.work
    build = args.work.with_name(args.work.name + "-adapters")
    build.mkdir()
    for exposure, label in ((False, "legacy"), (True, "explicit")):
        args.work = build / label
        args.work.mkdir()
        args.exposure_flow = exposure
        binary = prepare_px4(args)
        setattr(args, "px4_" + label, binary)
    args.work = run_work
    run(args)
    result = json.loads(args.output.read_text())
    keys = ("seed", "climb_height_m", "scenario", "explicit_exposure", "name")
    previous = {tuple(row[key] for key in keys): row for row in reference["scores"]}
    controls = []
    for row in result["scores"]:
        identifier = tuple(row[key] for key in keys)
        baseline = previous[identifier]
        if (
            row["arrival_trace_sha256"] != baseline["arrival_trace_sha256"]
            or row["transport"] != baseline["transport"]
        ):
            raise ValueError(f"Frozen packet trace changed: {identifier}")
        controls.append(
            dict(
                **{key: row[key] for key in keys},
                identical=row["output_sha256"] == baseline["output_sha256"],
            )
        )
    result["frozen_controls"] = controls
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    if any(
        not row["identical"]
        for row in controls
        if row["name"] in ("horizon", "retrodiction")
    ):
        raise ValueError("ESKF frozen-output control changed; every result retained")


if __name__ == "__main__":
    main()
