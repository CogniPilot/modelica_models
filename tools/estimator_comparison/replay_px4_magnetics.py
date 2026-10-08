"""Observe public native magnetic states, requiring frozen state/innovation parity."""

import argparse
import json
import os
from pathlib import Path
import subprocess

from manifest import digest
from native_delay import replace_once
from native_release import SOURCE_PINS


def run(args):
    if args.work.exists() or args.output.exists():
        raise ValueError("Choose new owned work and evidence paths")
    pilot = json.loads(args.pilot.read_text())
    adapter = args.cases / "px4-arrivals.cpp"
    revision = subprocess.check_output(
        ["git", "-C", str(args.px4_source), "rev-parse", "HEAD"], text=True
    ).strip()
    if (
        not pilot["complete"]
        or revision != SOURCE_PINS["px4"]
        or digest(adapter) != pilot["px4_adapter_sha256"]
        or digest(args.px4_library) != pilot["binary_sha256"]["px4_library"]
    ):
        raise ValueError("Require the complete frozen pilot, adapter and native core")
    observer = pilot["observer_manifests"]["px4"]
    for record in observer["files"]:
        if digest(args.px4_source / record["file"]) != record["after_sha256"]:
            raise ValueError("Native observer source changed")
    args.work.mkdir(parents=True)
    header = Path(__file__).with_name("native_magnetic_dump.h")
    staged = args.work / "observer.cpp"
    staged.write_text(
        '#include "native_magnetic_dump.h"\n'
        + replace_once(
            adapter.read_text(),
            "\t\t\t++n_updates;",
            "\t\t\t++n_updates;\n"
            "            writeNativeMagneticState(s.t - t_start, ekf, p->ekf2_mag_decl);",
        )
    )
    source = args.px4_source
    includes = [
        header.parent,
        args.harness / "px4/stubs",
        source / "src",
        source / "src/lib",
        source / "src/lib/matrix",
        source / "src/modules/ekf2/EKF",
        source / "src/modules/ekf2/EKF/python",
    ]
    binary = args.work / "observer"
    subprocess.run(
        [
            args.cxx,
            "-pipe",
            "-O3",
            "-DNDEBUG",
            "-std=c++17",
            *[
                f"-DCONFIG_EKF2_{feature}=1"
                for feature in (
                    "BAROMETER",
                    "GNSS",
                    "GRAVITY_FUSION",
                    "MAGNETOMETER",
                    "OPTICAL_FLOW",
                    "RANGE_FINDER",
                    "TERRAIN",
                )
            ],
            '-DMODULE_NAME="ekf2_replay"',
            "-D__PX4_POSIX=1",
            *["-I" + str(path) for path in includes],
            str(staged),
            str(args.px4_library),
            "-lm",
            "-o",
            str(binary),
        ],
        check=True,
    )
    evidence = dict(
        pilot_sha256=digest(args.pilot),
        native_revision=revision,
        adapter_sha256=digest(adapter),
        observed_adapter_sha256=digest(staged),
        observer_header_sha256=digest(header),
        library_sha256=digest(args.px4_library),
        binary_sha256=digest(binary),
        results=[],
        scope="Read-only public magnetic-state getters after each native update. No native core edits. Every state and innovation byte must match the frozen pilot.",
    )
    for score in pilot["scores"]:
        if score["name"] != "px4":
            continue
        scenario = score["scenario"]
        capture = args.cases / scenario / "capture"
        for filename, expected in pilot["input_sha256"].items():
            if digest(capture / filename) != expected:
                raise ValueError("Physical capture changed")
        if digest(capture / "arrivals.csv") != score["arrival_trace_sha256"]:
            raise ValueError("Sensor arrival trace changed")
        work = args.work / scenario
        work.mkdir()
        magnetic, innovations, estimate = (
            work / name for name in ("magnetic.csv", "innovations.csv", "estimate.csv")
        )
        environment = dict(
            os.environ,
            NATIVE_MAG_STATE_PATH=str(magnetic.resolve()),
            NATIVE_INNOVATION_PATH=str(innovations.resolve()),
        )
        with (work / "run.log").open("w") as log:
            subprocess.run(
                [
                    str(binary),
                    "--input",
                    str(capture),
                    "--output",
                    str(estimate),
                    "--arm-after",
                    str(pilot["mission"]["warmup_s"]),
                    *(["--no-gps"] if scenario == "denied" else []),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                env=environment,
                check=True,
            )
        if (
            digest(estimate) != score["output_sha256"]
            or digest(innovations) != score["innovations"]["observer_csv_sha256"]
        ):
            raise ValueError("Magnetic observer changed native states or innovations")
        evidence["results"].append(
            dict(
                scenario=scenario,
                state_parity=True,
                innovation_parity=True,
                state_sha256=digest(estimate),
                innovation_sha256=digest(innovations),
                magnetic_sha256=digest(magnetic),
            )
        )
        print(f"Verified {scenario} state and innovation byte parity", flush=True)
    if digest(args.px4_library) != evidence["library_sha256"]:
        raise ValueError("Native core changed during observation")
    args.output.write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "pilot",
        "cases",
        "harness",
        "px4-source",
        "px4-library",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--cxx", required=True)
    run(parser.parse_args())
