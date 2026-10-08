"""Reproduce clocked assignment and tunable-parameter export behavior."""

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reproduce(args):
    fixtures = Path(__file__).resolve().parent
    args.work.mkdir(parents=True, exist_ok=False)
    temporary = args.work / "tmp"
    temporary.mkdir()
    environment = dict(os.environ, TMPDIR=str(temporary))
    result = dict(complete=False, cases=[])

    def run(command, log):
        process = subprocess.run(
            command, env=environment, text=True, capture_output=True
        )
        log.write_text(process.stdout + process.stderr)
        process.check_returncode()

    for name, known_failures in (
        ("ReassignedFlag", 8),
        ("ReassignedFlagFixed", 0),
        ("ReassignedFlagVariable", 0),
        ("ReassignedFlagParameter", 8),
        ("ReassignedFlagParameterStatement", 0),
    ):
        source = args.work / (name + ".mo")
        source.write_bytes((fixtures / (name + ".mo.txt")).read_bytes())
        harness = args.work / (name + "-probe.c")
        harness.write_bytes((fixtures / (name + "-probe.c.txt")).read_bytes())
        export = args.work / (name + "-export")
        compile_command = [
            str(args.rumoca),
            "compile",
            str(source),
            "--model",
            name,
            "--target",
            "galec-production",
            "--output",
            str(export),
            "--cache-dir",
            str(args.work / "cache"),
        ]
        run(compile_command, args.work / (name + "-export.log"))
        production = export / name / "ProductionCode"
        binary = args.work / name
        build_command = [
            str(args.cc),
            "-O2",
            "-I" + str(production),
            str(harness),
            str(production / (name + ".c")),
            str(production / "rumoca_galec_kernels.c"),
            "-lm",
            "-o",
            str(binary),
        ]
        run(build_command, args.work / (name + "-build.log"))
        output = subprocess.check_output([str(binary)], env=environment, text=True)
        (args.work / (name + ".csv")).write_text(output)
        rows = list(csv.DictReader(io.StringIO(output)))
        failures = sum(row["flag"] != "1" for row in rows)
        result["cases"].append(
            dict(
                name=name,
                rows=len(rows),
                failed_expected_true=failures,
                known_rumoca_0_10_2_failures=known_failures,
                compile=compile_command,
                build=build_command,
                source_sha256=digest(source),
                harness_sha256=digest(harness),
                generated_sha256=digest(production / (name + ".c")),
                manifest_sha256=digest(export / name / "AlgorithmCode/manifest.xml"),
                binary_sha256=digest(binary),
            )
        )
        if len(rows) != 16:
            raise ValueError("Incomplete probe output")
    result.update(
        complete=True,
        all_source_semantics_pass=all(
            row["failed_expected_true"] == 0 for row in result["cases"]
        ),
        known_failure_reproduction=all(
            row["failed_expected_true"] == row["known_rumoca_0_10_2_failures"]
            for row in result["cases"]
        ),
        scope=(
            "Every source algorithm requires true for all 16 samples. Known failures "
            "describe the observed Rumoca 0.10.2 defects, not valid source semantics. "
            "This is a diagnostic fixture, not an estimator correctness certificate."
        ),
    )
    (args.work / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {row["name"]: row["failed_expected_true"] for row in result["cases"]}
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("rumoca", "cc", "work"):
        parser.add_argument(
            "--" + option, type=lambda p: Path(p).resolve(), required=True
        )
    reproduce(parser.parse_args())
