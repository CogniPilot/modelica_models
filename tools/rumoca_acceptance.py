#!/usr/bin/env python3
"""On-demand acceptance rig for the two open Rumoca record-lowering defects.

Given a candidate `rumoca` binary, run every check that decides whether the
AS-051 Boolean-connector balance fix and the AS-041/AS-042 record-call
materialization fix have landed without moving anything else, and print one
PASS/FAIL table.

This is NOT run by CI. It compiles the largest models in the repository and
runs Valgrind over generated code, which costs minutes, and it needs an
out-of-tree Rumoca checkout for the corpus-pin row. Run it by hand when a
compiler branch asks to become the pin.

Every row carries what the current pin measures and what the fix must produce,
so a run against the pin binary documents the defect and a run against a
candidate documents the repair. Rows a and b are EXPECTED TO FAIL against the
pin: that failure is the AS-051 defect.

    python3 tools/rumoca_acceptance.py --rumoca /path/to/candidate/rumoca \
        --baseline-rumoca /path/to/d4d80fbb/rumoca \
        --rumoca-src /path/to/rumoca/checkout \
        --msl-root /path/to/msl

Run `--rows repro-balance,horizon-balance` for the two cheap AS-051 rows.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Sequence


# What the models CI pin, Rumoca d4d80fbb, measures today. A candidate that
# reproduces these exactly has changed nothing; a candidate that changes one
# without changing the others is the interesting case the table is for.
PIN = "d4d80fbb"

# The re-base cost the horizon algebra actually needs, from
# docs/delayed-fusion-horizon-wcet.md: 22 SE_2(3) compositions at about 1,480
# instructions each. The ceiling is loose because the post-fix figure is not
# yet known, and a re-base three times the algebraic cost would still be two
# orders of magnitude better than what is measured now.
REBASE_CEILING = 100_000

ANSI = re.compile(r"\x1b\[[0-9;]*m")
BALANCE = re.compile(
    r"unbalanced model: (\d+) equations, (\d+) unknowns \(balance = (-?\d+)\)"
)
# Volatile by design: a fresh UUID and a wall-clock stamp per run, and the
# container checksums that digest them. Everything else in an eFMU must be a
# pure function of the model and the compiler.
VOLATILE = (
    re.compile(r'(id|manifestRefId)="\{[0-9a-fA-F-]+\}"'),
    re.compile(r'generationDateAndTime="[^"]*"'),
    re.compile(r'checksum="[0-9a-f]*"'),
)

FLIGHT_ARTIFACTS = (
    ("Vehicles/Rdd2/NavigationEstimator.mo", "Vehicles.Rdd2.NavigationEstimator"),
    (
        "Planning/Bezier/WaypointTrajectoryPlanner.mo",
        "Planning.Bezier.WaypointTrajectoryPlanner",
    ),
    ("Vehicles/Rdd2/GuidanceController.mo", "Vehicles.Rdd2.GuidanceController"),
)


class Row:
    """One acceptance check and the two numbers that bracket it."""

    def __init__(self, identifier: str, expected_pin: str, expected_fixed: str):
        self.identifier = identifier
        self.expected_pin = expected_pin
        self.expected_fixed = expected_fixed
        self.verdict = "UNMEASURED"
        self.measured = "not run"
        self.notes: list[str] = []

    def resolve(self, passed: bool, measured: str) -> "Row":
        self.verdict = "PASS" if passed else "FAIL"
        self.measured = measured
        return self

    def unmeasured(self, why: str) -> "Row":
        self.verdict = "UNMEASURED"
        self.measured = why
        return self


def main(arguments: Sequence[str] | None = None) -> int:
    options = parse_arguments(arguments)
    repository = find_repository_root(Path(__file__).resolve().parent)
    work = Path(options.work) if options.work else Path(
        tempfile.mkdtemp(prefix="rumoca-acceptance-")
    )
    work.mkdir(parents=True, exist_ok=True)

    checks = {
        "repro-balance": lambda: check_repro_balance(options, repository, work),
        "horizon-balance": lambda: check_horizon_balance(options, repository, work),
        "corpus-pin": lambda: check_corpus_pin(options, repository, work),
        "flight-artifacts": lambda: check_flight_artifacts(options, repository, work),
        "wcet-oracle": lambda: check_wcet_oracle(options, repository, work),
        "record-call-cardinality": lambda: check_call_cardinality(
            options, repository, work
        ),
    }
    selected = options.rows.split(",") if options.rows else list(checks)
    unknown = [name for name in selected if name not in checks]
    if unknown:
        print(f"no such row: {', '.join(unknown)}", file=sys.stderr)
        print(f"rows are: {', '.join(checks)}", file=sys.stderr)
        return 2

    print(f"rumoca-acceptance: candidate {options.rumoca}")
    print(f"rumoca-acceptance: work {work}")
    rows = []
    for name in selected:
        print(f"==> {name}", flush=True)
        rows.append(checks[name]())
    return report(rows)


def parse_arguments(arguments: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--rumoca", required=True, help="candidate compiler binary under test"
    )
    parser.add_argument(
        "--baseline-rumoca",
        help=f"pin ({PIN}) binary; without it the flight-artifact row is unmeasured",
    )
    parser.add_argument(
        "--rumoca-src",
        help="Rumoca checkout providing `cargo xtask verify corpus-pin`; "
        "without it the corpus-pin row is unmeasured",
    )
    parser.add_argument(
        "--msl-root",
        help="Modelica Standard Library root for the corpus-pin row; the "
        "directory holding `Modelica 4.1.0/package.mo`",
    )
    parser.add_argument("--work", help="scratch directory (default: a fresh temporary)")
    parser.add_argument("--rows", help="comma-separated subset of rows to run")
    parser.add_argument(
        "--timeout",
        type=int,
        default=3600,
        help="per-command ceiling in seconds (default: 3600)",
    )
    return parser.parse_args(arguments)


def find_repository_root(start: Path) -> Path:
    for directory in (start, *start.parents):
        if (directory / "Vehicles/package.mo").is_file():
            return directory
    raise SystemExit("rumoca-acceptance: run this from inside the models repository")


def compile_model(
    options: argparse.Namespace,
    repository: Path,
    model_file: str,
    model: str,
    output: Path,
    emission: Sequence[str],
    binary: str | None = None,
) -> tuple[int, str, float]:
    """Compile one model, returning the exit status, clean log, and seconds."""
    command = [
        binary or options.rumoca,
        "compile",
        model_file,
        "--model",
        model,
        "--source-root",
        str(repository),
        *emission,
        "--output",
        str(output),
    ]
    started = time.monotonic()
    try:
        finished = subprocess.run(
            command,
            cwd=repository,
            capture_output=True,
            text=True,
            timeout=options.timeout,
        )
    except subprocess.TimeoutExpired:
        return 124, f"timed out after {options.timeout}s", float(options.timeout)
    elapsed = time.monotonic() - started
    return (
        finished.returncode,
        ANSI.sub("", finished.stdout + finished.stderr),
        elapsed,
    )


def describe_balance(log: str) -> str:
    match = BALANCE.search(log)
    if not match:
        return "refused without a balance diagnostic"
    equations, unknowns, balance = match.groups()
    return f"ED001 {equations} equations / {unknowns} unknowns (balance = {balance})"


def check_repro_balance(
    options: argparse.Namespace, repository: Path, work: Path
) -> Row:
    row = Row(
        "repro-balance",
        "ED001 8 equations / 6 unknowns (balance = 2)",
        "balanced, lowers",
    )
    status, log, _ = compile_model(
        options,
        repository,
        "tools/rumoca-repros/connector-boolean-balance/ConnectorBooleanBalance.mo",
        "ConnectorBooleanBalance.Outer",
        work / "connector-boolean-balance.dae.json",
        ("--emit", "dae-json"),
    )
    if status == 0:
        return row.resolve(True, "balanced, lowers")
    return row.resolve(False, describe_balance(log))


def check_horizon_balance(
    options: argparse.Namespace, repository: Path, work: Path
) -> Row:
    # The balance moved from 14 to 26 when the delayed measurement queues
    # landed, and the new number is not a regression: it is the SAME defect
    # counting more of the same thing. AS-051 misses the Boolean components of
    # a sub-block's input connector, HorizonEstimator now holds an
    # Estimation.FusionHorizon.AidingBuffer beside the filter, and that buffer
    # carries twelve Booleans across its five aiding input connectors on top of
    # the filter's fourteen across six. The equation total moved with it
    # because the block grew, and again when the block began publishing the
    # horizon state alongside the predicted one. The BALANCE is the number the
    # row is about and it has not moved: the defect counts connector Booleans,
    # and neither change added an input connector.
    #
    # Keeping the old string here would have been the worst outcome available:
    # the row would have failed against a FIXED compiler for the wrong reason
    # and read as the defect surviving.
    row = Row(
        "horizon-balance",
        "ED001 4549 equations / 4523 unknowns (balance = 26)",
        "balanced at 4549/4549, lowers",
    )
    status, log, elapsed = compile_model(
        options,
        repository,
        "Estimation/FusionHorizon/HorizonEstimator.mo",
        "Estimation.FusionHorizon.HorizonEstimator",
        work / "horizon-estimator.dae.json",
        ("--emit", "dae-json"),
    )
    row.notes.append(f"lowering {elapsed:.1f}s")
    if status == 0:
        return row.resolve(True, "balanced, lowers")
    return row.resolve(False, describe_balance(log))


def check_corpus_pin(options: argparse.Namespace, repository: Path, work: Path) -> Row:
    row = Row("corpus-pin", "28 row(s), 0 failed", "28 row(s), 0 failed")
    if not options.rumoca_src:
        return row.unmeasured("no --rumoca-src")
    if not options.msl_root:
        return row.unmeasured("no --msl-root")
    command = [
        "cargo",
        "xtask",
        "verify",
        "corpus-pin",
        "--models-root",
        str(repository),
        "--msl-root",
        options.msl_root,
        "--rumoca-binary",
        str(Path(options.rumoca).resolve()),
    ]
    try:
        finished = subprocess.run(
            command,
            cwd=options.rumoca_src,
            capture_output=True,
            text=True,
            timeout=options.timeout,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError) as error:
        return row.unmeasured(f"could not run cargo xtask: {error}")
    log = ANSI.sub("", finished.stdout + finished.stderr)
    (work / "corpus-pin.log").write_text(log, encoding="utf-8")
    match = re.search(r"corpus pin: (\d+) row\(s\), (\d+) failed", log)
    if not match:
        return row.unmeasured("no corpus-pin verdict line; see corpus-pin.log")
    total, failed = int(match.group(1)), int(match.group(2))
    measured = f"{total} row(s), {failed} failed"
    return row.resolve(finished.returncode == 0 and failed == 0 and total == 28, measured)


def normalize(path: Path) -> bytes:
    """Strip the per-run identity out of an eFMU manifest."""
    if path.suffix != ".xml":
        return path.read_bytes()
    text = path.read_text(encoding="utf-8", errors="replace")
    for pattern in VOLATILE:
        text = pattern.sub("", text)
    return text.encode("utf-8")


def artifact_digest(root: Path) -> dict[str, bytes]:
    """Every emitted file except the zip, which carries its own timestamps."""
    digest = {}
    for path in sorted(root.rglob("*")):
        if path.is_file() and path.suffix != ".efmu":
            digest[str(path.relative_to(root))] = normalize(path)
    return digest


def check_flight_artifacts(
    options: argparse.Namespace, repository: Path, work: Path
) -> Row:
    row = Row(
        "flight-artifacts",
        "3/3 identical to itself",
        f"3/3 byte-identical to {PIN}",
    )
    if not options.baseline_rumoca:
        return row.unmeasured("no --baseline-rumoca")
    identical, deltas = 0, []
    for model_file, model in FLIGHT_ARTIFACTS:
        tag = model.replace(".", "_")
        candidate_dir = work / "artifacts" / "candidate" / tag
        baseline_dir = work / "artifacts" / "baseline" / tag
        for target, binary in (
            (candidate_dir, options.rumoca),
            (baseline_dir, options.baseline_rumoca),
        ):
            status, log, _ = compile_model(
                options,
                repository,
                model_file,
                model,
                target,
                ("--target", "galec-production"),
                binary=binary,
            )
            if status != 0:
                deltas.append(f"{model}: compile failed for {binary}")
                (work / f"{tag}.log").write_text(log, encoding="utf-8")
                break
        else:
            candidate = artifact_digest(candidate_dir)
            baseline = artifact_digest(baseline_dir)
            differing = sorted(
                name
                for name in set(candidate) | set(baseline)
                if candidate.get(name) != baseline.get(name)
            )
            if differing:
                deltas.extend(f"{model}: {name}" for name in differing)
            else:
                identical += 1
    row.notes.extend(deltas)
    measured = f"{identical}/{len(FLIGHT_ARTIFACTS)} byte-identical"
    if deltas:
        measured += f"; {len(deltas)} delta(s), listed below"
    return row.resolve(identical == len(FLIGHT_ARTIFACTS), measured)


def check_wcet_oracle(options: argparse.Namespace, repository: Path, work: Path) -> Row:
    row = Row(
        "wcet-oracle",
        "re-base about 82M to 90M executed instructions",
        f"re-base at or under {REBASE_CEILING:,} executed instructions",
    )
    root = work / "wcet"
    if root.exists():
        shutil.rmtree(root)
    root.mkdir(parents=True)
    status, log, elapsed = compile_model(
        options,
        repository,
        "Estimation/FusionHorizon/OutputPredictor.mo",
        "Estimation.FusionHorizon.OutputPredictor",
        root / "gen",
        ("--target", "galec-production"),
    )
    row.notes.append(f"OutputPredictor lowering {elapsed:.1f}s")
    if status != 0:
        (root / "lower.log").write_text(log, encoding="utf-8")
        return row.resolve(False, f"OutputPredictor did not lower ({elapsed:.1f}s)")
    shutil.copy(repository / "tools/wcet/driver_horizon.c", root)
    environment = dict(os.environ, HORIZON_WCET_ROOT=str(root))
    try:
        finished = subprocess.run(
            ["bash", "tools/wcet/callgrind_horizon.sh"],
            cwd=repository,
            capture_output=True,
            text=True,
            env=environment,
            timeout=options.timeout,
        )
    except subprocess.TimeoutExpired:
        return row.unmeasured(f"callgrind timed out after {options.timeout}s")
    counts = {}
    for line in finished.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and all(field.isdigit() for field in fields):
            counts[(fields[0], fields[1])] = int(fields[2])
    (work / "callgrind.log").write_text(
        finished.stdout + finished.stderr, encoding="utf-8"
    )
    needed = [("401", "1"), ("401", "0"), ("400", "0")]
    if not all(key in counts for key in needed):
        return row.unmeasured("callgrind produced no counts; see callgrind.log")
    # One tick of each kind, by differencing two runs that differ by exactly
    # one tick. Warm 401 puts the measured tick between release boundaries.
    rebase = counts[("401", "1")] - counts[("401", "0")]
    common = counts[("401", "0")] - counts[("400", "0")]
    row.notes.append(f"common tick {common:,} executed instructions")
    return row.resolve(
        rebase <= REBASE_CEILING, f"re-base {rebase:,} executed instructions"
    )


def check_call_cardinality(
    options: argparse.Namespace, repository: Path, work: Path
) -> Row:
    row = Row("record-call-cardinality", "3 call owners", "1 call owner")
    emitted = work / "record-call-cardinality.dae.json"
    status, log, _ = compile_model(
        options,
        repository,
        "tools/rumoca-repros/record-call-cardinality/RecordCallCardinality.mo",
        "RecordCallCardinality.Outer",
        emitted,
        ("--emit", "dae-json"),
    )
    if status != 0:
        (work / "record-call-cardinality.log").write_text(log, encoding="utf-8")
        return row.resolve(False, "did not lower; see record-call-cardinality.log")
    storage = json.loads(emitted.read_text(encoding="utf-8"))["storage"]
    wanted = [
        index
        for index, function in enumerate(storage["functions"])
        if function["name"].endswith("expensive")
    ]
    owners = sum(
        1
        for node in storage["expressions"]["nodes"]
        if isinstance(node, dict)
        and "call" in node
        and node["call"]["function"] in wanted
    )
    return row.resolve(owners == 1, f"{owners} call owner(s)")


def report(rows: Sequence[Row]) -> int:
    headers = ("row", "verdict", "measured", f"pin {PIN}", "required after the fix")
    table = [headers] + [
        (row.identifier, row.verdict, row.measured, row.expected_pin, row.expected_fixed)
        for row in rows
    ]
    widths = [max(len(line[column]) for line in table) for column in range(len(headers))]
    print()
    for index, line in enumerate(table):
        print("  ".join(field.ljust(widths[column]) for column, field in enumerate(line)).rstrip())
        if index == 0:
            print("  ".join("-" * width for width in widths))
    for row in rows:
        for note in row.notes:
            print(f"    {row.identifier}: {note}")
    failed = [row for row in rows if row.verdict == "FAIL"]
    unmeasured = [row for row in rows if row.verdict == "UNMEASURED"]
    print()
    print(
        f"rumoca-acceptance: {len(rows) - len(failed) - len(unmeasured)} passed, "
        f"{len(failed)} failed, {len(unmeasured)} unmeasured"
    )
    if unmeasured:
        print(
            "rumoca-acceptance: an unmeasured row compared nothing and is not a pass"
        )
    return 1 if failed or unmeasured else 0


if __name__ == "__main__":
    sys.exit(main())
