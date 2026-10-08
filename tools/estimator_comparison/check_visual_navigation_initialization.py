"""Check generated temporary-initialization sensitivity against frozen replays."""

import argparse
import json
import os
from pathlib import Path
import subprocess

import numpy as np

from check_visual_navigation import Replay, digest


def check(args):
    repository = Path(__file__).resolve().parents[2]
    qualification = args.qualification.resolve()
    work = args.work.resolve()
    if work.exists() or work.is_relative_to(repository):
        raise ValueError("Choose a fresh external verification directory")
    receipt = json.loads((qualification / "qualification.json").read_text())
    result = json.loads((qualification / "result.json").read_text())
    if not receipt["complete"] or not receipt["passed"] or not result["passed"]:
        raise ValueError("A complete passing reference qualification is required")
    if digest(qualification / "result.json") != receipt["result_sha256"]:
        raise ValueError("Reference result changed")
    for path, expected in receipt["source_sha256"].items():
        if digest(repository / path) != expected:
            raise ValueError(f"Reference source changed: {path}")
    work.mkdir(parents=True)
    (work / "tmp").mkdir()
    production = qualification / "export/Tests_VisualNavigationReplay/ProductionCode"
    for path, expected in receipt["generated_sha256"].items():
        if digest(qualification / path) != expected:
            raise ValueError(f"Generated source changed: {path}")
    report = dict(
        complete=False,
        passed=False,
        builds=[],
        qualification_sha256=digest(qualification / "qualification.json"),
        checker_sha256=digest(Path(__file__)),
        compiler_sha256=digest(args.cc),
        scope="Exact output comparison on the frozen captures under zero and pattern automatic-variable initialization; not a general proof of absence of undefined behavior.",
    )
    destination = work / "result.json"
    environment = dict(os.environ, TMPDIR=str(work / "tmp"))
    for initialization in ("zero", "pattern"):
        library = work / f"visual-navigation-{initialization}.so"
        command = [
            str(args.cc),
            "-O2",
            "-std=c11",
            "-shared",
            "-fPIC",
            f"-ftrivial-auto-var-init={initialization}",
            f"-I{production}",
            str(repository / "tools/estimator_comparison/visual_navigation.c"),
            str(production / "Tests_VisualNavigationReplay.c"),
            str(production / "rumoca_galec_kernels.c"),
            "-lm",
            "-o",
            str(library),
        ]
        build = dict(initialization=initialization, command=command, cases=[])
        report["builds"].append(build)
        destination.write_text(json.dumps(report, indent=2) + "\n")
        with (work / f"{initialization}-build.log").open("w") as log:
            subprocess.run(
                command,
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
        build["library_sha256"] = digest(library)
        replay = Replay(library)
        for case in result["cases"]:
            capture = qualification / "captures" / f"capture-{case['seed']}.npz"
            reference = (
                qualification / "replays" / f"{case['seed']}-{case['scenario']}.npz"
            )
            if (
                digest(capture) != case["capture_sha256"]
                or digest(reference) != case["replay_sha256"]
            ):
                raise ValueError("Frozen capture or replay changed")
            with np.load(capture) as saved:
                data = dict(saved)
            output, availability = replay.run(data, case["scenario"])
            with np.load(reference) as saved:
                same = (
                    output.tobytes() == saved["output"].tobytes()
                    and availability.tobytes() == saved["availability"].tobytes()
                )
            build["cases"].append(
                dict(
                    seed=case["seed"],
                    scenario=case["scenario"],
                    byte_identical=bool(same),
                )
            )
            destination.write_text(json.dumps(report, indent=2) + "\n")
        print(
            f"{initialization}: {sum(case['byte_identical'] for case in build['cases'])}/{len(build['cases'])} byte-identical paired replays",
            flush=True,
        )
    report.update(
        complete=True,
        passed=all(
            case["byte_identical"]
            for build in report["builds"]
            for case in build["cases"]
        ),
    )
    destination.write_text(json.dumps(report, indent=2) + "\n")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--cc", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    check(parser.parse_args())
