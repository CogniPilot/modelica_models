"""Export, compile and independently qualify the shared visual aiding primitives."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def qualify(args):
    repository = Path(__file__).resolve().parents[2]
    work = args.work.resolve()
    if work.is_relative_to(repository) or work.exists():
        raise ValueError("Choose a fresh build directory outside the source checkout")
    work.mkdir(parents=True)
    (work / "tmp").mkdir()
    environment = dict(
        os.environ,
        TMPDIR=str(work / "tmp"),
        OPENBLAS_NUM_THREADS="1",
        PYTHONDONTWRITEBYTECODE="1",
    )
    sources = [
        *repository.glob("Estimation/StrapdownINS/ESKF/**/*.mo"),
        *repository.glob("LieGroups/**/*.mo"),
        *repository.glob("LinearAlgebra/**/*.mo"),
        *repository.glob("SLAM/Fusion/*.mo"),
        *repository.glob("SLAM/Simulation/*.mo"),
    ]
    sources += [
        repository / path
        for path in (
            "Estimation/package.mo",
            "Estimation/StrapdownINS/package.mo",
            "Tests/package.mo",
            "SLAM/package.mo",
            "Tests/VisualTangentReplay.mo",
            "Tests/VisualLandmarkReplay.mo",
            "Tests/VisualCouplingReplay.mo",
            "Tests/SyntheticLandmarkReplay.mo",
            "tools/estimator_comparison/visual_tangent.c",
            "tools/estimator_comparison/visual_landmark.c",
            "tools/estimator_comparison/visual_coupling.c",
            "tools/estimator_comparison/visual_synthetic.c",
            "tools/estimator_comparison/check_visual_tangent.py",
            "tools/estimator_comparison/check_visual_landmark.py",
            "tools/estimator_comparison/check_visual_coupling.py",
            "tools/estimator_comparison/check_visual_synthetic.py",
            "tools/estimator_comparison/qualify_visual.py",
        )
    ]
    source_hashes = {
        str(path.relative_to(repository)): digest(path) for path in sorted(sources)
    }
    receipt = dict(complete=False, source_sha256=source_hashes, commands=[], results={})
    receipt_path = work / "qualification.json"

    def execute(command, log):
        receipt["commands"].append(command)
        receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
        with log.open("w") as output:
            subprocess.run(
                command,
                cwd=repository,
                env=environment,
                stdout=output,
                stderr=subprocess.STDOUT,
                check=True,
            )

    for kind, model in (
        ("tangent", "VisualTangentReplay"),
        ("landmark", "VisualLandmarkReplay"),
        ("coupling", "VisualCouplingReplay"),
        ("synthetic", "SyntheticLandmarkReplay"),
    ):
        export = work / f"{kind}-export"
        execute(
            [
                str(args.rumoca),
                "compile",
                f"Tests/{model}.mo",
                "--model",
                f"Tests.{model}",
                "--source-root",
                str(repository),
                "--target",
                "galec-production",
                "--output",
                str(export),
                "--cache-dir",
                str(work / "rumoca-cache"),
            ],
            work / f"{kind}-export.log",
        )
        production = export / f"Tests_{model}" / "ProductionCode"
        executable = work / f"{kind}-probe"
        execute(
            [
                str(args.cc),
                "-O2",
                "-std=c11",
                "-Wall",
                "-Wextra",
                f"-I{production}",
                str(repository / f"tools/estimator_comparison/visual_{kind}.c"),
                str(production / f"Tests_{model}.c"),
                str(production / "rumoca_galec_kernels.c"),
                "-lm",
                "-o",
                str(executable),
            ],
            work / f"{kind}-build.log",
        )
        result = work / f"{kind}-result.json"
        execute(
            [
                sys.executable,
                str(repository / f"tools/estimator_comparison/check_visual_{kind}.py"),
                "--executable",
                str(executable),
                "--output",
                str(result),
            ],
            work / f"{kind}-check.log",
        )
        receipt["results"][kind] = json.loads(result.read_text())
        receipt.setdefault("generated_sha256", {}).update(
            {
                str(path.relative_to(work)): digest(path)
                for path in sorted(production.glob("*"))
                if path.is_file()
            }
        )
    if any(
        digest(repository / path) != expected
        for path, expected in source_hashes.items()
    ):
        raise ValueError("Source changed during qualification")
    receipt.update(
        complete=True,
        passed=all(value["passed"] for value in receipt["results"].values()),
        scope="Generated visual primitives, synthetic landmark camera and linearized sufficient-statistic comparison; not end-to-end SLAM.",
    )
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    print(
        json.dumps(
            {
                "complete": receipt["complete"],
                "passed": receipt["passed"],
                "receipt": str(receipt_path),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rumoca", type=Path, required=True)
    parser.add_argument("--cc", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    qualify(parser.parse_args())
