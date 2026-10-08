"""Build and replay the Modelica loose/tight fixed-map navigation experiment."""

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
    if work.exists() or work.is_relative_to(repository):
        raise ValueError("Choose a fresh build directory outside the source checkout")
    if len(set(args.seeds)) != len(args.seeds):
        raise ValueError("Independent seeds must be distinct")
    work.mkdir(parents=True)
    (work / "tmp").mkdir()
    environment = dict(
        os.environ,
        TMPDIR=str(work / "tmp"),
        OPENBLAS_NUM_THREADS="1",
        PYTHONDONTWRITEBYTECODE="1",
    )
    packages = ("Tests", "Estimation", "Avionics", "LieGroups", "LinearAlgebra", "SLAM")
    sources = [
        path for package in packages for path in (repository / package).rglob("*.mo")
    ]
    sources += [
        repository / "tools/estimator_comparison" / name
        for name in (
            "visual_navigation.c",
            "check_visual_navigation.py",
            "qualify_visual_navigation.py",
            "check_visual_coupling.py",
            "check_visual_tangent.py",
        )
    ]
    hashes = {
        str(path.relative_to(repository)): digest(path) for path in sorted(sources)
    }
    receipt_path = work / "qualification.json"
    receipt = dict(
        complete=False,
        passed=False,
        source_sha256=hashes,
        commands=[],
        tool_sha256={"rumoca": digest(args.rumoca), "cc": digest(args.cc)},
        declaration=dict(
            seeds=args.seeds,
            independent_noise_draws=len(args.seeds),
            duration_s=30,
            imu_hz=100,
            gps_hz=5,
            camera_hz=10,
            gps_scenarios=["gps", "denied", "transition"],
            gps_outage_s=[10, 20],
            camera_outage_s=[14, 16],
            map="Four exact world-referenced ground landmarks",
            image_size_pixels=[640, 480],
            visibility="Every generated noisy observation must have positive depth and lie inside the image",
            delay_s=0,
            bias_model="Constant random biases, zero random-walk spectral density",
            gates="Disabled; acceptance and absence of duplicate/unavailable fusion are checked",
            equivalence_nominal_absolute_tolerance=0.001,
            equivalence_covariance_absolute_tolerance=1e-5,
            consistency="Raw full15 covariance must be symmetric and Cholesky-positive on every row; NEES15 and dimension-specific NIS are reported without a statistical certification threshold",
            scope="Sequential fixed-known-map visual aiding; no live uncertain map, retained-reference correlations, feature association, camera rendering, transport delay, loop closure or native EKF comparison",
        ),
    )

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

    execute([str(args.rumoca), "--version"], work / "rumoca-version.log")
    if "0.10.2" not in (work / "rumoca-version.log").read_text():
        raise ValueError("This experiment requires the pinned Rumoca 0.10.2")
    execute(
        [
            str(args.rumoca),
            "compile",
            "Tests/VisualNavigationReplay.mo",
            "--model",
            "Tests.VisualNavigationReplay",
            "--source-root",
            str(repository),
            "--target",
            "galec-production",
            "--output",
            str(work / "export"),
            "--cache-dir",
            str(work / "rumoca-cache"),
        ],
        work / "export.log",
    )
    production = work / "export/Tests_VisualNavigationReplay/ProductionCode"
    library = work / "visual-navigation.so"
    execute(
        [
            str(args.cc),
            "-O2",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-shared",
            "-fPIC",
            f"-I{production}",
            str(repository / "tools/estimator_comparison/visual_navigation.c"),
            str(production / "Tests_VisualNavigationReplay.c"),
            str(production / "rumoca_galec_kernels.c"),
            "-lm",
            "-o",
            str(library),
        ],
        work / "build.log",
    )
    receipt["generated_sha256"] = {
        str(path.relative_to(work)): digest(path)
        for path in sorted(production.glob("*"))
        if path.is_file()
    }
    execute(
        [
            sys.executable,
            str(repository / "tools/estimator_comparison/check_visual_navigation.py"),
            "--library",
            str(library),
            "--output",
            str(work / "result.json"),
            "--seeds",
            *map(str, args.seeds),
        ],
        work / "replay.log",
    )
    if any(digest(repository / path) != expected for path, expected in hashes.items()):
        raise ValueError("Source changed during qualification")
    result = json.loads((work / "result.json").read_text())
    receipt.update(
        complete=True,
        passed=result["passed"],
        result_sha256=digest(work / "result.json"),
        library_sha256=digest(library),
    )
    receipt_path.write_text(json.dumps(receipt, indent=2) + "\n")
    print(
        json.dumps(
            dict(complete=True, passed=receipt["passed"], receipt=str(receipt_path)),
            indent=2,
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rumoca", type=Path, required=True)
    parser.add_argument("--cc", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument(
        "--seeds", type=int, nargs="+", default=list(range(20271041, 20271049))
    )
    qualify(parser.parse_args())
