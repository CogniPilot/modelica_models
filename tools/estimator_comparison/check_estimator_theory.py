"""Recheck selected GNC estimation theorems and run its kernel axiom audit."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


MODULES = (
    "CovariancePropagation",
    "NoiseCoordinates",
    "BearingOutput",
    "BearingComparison",
    "InertialBias",
    "GroupAffine",
    "RemainderStability",
    "PredictionMoments",
    "KalmanCorrection",
    "InertialAidingObservability",
)


def check(args):
    if args.build.exists() or args.output.exists():
        raise ValueError("Choose new proof build and evidence paths")
    version = subprocess.check_output([str(args.lean), "--version"], text=True).strip()
    if "version 4.29.1," not in version:
        raise ValueError("Use the GNC pinned Lean 4.29.1 release")
    args.build.mkdir(parents=True)
    libraries = args.build / "lib/lean"
    libraries.mkdir(parents=True)
    cached_project = args.cache / "build/lib/lean"
    for path in cached_project.rglob("*"):
        if path.is_file():
            target = libraries / path.relative_to(cached_project)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(path)
    paths = [
        libraries,
        args.cache / "build/lib/lean",
        *sorted((args.cache / "packages").glob("*/.lake/build/lib/lean")),
    ]
    environment = dict(os.environ, LEAN_PATH=":".join(map(str, paths)))
    commands, sources = [], {}
    for module in MODULES:
        relative = Path("GNC/Estimation") / (module + ".lean")
        source = args.gnc / relative
        destination = libraries / relative.with_suffix(".olean")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            destination.unlink()
        sources[str(relative)] = hashlib.sha256(source.read_bytes()).hexdigest()
        command = [str(args.lean), str(relative), "-o", str(destination)]
        commands.append(command)
        completed = subprocess.run(
            command, cwd=args.gnc, env=environment, text=True, capture_output=True
        )
        (args.build / (module + ".log")).write_text(completed.stdout + completed.stderr)
        if completed.returncode:
            raise ValueError(
                f"Proof failed: {module}: {completed.stdout}{completed.stderr}"
            )
    audit = args.build / "Audit.lean"
    audit.write_text(
        "\n".join("import GNC.Estimation." + name for name in MODULES)
        + "\nimport Verification.Policy\nrun_cmd Verification.Policy.emitJson\n"
    )
    completed = subprocess.run(
        [str(args.lean), str(audit)],
        cwd=args.gnc,
        env=environment,
        text=True,
        capture_output=True,
        check=True,
    )
    (args.build / "audit.log").write_text(completed.stdout + completed.stderr)
    policy = json.loads(completed.stdout)
    assert set(policy["axioms"]) <= {"propext", "Classical.choice", "Quot.sound"}
    report = dict(
        lean=version,
        modules=list(MODULES),
        source_sha256=sources,
        proof_sha256={
            str(p.relative_to(libraries)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(libraries.rglob("*.olean"))
            if not p.is_symlink()
        },
        audit=policy,
        commands=commands,
        limitations=[
            "Targeted estimation-module check, not a full GNC release build.",
            "Other dependencies reuse the existing checked GNC/mathlib cache.",
            "Exact-real mathematical identities; no Modelica-to-Lean refinement proof or universal firmware-performance theorem.",
        ],
    )
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            dict(
                modules=len(MODULES),
                axioms=policy["axioms"],
                declarations=policy["project_declarations"],
            )
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("gnc", "cache", "lean", "build", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    for name in ("gnc", "cache", "lean", "build", "output"):
        setattr(args, name, getattr(args, name).resolve())
    check(args)
