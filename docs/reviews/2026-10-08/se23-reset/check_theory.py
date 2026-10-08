"""Check the SE3 Taylor remainder bounds with the pinned GNC Lean cache."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(args):
    if args.build.exists() or args.output.exists():
        raise ValueError("Use fresh owned build and evidence paths")
    version = subprocess.check_output([str(args.lean), "--version"], text=True).strip()
    if "version 4.29.1," not in version:
        raise ValueError("Use GNC's pinned Lean 4.29.1")
    source = Path(__file__).with_name("ResetSeries.lean")
    paths = [
        args.cache / "build/lib/lean",
        *sorted((args.cache / "packages").glob("*/.lake/build/lib/lean")),
    ]
    args.build.mkdir(parents=True)
    artifact = args.build / "ResetSeries.olean"
    command = [str(args.lean), str(source), "-o", str(artifact)]
    completed = subprocess.run(
        command,
        cwd=source.parent,
        env=dict(os.environ, LEAN_PATH=":".join(map(str, paths))),
        text=True,
        capture_output=True,
    )
    log = completed.stdout + completed.stderr
    (args.build / "lean.log").write_text(log)
    if completed.returncode:
        raise ValueError(log)
    audit = re.findall(r"'([^']+)' depends on axioms: \[([^]]*)\]", log)
    expected = re.findall(r"^#print axioms (\S+)$", source.read_text(), re.MULTILINE)
    if len(audit) != 7 or sorted(name for name, _ in audit) != sorted(expected):
        raise ValueError("Missing theorem axiom audit")
    axioms = {name: [a.strip() for a in used.split(",")] for name, used in audit}
    if any(
        set(used) - {"propext", "Classical.choice", "Quot.sound"}
        for used in axioms.values()
    ):
        raise ValueError("Unapproved theorem axiom")
    imported = args.cache / "build/lib/lean/GNC/Analysis/TrigonometricPolynomial.olean"
    result = dict(
        lean=version,
        command=command,
        source_sha256=digest(source),
        checker_sha256=digest(Path(__file__)),
        artifact_sha256=digest(artifact),
        imported_gnc_taylor_sha256=digest(imported),
        audit=axioms,
        scope="Exact-real error bounds for all five SE3 Taylor coefficients over positive angles up to 0.2 rad, and branch coverage for ESKF corrections through 0.15 rad. Uses GNC Taylor theorem and mathlib trigonometric derivative bounds. No floating-point refinement, complete filter boundedness or EKF2/EKF3 superiority theorem.",
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            dict(theorems=len(audit), axioms=sorted(set(sum(axioms.values(), []))))
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cache", "lean", "build", "output"):
        parser.add_argument(
            "--" + name, type=lambda p: Path(p).resolve(), required=True
        )
    check(parser.parse_args())
