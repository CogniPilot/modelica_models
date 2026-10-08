"""Recheck GNC sampled-error bounds and audit every delivered theorem."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


MODULES = {
    "SampledErrorBound": 21,
    "GpsDeniedGeometry": 4,
    "LieErrorEnvelope": 13,
    "CorrectionErrorBound": 4,
}
DEPENDENCIES = (
    "GNC.Analysis.LinearODE",
    "GNC.Analysis.TaylorCertificate",
    "GNC.Estimation.RemainderStability",
    "GNC.Estimation.InertialBias",
    "GNC.Dynamics.LieErrorReconstruction",
)
ALLOWED_AXIOMS = {"propext", "Classical.choice", "Quot.sound"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_audit(log, expected):
    entries = re.findall(r"'([^']+)' depends on axioms: \[([^]]*)\]", log)
    if sorted(name for name, _ in entries) != sorted(expected):
        raise ValueError("Missing, duplicated or unexpected theorem audit")
    audited = {
        name: [axiom.strip() for axiom in axioms.split(",") if axiom.strip()]
        for name, axioms in entries
    }
    if any(set(axioms) - ALLOWED_AXIOMS for axioms in audited.values()):
        raise ValueError("Unapproved theorem axiom")
    return audited


def check(args):
    if args.build.exists() or args.output.exists():
        raise ValueError("Use fresh owned build and receipt paths")
    version = subprocess.check_output([str(args.lean), "--version"], text=True).strip()
    if "version 4.29.1," not in version:
        raise ValueError("Use GNC's pinned Lean 4.29.1")
    libraries = args.build / "lib/lean"
    libraries.mkdir(parents=True)
    cached = args.cache / "build/lib/lean"
    for path in cached.rglob("*"):
        if path.is_file():
            target = libraries / path.relative_to(cached)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(path)
    paths = [
        libraries,
        *sorted((args.cache / "packages").glob("*/.lake/build/lib/lean")),
    ]
    environment = dict(os.environ, LEAN_PATH=":".join(map(str, paths)))
    commands, sources, artifacts, expected = [], {}, {}, []

    def compile_module(module, root):
        relative = Path(*module.split(".")).with_suffix(".lean")
        source = root / relative
        artifact = libraries / relative.with_suffix(".olean")
        artifact.parent.mkdir(parents=True, exist_ok=True)
        if artifact.is_symlink():
            artifact.unlink()
        sources[module] = digest(source)
        command = [
            str(args.lean),
            "--threads=1",
            "--tstack=65536",
            str(relative),
            "-o",
            str(artifact),
        ]
        commands.append(dict(command=command, cwd=str(root)))
        result = subprocess.run(
            command, cwd=root, env=environment, text=True, capture_output=True
        )
        log = result.stdout + result.stderr
        (args.build / (module + ".log")).write_text(log)
        if result.returncode:
            raise ValueError(f"Proof failed: {module}: {log}")
        if digest(source) != sources[module]:
            raise ValueError(f"Source changed during verification: {module}")
        artifacts[module] = digest(artifact)
        return source

    for dependency in DEPENDENCIES:
        compile_module(dependency, args.gnc)
    for name, count in MODULES.items():
        module = "GNC.Estimation." + name
        source = compile_module(module, args.gnc)
        declarations = re.findall(r"^theorem\s+(\S+)", source.read_text(), re.MULTILINE)
        if len(declarations) != count:
            raise ValueError(f"Unexpected public theorem inventory: {module}")
        expected.extend(module + "." + name for name in declarations)
    audit_source = args.build / "Audit.lean"
    audit_source.write_text(
        "\n".join("import GNC.Estimation." + name for name in MODULES)
        + "\n"
        + "\n".join("#print axioms " + name for name in expected)
        + "\n"
    )
    command = [str(args.lean), "--threads=1", str(audit_source)]
    result = subprocess.run(command, env=environment, text=True, capture_output=True)
    log = result.stdout + result.stderr
    (args.build / "audit.log").write_text(log)
    if result.returncode:
        raise ValueError(log)
    audited = parse_audit(log, expected)
    receipt = dict(
        lean=version,
        gnc_source_head=subprocess.check_output(
            ["git", "-C", str(args.gnc), "rev-parse", "HEAD"], text=True
        ).strip(),
        checker_sha256=digest(Path(__file__)),
        source_sha256=sources,
        artifact_sha256=artifacts,
        audit_source_sha256=digest(audit_source),
        commands=commands + [dict(command=command, cwd=str(Path.cwd()))],
        audit=audited,
        complete=True,
        scope=(
            "Conditional sampled nonlinear error tubes, finite GPS outages, GPS-return "
            "recovery, publication prediction, exact-real SE23 navigation log envelopes "
            "with explicit nonlinear bias forcing, adaptive-gain correction/reset/implementation defects, coordinate conversion and ideal "
            "GPS-denied horizontal observability. Comparison theorems distinguish ordered "
            "upper envelopes from uniform-upper/rival-witness worst-case separation. "
            "No certified numeric ESKF/EKF2/EKF3 map constants, compiler refinement, "
            "Gaussian probability coverage or native estimator superiority."
        ),
        dependency_policy=(
            "Five direct GNC dependencies rechecked from source into an owned overlay; "
            "other GNC/mathlib dependencies reuse checked cache objects. Every delivered "
            "theorem is audited transitively for axioms. This is not a full GNC build."
        ),
    )
    args.output.write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(dict(theorems=len(audited), axioms=sorted(ALLOWED_AXIOMS))))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("gnc", "cache", "lean", "build", "output"):
        parser.add_argument(
            "--" + name, type=lambda p: Path(p).resolve(), required=True
        )
    check(parser.parse_args())
