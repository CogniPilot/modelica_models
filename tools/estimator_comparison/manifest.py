#!/usr/bin/env python3
"""Hash reviewed captures and source files without embedding local paths."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def digest(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def hashes(root, paths):
    return {str(p.relative_to(root)): digest(p) for p in sorted(paths) if p.is_file()}


def manifest(args):
    repo = Path(__file__).resolve().parents[2]
    sources = [
        p
        for pattern in [
            "**/*.mo",
            "tools/estimator_comparison/*.py",
            "tools/estimator_comparison/*.c",
            "tools/ci.py",
            "flake.nix",
            "flake.lock",
        ]
        for p in repo.glob(pattern)
        if not any(x in p.parts for x in ["artifacts", "dev", ".git"])
    ]
    native = args.native_repo.resolve()
    result = dict(
        date="2026-10-07",
        rumoca_release="v0.10.2",
        rumoca_commit="276e82bd0fd48960ac201e782918d28bf19cf7ce",
        px4_commit="f1c0a1f794edf8e5e974b6ed96df3f95eda0df39",
        ardupilot_commit="1511f27194f1dcc3728270883047bdf022b3fd53",
        native_harness_commit=subprocess.check_output(
            ["git", "-C", str(native), "rev-parse", "HEAD"], text=True
        ).strip(),
        modelica_base_commit=subprocess.check_output(
            ["git", "-C", str(repo), "merge-base", "HEAD", "origin/main"], text=True
        ).strip(),
        note="Source SHA-256 hashes identify the reviewed implementation, including changes beyond base commit. Native toolchain/environment defaults may vary across platforms.",
        source_sha256=hashes(repo, sources),
        native_harness_sha256=hashes(native, native.glob("harness/**/*")),
        capture_sha256=hashes(
            args.data_root,
            [
                p
                for p in args.data_root.glob("*/*")
                if p.parent.name
                in [
                    f"{prefix}{speed}_{seed}"
                    for prefix in ["", "lower_"]
                    for speed in [0.25, 0.6]
                    for seed in [7, 19, 41]
                ]
            ],
        ),
        high_output_sha256=hashes(args.results, args.results.glob("*.csv")),
        lower_output_sha256=hashes(
            args.lower_results, args.lower_results.glob("*.csv")
        ),
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    for flag in ["native-repo", "data-root", "results", "lower-results", "output"]:
        p.add_argument("--" + flag, type=Path, required=True)
    manifest(p.parse_args())
