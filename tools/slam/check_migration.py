"""Audit every migrated Modelica class against its pinned source tokens."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

from import_from_slam_web import TOKEN


def tokens(text):
    return [
        match.group()
        for match in TOKEN.finditer(text)
        if not match.group().startswith(("//", "/*"))
    ]


def check(args):
    repository = Path(__file__).resolve().parents[2]
    manifest = json.loads(args.manifest.read_text())
    original = {}
    for path, expected in manifest["source_sha256"].items():
        data = subprocess.check_output(
            [
                "git",
                "-C",
                str(args.source_repo),
                "show",
                f"{manifest['upstream_revision']}:{path}",
            ]
        )
        if hashlib.sha256(data).hexdigest() != expected:
            raise ValueError(f"Pinned source hash mismatch {path}")
        original[path] = data.decode()
    qualified = {
        entry["original_name"]: entry["qualified_name"] for entry in manifest["classes"]
    }
    for entry in manifest["classes"]:
        destination = repository / entry["destination"]
        data = destination.read_bytes()
        if hashlib.sha256(data).hexdigest() != entry["destination_sha256"]:
            raise ValueError(f"Migrated source hash mismatch {destination}")
        text = data.decode().split("\n", 1)[1]
        imports = re.findall(r"^  import (\w+) = ([\w.]+);\n", text, re.MULTILINE)
        if any(qualified[name] != target for name, target in imports):
            raise ValueError(f"Import target mismatch {destination}")
        restored = re.sub(r"^  import \w+ = [\w.]+;\n", "", text, flags=re.MULTILINE)
        if entry["source"].startswith("models/Examples/"):
            restored = restored.replace("SLAM.Examples.", "Examples.")
        actual_tokens = tokens(restored)
        source_tokens = tokens(original[entry["source"]])
        starts = [
            index
            for index in range(len(source_tokens) - len(actual_tokens) + 1)
            if source_tokens[index : index + len(actual_tokens)] == actual_tokens
        ]
        if len(starts) != 1:
            raise ValueError(f"Arithmetic/declaration token mismatch {destination}")
    package_count = 0
    for root in ("SLAM", "Vision"):
        for package in (repository / root).rglob("package.mo"):
            entries = (package.parent / "package.order").read_text().splitlines()
            children = {
                path.stem
                for path in package.parent.glob("*.mo")
                if path.name != "package.mo"
            }
            children.update(
                path.name
                for path in package.parent.iterdir()
                if path.is_dir() and (path / "package.mo").exists()
            )
            if set(entries) != children or len(entries) != len(set(entries)):
                raise ValueError(f"Package inventory mismatch {package}")
            package_count += 1
    result = dict(
        complete=True,
        passed=True,
        classes_checked=len(manifest["classes"]),
        source_files_checked=len(original),
        packages_checked=package_count,
        manifest_sha256=hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
        upstream_revision=manifest["upstream_revision"],
        scope="Pinned source hashes, every class's arithmetic/declaration tokens, import targets and package inventories; numerical and compiler qualification are separate.",
    )
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    check(parser.parse_args())
