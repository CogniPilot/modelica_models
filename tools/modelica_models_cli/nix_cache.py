"""Maintain CI's targeted cache of the pinned Rumoca runtime outputs."""

from __future__ import annotations

import argparse
from pathlib import Path
import os
import subprocess
import sys
from typing import Sequence

from .common import ToolError, repository_root


OUTPUTS = ("rumoca-cli", "rumoca-python-runtime", "rumoca-runtime")


def capture(root: Path, *command: str) -> str:
    result = subprocess.run(
        command,
        cwd=root,
        stdout=subprocess.PIPE,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def output_paths(root: Path) -> dict[str, Path]:
    paths = {
        name: Path(capture(root, "nix", "eval", "--raw", f".#{name}.outPath"))
        for name in OUTPUTS
    }
    for path in paths.values():
        if not str(path).startswith("/nix/store/") or "-rumoca-" not in path.name:
            raise ToolError(f"unexpected Rumoca output path: {path}")
    return paths


def cache_key(root: Path) -> str:
    return output_paths(root)["rumoca-runtime"].name


def ensure(root: Path, cache_directory: Path) -> None:
    paths = output_paths(root)
    cli_path = paths["rumoca-cli"]
    python_path = paths["rumoca-python-runtime"]
    expected_path = paths["rumoca-runtime"]
    cache_directory.mkdir(parents=True, exist_ok=True)
    archive = cache_directory / "rumoca-runtime.export"
    manifest = cache_directory / "upstream-runtime-paths"

    if archive.is_file() and archive.stat().st_size:
        if not manifest.is_file() or not manifest.stat().st_size:
            raise ToolError("cached Rumoca runtime has no upstream-path manifest")
        print(f"Importing cached Rumoca runtime {expected_path}")
        upstream_paths = manifest.read_text(encoding="utf-8").splitlines()
        subprocess.run(
            ["nix-store", "--realise", *upstream_paths], cwd=root, check=True
        )
        with archive.open("rb") as stream:
            subprocess.run(["nix-store", "--import"], cwd=root, stdin=stream, check=True)
    else:
        print(f"No cached Rumoca runtime; building {expected_path}")
        built_path = Path(
            capture(root, "nix", "build", "--no-link", "--print-out-paths", ".#rumoca-runtime")
        )
        if built_path != expected_path:
            raise ToolError(f"built Rumoca path {built_path}, expected {expected_path}")
        requisites = capture(
            root, "nix-store", "--query", "--requisites", str(built_path)
        ).splitlines()
        excluded = {str(python_path), str(cli_path), str(built_path)}
        manifest.write_text(
            "".join(f"{path}\n" for path in requisites if path not in excluded),
            encoding="utf-8",
        )
        temporary = archive.with_suffix(".export.tmp")
        with temporary.open("wb") as stream:
            subprocess.run(
                ["nix-store", "--export", str(python_path), str(cli_path), str(built_path)],
                cwd=root,
                stdout=stream,
                check=True,
            )
        temporary.replace(archive)
        print(
            "Prepared targeted Rumoca runtime cache "
            f"(3 paths, {archive.stat().st_size} bytes)"
        )

    subprocess.run(["nix-store", "--verify-path", str(expected_path)], cwd=root, check=True)
    executable = expected_path / "bin" / "rumoca"
    if not executable.is_file():
        raise ToolError(f"cached Rumoca executable is missing: {executable}")
    subprocess.run([str(executable), "--version"], cwd=root, check=True)


def main(arguments: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    key_parser = subparsers.add_parser("key", help="print the runtime cache key")
    key_parser.add_argument(
        "--github-output",
        action="store_true",
        help="write key=... to GITHUB_OUTPUT",
    )
    ensure_parser = subparsers.add_parser("ensure", help="import or build the runtime")
    ensure_parser.add_argument("cache_directory", type=Path)
    options = parser.parse_args(arguments)
    try:
        root = repository_root()
        if options.command == "key":
            key = cache_key(root)
            if options.github_output:
                destination = os.environ.get("GITHUB_OUTPUT")
                if not destination:
                    raise ToolError("GITHUB_OUTPUT is unset")
                with Path(destination).open("a", encoding="utf-8") as stream:
                    stream.write(f"key={key}\n")
            else:
                print(key)
        else:
            ensure(root, options.cache_directory)
    except (OSError, subprocess.CalledProcessError, ToolError) as error:
        print(f"modelica-models nix-cache: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
