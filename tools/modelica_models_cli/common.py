"""Shared, cross-platform process and repository helpers."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
from typing import Sequence


class ToolError(RuntimeError):
    """An expected command failure with a concise user-facing message."""


def repository_root(start: Path | None = None) -> Path:
    configured = os.environ.get("MODELICA_MODELS_ROOT")
    candidates: list[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())
    location = (start or Path.cwd()).resolve()
    candidates.extend((location, *location.parents))
    for candidate in candidates:
        if (
            (candidate / "Tests/package.mo").is_file()
            and (candidate / "Vehicles/package.mo").is_file()
            and (candidate / "README.md").is_file()
        ):
            return candidate.resolve()
    raise ToolError(f"could not find the modelica_models checkout above {location}")


def executable(environment_name: str, fallback: str) -> str:
    return os.environ.get(environment_name) or fallback


def executable_path(program: str) -> Path | None:
    candidate = Path(program)
    if candidate.parent != Path("."):
        return candidate.resolve() if candidate.is_file() else None
    found = shutil.which(program)
    return Path(found).resolve() if found else None


def require_executable(environment_name: str, fallback: str, label: str) -> Path:
    program = executable(environment_name, fallback)
    path = executable_path(program)
    if path is None:
        raise ToolError(
            f"{label} executable not found; install it or set {environment_name}"
        )
    return path


def repository_environment(root: Path) -> dict[str, str]:
    environment = os.environ.copy()
    environment["MODELICA_MODELS_ROOT"] = str(root)
    for name in ("MODELICAPATH", "OPENMODELICALIBRARY"):
        inherited = environment.get(name)
        environment[name] = os.pathsep.join(
            part for part in (str(root), inherited) if part
        )
    return environment


def run(
    command: Sequence[str | os.PathLike[str]],
    root: Path,
    *,
    environment: dict[str, str] | None = None,
) -> None:
    subprocess.run(
        [str(argument) for argument in command],
        cwd=root,
        env=environment,
        check=True,
    )
