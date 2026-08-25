"""Validate the standards-facing Modelica package structure."""

from __future__ import annotations

from pathlib import Path

from .common import ToolError


def modelica_files(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.rglob("*.mo")
        if ".git" not in path.parts
        and not path.is_relative_to(root / "tools" / "rumoca-repros")
    )


def check(root: Path) -> None:
    errors: list[str] = []
    packages = sorted(
        path for path in root.rglob("package.mo") if ".git" not in path.parts
    )
    for package_file in packages:
        directory = package_file.parent
        children = sorted(
            path for path in directory.glob("*.mo") if path.name != "package.mo"
        )
        order_file = directory / "package.order"
        if children and not order_file.is_file():
            errors.append(
                f"{directory.relative_to(root)} has {len(children)} child classes "
                "but no package.order"
            )

    order_files = sorted(
        path for path in root.rglob("package.order") if ".git" not in path.parts
    )
    for order_file in order_files:
        entries = order_file.read_text(encoding="utf-8").splitlines()
        seen: set[str] = set()
        duplicates: list[str] = []
        for entry in entries:
            if entry in seen and entry not in duplicates:
                duplicates.append(entry)
            seen.add(entry)
        relative_order = order_file.relative_to(root)
        if duplicates:
            errors.append(
                f"{relative_order} contains duplicate entries: "
                + ", ".join(duplicates)
            )
        for child in sorted(
            path
            for path in order_file.parent.glob("*.mo")
            if path.name != "package.mo"
        ):
            if child.stem not in entries:
                errors.append(f"{relative_order} does not list {child.stem}")

    for modelica_file in modelica_files(root):
        relative = modelica_file.relative_to(root)
        directory = relative.parent
        expected_directory = directory.parent if modelica_file.name == "package.mo" else directory
        expected_package = ".".join(expected_directory.parts)
        lines = modelica_file.read_text(encoding="utf-8").splitlines()
        first_line = lines[0] if lines else ""
        actual = "".join(first_line.split())
        expected = "within;" if not expected_package else f"within{expected_package};"
        if actual != expected:
            display_expected = (
                "within;" if not expected_package else f"within {expected_package};"
            )
            errors.append(
                f"{relative} starts with {actual!r}; expected {display_expected!r}"
            )

    if errors:
        formatted = "\n".join(f"Modelica library check: {error}" for error in errors)
        raise ToolError(formatted)
    print("Modelica library structure check passed.")
