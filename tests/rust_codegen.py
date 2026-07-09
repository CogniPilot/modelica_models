from __future__ import annotations

import re


_TEMPORARY_RE = re.compile(r"^(\s*let __r\d+)( = .*)$")


def normalize_rumoca_rust(code: str) -> str:
    """Normalize small Rust syntax issues emitted by rumoca's rust-solve backend."""
    lines = [
        _annotate_temporary(_normalize_ternary_line(line))
        for line in code.splitlines()
    ]
    normalized = "\n".join(lines) + "\n"
    if " ? " in normalized:
        raise AssertionError("unhandled C-style ternary in generated Rust")
    return normalized


def _normalize_ternary_line(line: str) -> str:
    stripped = line.lstrip()
    if not (
        stripped.startswith("let ")
        and " = " in stripped
        and " ? " in stripped
        and " : " in stripped
        and stripped.rstrip().endswith(";")
    ):
        return line

    indent = line[: len(line) - len(stripped)]
    lhs, rhs = stripped.rsplit(" = ", 1)
    expr = rhs.rstrip().removesuffix(";").strip()
    if expr.startswith("(") and expr.endswith(")"):
        expr = expr[1:-1].strip()

    condition, rest = expr.split(" ? ", 1)
    value_if_true, value_if_false = rest.rsplit(" : ", 1)
    return (
        f"{indent}{lhs} = if {condition.strip()} "
        f"{{ {value_if_true.strip()} }} else {{ {value_if_false.strip()} }};"
    )


def _annotate_temporary(line: str) -> str:
    return _TEMPORARY_RE.sub(r"\1: f64\2", line)
