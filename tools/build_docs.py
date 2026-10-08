#!/usr/bin/env python3
"""Publish embedded Modelica documentation without loading a compiler."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import html
from pathlib import Path
import re

TOKEN = re.compile(
    r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"|\'[^\']*\'|[A-Za-z_]\w*|\S', re.S
)
KINDS = {
    "package",
    "model",
    "block",
    "record",
    "connector",
    "function",
    "class",
    "type",
}
SOURCE_URL = "https://github.com/CogniPilot/modelica_models/blob/main/"


@dataclass
class Class:
    name: str
    kind: str
    path: Path
    start: int
    stop: int
    tokens: list[tuple[str, int, int]] = field(default_factory=list)
    info: str = ""
    description: str = ""


def decode_string(value: str) -> str:
    """Modelica escapes (do not reinterpret Unicode as Python byte escapes)."""
    escapes = {
        "n": "\n",
        "r": "\r",
        "t": "\t",
        "b": "\b",
        "f": "\f",
        "v": "\v",
        "a": "\a",
    }
    return re.sub(r"\\(.)", lambda m: escapes.get(m[1], m[1]), value[1:-1])


def parse(source: str, path: Path) -> list[Class]:
    tokens = [
        (m[0], m.start(), m.end())
        for m in TOKEN.finditer(source)
        if not m[0].startswith(("//", "/*"))
    ]
    within = ""
    if tokens and tokens[0][0] == "within":
        end = next(i for i, t in enumerate(tokens) if t[0] == ";")
        within = "".join(t[0] for t in tokens[1:end])
    result: list[Class] = []
    stack: list[Class] = []
    i = 0
    while i < len(tokens):
        value, start, stop = tokens[i]
        if (
            value in KINDS
            and i + 1 < len(tokens)
            and re.fullmatch(r"[A-Za-z_]\w*", tokens[i + 1][0])
        ):
            # Redeclarations inside component modifications are bindings,
            # not newly declared members. A class-extends body is a member.
            if i and tokens[i - 1][0] == "redeclare" and tokens[i + 1][0] != "extends":
                if stack:
                    stack[-1].tokens.append(tokens[i])
                i += 1
                continue
            name_index = i + 2 if tokens[i + 1][0] == "extends" else i + 1
            name = tokens[name_index][0]
            parent = stack[-1].name if stack else within
            item = Class(
                ".".join(filter(None, [parent, name])), value, path, start, len(source)
            )
            result.append(item)
            # Short class definitions have '=' after the name/description.
            j = name_index + 1
            while j < len(tokens) and tokens[j][0].startswith('"'):
                item.description += decode_string(tokens[j][0])
                j += 1
                if j < len(tokens) and tokens[j][0] == "+":
                    j += 1
                else:
                    break
            if j < len(tokens) and tokens[j][0] == "=":
                depth = 0
                while j < len(tokens):
                    token = tokens[j][0]
                    depth += token in ("(", "[", "{")
                    depth -= token in (")", "]", "}")
                    item.tokens.append(tokens[j])
                    if token == ";" and depth == 0:
                        item.stop = tokens[j][2]
                        break
                    j += 1
                i = j + 1
                continue
            stack.append(item)
            item.tokens.extend(tokens[i : name_index + 1])
            i = name_index + 1
            continue
        if stack:
            stack[-1].tokens.append(tokens[i])
            if (
                value == "end"
                and i + 1 < len(tokens)
                and tokens[i + 1][0] == stack[-1].name.split(".")[-1]
            ):
                item = stack.pop()
                item.stop = tokens[min(i + 2, len(tokens) - 1)][2]
                i += 3
                continue
        i += 1
    if stack:
        raise ValueError(f"{path}: unclosed class {stack[-1].name}")
    for item in result:
        own = item.tokens
        for j, token in enumerate(own):
            if token[0] != "Documentation" or j + 1 >= len(own) or own[j + 1][0] != "(":
                continue
            depth = 1
            k = j + 2
            while k < len(own) and depth:
                value = own[k][0]
                if (
                    depth == 1
                    and value == "info"
                    and k + 2 < len(own)
                    and own[k + 1][0] == "="
                ):
                    k += 2
                    parts = []
                    while k < len(own) and own[k][0].startswith('"'):
                        parts.append(decode_string(own[k][0]))
                        k += 1
                        if k < len(own) and own[k][0] == "+":
                            k += 1
                        else:
                            break
                    item.info = "".join(parts)
                    continue
                depth += value == "("
                depth -= value == ")"
                k += 1
    return result


def rewrite_links(info: str, names: set[str]) -> str:
    def replace(match: re.Match[str]) -> str:
        target = match[2]
        name, marker, anchor = target.partition("#")
        if name in names:
            return (
                match[1]
                + name
                + ".html"
                + (marker + anchor if marker else "")
                + match[3]
            )
        return match[0]

    info = re.sub(
        r"(?i)(href\s*=\s*[\"\'])modelica://([^\"\']+)([\"\'])", replace, info
    )
    return re.sub(r"</?html\b[^>]*>", "", info, flags=re.I)


STYLE = """body{font:16px/1.6 system-ui,sans-serif;margin:auto;max-width:1100px;padding:2rem;color:#172b3a;background:#fafcfe}a{color:#075da8}header{border-bottom:1px solid #ccd9e2;margin-bottom:2rem}nav{display:flex;flex-wrap:wrap;gap:.4rem 1rem}h1{overflow-wrap:anywhere;line-height:1.2}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#edf2f6;padding:1rem;font-size:.85rem}table{border-collapse:collapse;max-width:100%}td,th{padding:.4rem;border:1px solid #ccd9e2}li{margin:.25rem 0}.kind{color:#526575}input{font:inherit;padding:.5rem;width: min(90%,32rem)}"""


def build(root: Path, output: Path) -> int:
    classes = []
    sources = {}
    # Only root Modelica packages are public library trees; tools/dev fixtures
    # and build outputs must never enter the published API.
    for package in sorted(root.iterdir()):
        if package.name in {
            "dev",
            "tools",
            "artifacts",
            "target",
            "docs",
        } or package.name.startswith("."):
            continue
        if not package.is_dir() or not (package / "package.mo").is_file():
            continue
        for path in sorted(package.rglob("*.mo")):
            relative = path.relative_to(root)
            sources[relative] = path.read_text()
            classes.extend(parse(sources[relative], relative))
    names = {item.name for item in classes}
    if len(names) != len(classes):
        raise ValueError("Duplicate Modelica class names")
    output.mkdir(parents=True, exist_ok=True)
    (output / "style.css").write_text(STYLE)
    (output / ".nojekyll").touch()
    top = sorted(n for n in names if "." not in n)
    nav = (
        '<nav><a href="index.html">All classes</a>'
        + "".join(f'<a href="{n}.html">{n}</a>' for n in top)
        + "</nav>"
    )

    def page(title: str, body: str) -> str:
        return (
            '<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'
            + html.escape(title)
            + '</title><link rel="stylesheet" href="style.css"></head><body><header><p>CogniPilot Modelica library</p>'
            + nav
            + "</header><main>"
            + body
            + "</main></body></html>"
        )

    for item in classes:
        parent = item.name.rpartition(".")[0]
        body = f'<p class="kind">{item.kind}</p><h1>{item.name}</h1>'
        if item.description:
            body += "<p>" + html.escape(item.description) + "</p>"
        if parent in names:
            body += f'<p>Package: <a href="{parent}.html">{parent}</a></p>'
        line = sources[item.path].count("\n", 0, item.start) + 1
        body += f'<p><a href="{SOURCE_URL}{item.path.as_posix()}#L{line}">View source on GitHub</a></p>'
        body += (
            rewrite_links(item.info, names)
            if item.info
            else "<p>Browse the members and source declarations below.</p>"
        )
        children = sorted(n for n in names if n.rpartition(".")[0] == item.name)
        if children:
            body += (
                "<h2>Members</h2><ul>"
                + "".join(
                    f'<li><a href="{n}.html">{n.split(".")[-1]}</a></li>'
                    for n in children
                )
                + "</ul>"
            )
        body += (
            "<h2>Modelica declarations and implementation</h2><details><summary>Show parameters, interfaces and source</summary><p>Parameters, inputs, outputs, defaults and equations from the source file.</p><pre><code>"
            + html.escape(sources[item.path][item.start : item.stop])
            + "</code></pre></details>"
        )
        (output / (item.name + ".html")).write_text(page(item.name, body))
    by_name = {item.name: item for item in classes}
    packages = "".join(
        f'<li><a href="{n}.html">{n}</a> — {html.escape(by_name[n].description)}</li>'
        for n in top
    )
    entries = "".join(f'<li><a href="{n}.html">{n}</a></li>' for n in sorted(names))
    body = (
        "<h1>Modelica library reference</h1>"
        "<p>Explore reusable models through their embedded documentation and source declarations.</p>"
        "<h2>Packages</h2><ul>" + packages + "</ul>"
        "<h2>Find a class</h2><label>Package or class name "
        '<input id="search" type="search" placeholder="Start typing to search all classes"></label>'
        '<details id="index"><summary>Browse all classes</summary><ul id="classes">'
        + entries
        + "</ul></details>"
        '<script>const search=document.getElementById("search"),index=document.getElementById("index");'
        'search.addEventListener("input",e=>{const query=e.target.value.trim().toLowerCase();'
        'index.open=Boolean(query);for(const row of document.querySelectorAll("#classes li"))'
        "row.hidden=!row.textContent.toLowerCase().includes(query)})</script>"
    )
    (output / "index.html").write_text(page("Modelica library reference", body))
    return len(classes)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    parser.add_argument(
        "--output", type=Path, required=True, help="Dedicated generated site directory"
    )
    args = parser.parse_args()
    print(
        f"Published {build(args.root.resolve(), args.output.resolve())} Modelica classes to {args.output}"
    )


if __name__ == "__main__":
    main()
