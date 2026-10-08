#!/usr/bin/env python3
"""Publish embedded Modelica documentation without loading a compiler."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, replace
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


@dataclass
class Declaration:
    category: str
    name: str
    type_name: str
    details: str
    description: str


def split_top_level(tokens: list[tuple[str, int, int]], separator: str):
    """Split a token sequence without splitting arrays or modifications."""
    depth = 0
    group = []
    for token in tokens:
        value = token[0]
        if value == separator and depth == 0:
            yield group
            group = []
            continue
        group.append(token)
        depth += value in ("(", "[", "{")
        depth -= value in (")", "]", "}")
    if group:
        yield group


def api(
    item: Class, source: str, public_only: bool = True
) -> tuple[list[Declaration], list[str]]:
    """Extract explicit public components, never flatten or evaluate Modelica."""
    tokens = item.tokens
    if not tokens or tokens[0][0] == "=":
        return [], []
    start = 3 if len(tokens) > 1 and tokens[1][0] == "extends" else 2
    while start < len(tokens) and (
        tokens[start][0].startswith('"') or tokens[start][0] == "+"
    ):
        start += 1
    declarations = []
    bases = []
    public = True
    prefixes = {
        "parameter",
        "input",
        "output",
        "constant",
        "discrete",
        "flow",
        "stream",
        "final",
        "replaceable",
        "inner",
        "outer",
        "each",
    }
    for statement in split_top_level(tokens[start:], ";"):
        while statement and statement[0][0] in {"public", "protected"}:
            public = statement.pop(0)[0] == "public"
        if not statement:
            continue
        if statement[0][0] in {"equation", "algorithm", "initial", "end", "external"}:
            break
        if statement[0][0] == "extends":
            name = []
            for token in statement[1:]:
                if token[0] != "." and not re.fullmatch(r"[A-Za-z_]\w*", token[0]):
                    break
                name.append(token[0])
            if name:
                bases.append("".join(name))
            continue
        if public_only and not public:
            continue
        position = 0
        qualifiers = []
        while position < len(statement) and statement[position][0] in prefixes:
            qualifiers.append(statement[position][0])
            position += 1
        category = next(
            (
                q
                for q in ("parameter", "input", "output", "constant")
                if q in qualifiers
            ),
            "field",
        )
        if category == "field" and item.kind not in {"record", "connector"}:
            continue
        if position >= len(statement) or not re.fullmatch(
            r"[A-Za-z_]\w*", statement[position][0]
        ):
            continue
        if statement[position][0] in {"annotation", "import", "redeclare"}:
            continue
        type_start = position
        position += 1
        while position + 1 < len(statement) and statement[position][0] == ".":
            position += 2
        # Type-level dimensions belong to the type, not to the component name.
        if position < len(statement) and statement[position][0] == "[":
            depth = 1
            position += 1
            while position < len(statement) and depth:
                depth += statement[position][0] == "["
                depth -= statement[position][0] == "]"
                position += 1
        type_name = source[statement[type_start][1] : statement[position - 1][2]]
        for component in split_top_level(statement[position:], ","):
            if not component or not re.fullmatch(r"[A-Za-z_]\w*", component[0][0]):
                continue
            depth = 0
            description = []
            detail_stop = component[-1][2]
            for index, token in enumerate(component[1:], 1):
                if depth == 0 and token[0] == "annotation":
                    detail_stop = min(detail_stop, token[1])
                    break
                if (
                    depth == 0
                    and token[0].startswith('"')
                    and (
                        description
                        or component[index - 1][0]
                        not in {"=", "+", "-", "*", "/", "then", "else"}
                    )
                ):
                    detail_stop = min(detail_stop, token[1])
                    description.append(decode_string(token[0]))
                depth += token[0] in ("(", "[", "{")
                depth -= token[0] in (")", "]", "}")
            details = source[component[0][2] : detail_stop].strip()
            declarations.append(
                Declaration(
                    category, component[0][0], type_name, details, "".join(description)
                )
            )
    return declarations, bases


def resolve_name(name: str, owner: str, names: set[str]) -> str | None:
    scope = owner
    while scope:
        candidate = scope + "." + name
        if candidate in names:
            return candidate
        scope = scope.rpartition(".")[0]
    return name if name in names else None


def source_page_name(path: Path) -> str:
    return "source-" + ".".join(path.parts) + ".html"


KEYWORDS = KINDS | {
    "within",
    "end",
    "extends",
    "parameter",
    "input",
    "output",
    "protected",
    "public",
    "equation",
    "algorithm",
    "initial",
    "annotation",
    "redeclare",
    "replaceable",
    "partial",
    "encapsulated",
    "if",
    "then",
    "else",
    "elseif",
    "for",
    "loop",
    "while",
    "when",
    "elsewhen",
    "true",
    "false",
    "constant",
    "discrete",
    "connect",
    "external",
    "import",
    "return",
    "break",
}


def reference_links(
    source: str, path: Path, classes: list[Class], sources: dict[Path, str]
) -> dict[int, tuple[int, str]]:
    """Conservative lexical navigation, not compiler name resolution."""
    names = {item.name for item in classes}
    by_name = {item.name: item for item in classes}
    local = [item for item in classes if item.path == path]
    tokens = [
        (m[0], m.start(), m.end())
        for m in TOKEN.finditer(source)
        if not m[0].startswith(("//", "/*", '"'))
    ]
    context = {}
    for item in local:
        declarations, _ = api(replace(item, kind="record"), source, public_only=False)
        shadows = {declaration.name for declaration in declarations}
        for child in local:
            if child.name.rpartition(".")[0] == item.name and re.search(
                r"\breplaceable\s*$", source[: child.start]
            ):
                shadows.add(child.name.rsplit(".", 1)[-1])
        aliases = {}
        own = item.tokens
        for position, token in enumerate(own):
            if token[0] != "import":
                continue
            statement = []
            for following in own[position + 1 :]:
                if following[0] == ";":
                    break
                statement.append(following[0])
            if "=" in statement:
                equals = statement.index("=")
                if equals == 1:
                    aliases[statement[0]] = "".join(statement[2:])
            elif statement and "*" not in statement and "{" not in statement:
                aliases[statement[-1]] = "".join(statement)
        context[item.name] = (shadows, aliases)
    result = {}
    position = 0
    while position < len(tokens):
        value, start, stop = tokens[position]
        if not re.fullmatch(r"[A-Za-z_]\w*", value) or value in KEYWORDS:
            position += 1
            continue
        owner = min(
            (item for item in local if item.start <= start < item.stop),
            key=lambda item: item.stop - item.start,
            default=None,
        )
        end = position + 1
        parts = [value]
        while (
            end + 1 < len(tokens)
            and tokens[end][0] == "."
            and re.fullmatch(r"[A-Za-z_]\w*", tokens[end + 1][0])
        ):
            parts.append(tokens[end + 1][0])
            stop = tokens[end + 1][2]
            end += 2
        if owner:
            shadows = set()
            aliases = {}
            scope = owner.name
            while scope:
                scope_shadows, scope_aliases = context.get(scope, (set(), {}))
                shadows.update(scope_shadows)
                for alias, target in scope_aliases.items():
                    aliases.setdefault(alias, target)
                scope = scope.rpartition(".")[0]
            if parts[0] not in shadows:
                candidate = ".".join(parts)
                if parts[0] in aliases:
                    candidate = ".".join([aliases[parts[0]], *parts[1:]])
                target = resolve_name(candidate, owner.name, names)
                if target:
                    definition = by_name[target]
                    line = sources[definition.path].count("\n", 0, definition.start) + 1
                    result[start] = (
                        stop,
                        source_page_name(definition.path) + f"#L{line}",
                    )
        position = end
    return result


def highlighted_source(
    source: str, links: dict[int, tuple[int, str]] | None = None
) -> str:
    """Preserve text exactly; line numbers are CSS content, never source text."""
    links = links or {}
    fragments = []
    cursor = 0
    for match in TOKEN.finditer(source):
        if match.start() < cursor:
            continue
        fragments.append(html.escape(source[cursor : match.start()]))
        if match.start() in links:
            stop, target = links[match.start()]
            value = html.escape(source[match.start() : stop])
            fragments.append(
                "\n".join(
                    f'<a class="definition" href="{target}" title="Go to definition">{part}</a>'
                    for part in value.split("\n")
                )
            )
            cursor = stop
            continue
        value = match[0]
        style = (
            "comment"
            if value.startswith(("//", "/*"))
            else "string"
            if value.startswith('"')
            else "keyword"
            if value in KEYWORDS
            else ""
        )
        escaped = html.escape(value)
        if style:
            escaped = "\n".join(
                f'<span class="{style}">{part}</span>' for part in escaped.split("\n")
            )
        fragments.append(escaped)
        cursor = match.end()
    fragments.append(html.escape(source[cursor:]))
    return "\n".join(
        f'<span class="source-line" id="L{number}"><a class="line-number" href="#L{number}" data-line="{number}" aria-label="Line {number}"></a>{line}</span>'
        for number, line in enumerate("".join(fragments).split("\n"), 1)
    )


def member_order(item: Class, children: list[Class], root: Path) -> list[Class]:
    order_file = root / item.path.parent / "package.order"
    order = (
        order_file.read_text().splitlines()
        if item.path.name == "package.mo" and order_file.exists()
        else []
    )
    rank = {name.strip(): i for i, name in enumerate(order)}
    return sorted(
        children,
        key=lambda child: (
            rank.get(child.name.rsplit(".", 1)[-1], len(rank)),
            child.start if child.path == item.path else len(rank),
            child.name,
        ),
    )


def api_table(declarations: list[Declaration], owner: str, names: set[str]) -> str:
    if not declarations:
        return ""
    rows = []
    for declaration in declarations:
        type_name = html.escape(declaration.type_name)
        target = resolve_name(declaration.type_name, owner, names)
        if target:
            type_name = f'<a href="{target}.html">{type_name}</a>'
        rows.append(
            "<tr>"
            f"<td>{declaration.category}</td><td><code>{html.escape(declaration.name)}</code></td>"
            f"<td><code>{type_name}</code></td><td><code>{html.escape(declaration.details)}</code></td>"
            f"<td>{html.escape(declaration.description)}</td></tr>"
        )
    return (
        "<h2>Public declarations</h2><p>Explicit source declarations; dimensions, modifications "
        'and defaults are shown as written, without evaluation.</p><div class="table-scroll"><table>'
        "<thead><tr><th>Role</th><th>Name</th><th>Type</th><th>Dimensions / attributes / default</th>"
        "<th>Description</th></tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table></div>"
    )


def build(root: Path, output: Path) -> int:
    classes = []
    sources = {}
    raw_sources = {}
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
            raw_sources[relative] = path.read_bytes()
            sources[relative] = raw_sources[relative].decode("utf-8")
            classes.extend(parse(sources[relative], relative))
    names = {item.name for item in classes}
    if len(names) != len(classes):
        raise ValueError("Duplicate Modelica class names")
    output.mkdir(parents=True, exist_ok=True)
    (output / "style.css").write_bytes(
        (Path(__file__).parent / "docs_assets/reference.css").read_bytes()
    )
    assets = Path(__file__).parent / "docs_assets"
    if (assets / "cognipilot-logo.svg").is_file():
        (output / "cognipilot-logo.svg").write_bytes(
            (assets / "cognipilot-logo.svg").read_bytes()
        )
    (output / "reference.js").write_bytes((assets / "reference.js").read_bytes())
    (output / ".nojekyll").touch()
    credits = []
    for filename, label in (("LICENSE", "License"), ("NOTICE", "Credits")):
        if (root / filename).is_file():
            (output / filename).write_bytes((root / filename).read_bytes())
            credits.append(f'<a href="{filename}">{label}</a>')
    footer = "<footer>CogniPilot · Modelica reference"
    if credits:
        footer += " · " + " · ".join(credits)
    footer += "</footer>"
    top = sorted(n for n in names if "." not in n)
    nav = (
        '<nav aria-label="Packages"><a href="index.html">Overview</a>'
        + "".join(f'<a href="{n}.html">{n}</a>' for n in top)
        + "</nav>"
    )

    def page(title: str, body: str) -> str:
        return (
            '<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            "<title>" + html.escape(title) + "</title>"
            '<link rel="stylesheet" href="style.css">'
            '<script src="reference.js" defer></script></head><body><header>'
            '<a class="brand" href="index.html"><img src="cognipilot-logo.svg" '
            'width="200" alt="CogniPilot"></a>'
            '<p class="eyebrow">MODELICA LIBRARY · ENGINEERING REFERENCE</p>'
            + nav
            + '<form class="quick-search" action="index.html" role="search">'
            '<label for="quick-search">Find documentation or source</label>'
            '<input id="quick-search" name="q" type="search" placeholder="Search classes…">'
            "</form></header><main>" + body + "</main>" + footer + "</body></html>"
        )

    raw_directory = output / "raw"
    for path, source in sources.items():
        raw_path = raw_directory / path
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        raw_path.write_bytes(raw_sources[path])
        raw_url = "raw/" + path.as_posix()
        file_members = [item for item in classes if item.path == path]
        links = " · ".join(
            f'<a href="{item.name}.html">{item.name}</a>' for item in file_members
        )
        body = (
            f"<h1>{html.escape(path.as_posix())}</h1>"
            f'<p class="actions"><a href="{raw_url}" download>Download original .mo</a>'
            f'<a href="{SOURCE_URL}{path.as_posix()}">GitHub source</a></p>'
            f"<p>Documentation: {links}</p>"
            '<pre class="source"><code>'
            + highlighted_source(
                source, reference_links(source, path, classes, sources)
            )
            + "</code></pre>"
        )
        (output / source_page_name(path)).write_text(page(path.as_posix(), body))

    by_name = {item.name: item for item in classes}
    for item in classes:
        body = f'<p class="kind">{item.kind}</p><h1>{item.name}</h1>'
        if item.description:
            body += "<p>" + html.escape(item.description) + "</p>"
        ancestry = item.name.split(".")
        breadcrumbs = []
        for depth in range(1, len(ancestry)):
            ancestor = ".".join(ancestry[:depth])
            if ancestor in names:
                breadcrumbs.append(
                    f'<a href="{ancestor}.html">{ancestry[depth - 1]}</a>'
                )
        body += (
            '<nav class="breadcrumbs" aria-label="Breadcrumbs">'
            + " / ".join(breadcrumbs)
            + "</nav>"
        )
        line = sources[item.path].count("\n", 0, item.start) + 1
        body += (
            '<nav class="actions" aria-label="Class views">'
            '<a href="#documentation">Documentation</a>'
            f'<a href="{source_page_name(item.path)}#L{line}">View source</a>'
            f'<a href="raw/{item.path.as_posix()}" download>Download .mo</a>'
            f'<a href="{SOURCE_URL}{item.path.as_posix()}#L{line}">GitHub</a></nav>'
            '<section id="documentation">'
        )
        body += rewrite_links(item.info, names) if item.info else ""
        body += "</section>"
        declarations, bases = api(item, sources[item.path])
        if bases:
            references = []
            for base in bases:
                target = resolve_name(base, item.name, names)
                references.append(
                    f'<a href="{target}.html">{html.escape(base)}</a>'
                    if target
                    else html.escape(base)
                )
            body += "<h2>Base classes</h2><p>" + ", ".join(references) + "</p>"
            body += "<p>Follow base classes for inherited declarations; this page lists explicit declarations only.</p>"
        body += api_table(declarations, item.name, names)
        children = member_order(
            item, [by_name[n] for n in names if n.rpartition(".")[0] == item.name], root
        )
        if children:
            rows = "".join(
                f'<tr><td><a href="{child.name}.html">{child.name.rsplit(".", 1)[-1]}</a></td>'
                f"<td>{child.kind}</td><td>{html.escape(child.description)}</td></tr>"
                for child in children
            )
            body += (
                '<h2>Members</h2><div class="table-scroll"><table><thead><tr>'
                "<th>Name</th><th>Kind</th><th>Description</th></tr></thead><tbody>"
                + rows
                + "</tbody></table></div>"
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
    entries = "".join(
        f'<li><a href="{n}.html">{n}</a> '
        f'<a class="source-shortcut" href="{source_page_name(by_name[n].path)}#L{sources[by_name[n].path].count(chr(10), 0, by_name[n].start) + 1}">source</a></li>'
        for n in sorted(names)
    )
    body = (
        "<h1>Modelica library reference</h1>"
        "<p>Explore reusable models through their embedded documentation and source declarations.</p>"
        "<h2>Packages</h2><ul>" + packages + "</ul>"
        "<h2>Find a class</h2><label>Package or class name "
        '<input id="search" type="search" placeholder="Start typing to search all classes"></label>'
        '<details id="index"><summary>Browse all classes</summary><ul id="classes">'
        + entries
        + "</ul></details>"
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
