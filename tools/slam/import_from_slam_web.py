"""Migrate pinned authored SLAM classes without importing vendored libraries."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


DECLARATION = re.compile(
    r"^(?:(?:pure|partial|encapsulated|impure)\s+)*"
    r"(model|function|package|record|block|class|connector|type)\s+(\w+)\b",
    re.MULTILINE,
)
TOKEN = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|[A-Za-z_]\w*|\S')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def namespace(path):
    parent = path.parent.as_posix()
    groups = {
        "Estimation/Inertial": "SLAM.Inertial",
        "Estimation/Localization": "SLAM.Localization",
        "Vision/Features": "Vision.Features",
        "Vision/Matching": "Vision.Matching",
        "Mapping": "SLAM.Mapping",
        "LoopClosure": "SLAM.LoopClosure",
        "Optimization": "SLAM.PoseGraph",
        "SLAM": "SLAM",
        "Evaluation": "SLAM.Examples",
        "Examples": "SLAM.Examples",
        "Scene": "SLAM.Examples.Scene",
        "Vehicles": "SLAM.Examples.Vehicles",
    }
    if parent == "Math":
        return "LinearAlgebra" if path.stem == "SPD6Solve" else "Vision.Registration"
    if parent == "Sensors":
        return (
            "Vision.Sensors"
            if path.stem == "D435ImageProfile"
            else "SLAM.Examples.Sensors"
        )
    return groups[parent]


def imports_for(fragment, name, classes):
    tokens = [match.group() for match in TOKEN.finditer(fragment)]
    dependencies = set()
    for index, token in enumerate(tokens):
        if (
            token in classes
            and token != name
            and (index == 0 or tokens[index - 1] != ".")
        ):
            dependencies.add(token)
    return [
        f"  import {dependency} = {classes[dependency]};\n"
        for dependency in sorted(dependencies)
    ]


def migrate(args):
    repository = Path(__file__).resolve().parents[2]
    revision = subprocess.check_output(
        ["git", "-C", str(args.source_repo), "rev-parse", args.revision], text=True
    ).strip()
    tracked = subprocess.check_output(
        [
            "git",
            "-C",
            str(args.source_repo),
            "ls-tree",
            "-r",
            "--name-only",
            revision,
            "models",
        ],
        text=True,
    ).splitlines()
    files, records, classes = {}, [], {}
    for source in tracked:
        path = Path(source).relative_to("models")
        if (
            path.suffix != ".mo"
            or {"Libraries", "upstream"}.intersection(path.parts)
            or path.name == "package.mo"
        ):
            continue
        data = subprocess.check_output(
            ["git", "-C", str(args.source_repo), "show", f"{revision}:{source}"]
        )
        text = data.decode()
        files[source] = digest(data)
        declarations = list(DECLARATION.finditer(text))
        previous_end = 0
        for declaration in declarations:
            name = declaration.group(2)
            endings = list(
                re.finditer(
                    r"^end\s+" + name + r"\s*;", text[declaration.end() :], re.MULTILINE
                )
            )
            if name in classes or len(endings) != 1:
                raise ValueError(f"Ambiguous top-level class {source}:{name}")
            end = declaration.end() + endings[0].end()
            fragment = text[previous_end:end]
            if previous_end == 0:
                fragment = re.sub(r"\Awithin\s+\w+(?:\.\w+)*\s*;\s*", "", fragment)
            qualified = namespace(path) + "." + name
            classes[name] = qualified
            records.append(
                dict(name=name, qualified=qualified, source=source, fragment=fragment)
            )
            previous_end = end
        if text[previous_end:].strip():
            raise ValueError(f"Unconsumed source suffix: {source}")
    generated, package_directories, receipt_classes = {}, set(), []
    for record in records:
        fragment = record["fragment"]
        if record["source"].startswith("models/Examples/"):
            fragment = re.sub(r"\bExamples\.", "SLAM.Examples.", fragment)
        declaration = DECLARATION.search(fragment)
        insertion = declaration.end()
        trailing = fragment[insertion:]
        documentation = re.match(r'\s*"(?:\\.|[^"\\])*"', trailing)
        if documentation:
            insertion += documentation.end()
        imports = "".join(imports_for(fragment, record["name"], classes))
        if imports:
            fragment = fragment[:insertion] + "\n" + imports + fragment[insertion:]
        package = record["qualified"].rsplit(".", 1)[0]
        destination = Path(*record["qualified"].split(".")).with_suffix(".mo")
        data = (f"within {package};\n" + fragment.strip() + "\n").encode()
        if (repository / destination).exists():
            raise ValueError(f"Refusing to overwrite existing class {destination}")
        generated[destination] = data
        parent = destination.parent
        while parent != Path("."):
            package_directories.add(parent)
            parent = parent.parent
        receipt_classes.append(
            dict(
                source=record["source"],
                original_name=record["name"],
                destination=destination.as_posix(),
                qualified_name=record["qualified"],
                original_class_sha256=digest(record["fragment"].encode()),
                destination_sha256=digest(data),
            )
        )
    receipt = dict(
        upstream_repository="https://github.com/CogniPilot/slam_web",
        upstream_revision=revision,
        license="Apache-2.0",
        source_sha256=files,
        classes=receipt_classes,
        class_count=len(receipt_classes),
        transformation="Split top-level classes; add within clauses and explicit import aliases. Preserve arithmetic, source comments and nested classes. Update three Examples qualifiers.",
        excluded=[
            "models/Libraries",
            "models/upstream",
            "browser/TypeScript runtime",
            "test fixtures",
        ],
        consumer_cutover_complete=False,
    )
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(receipt, indent=2) + "\n")
    for path, data in generated.items():
        (repository / path).parent.mkdir(parents=True, exist_ok=True)
        (repository / path).write_bytes(data)
    for directory in sorted(
        package_directories, key=lambda path: (len(path.parts), str(path))
    ):
        package_file = repository / directory / "package.mo"
        if not package_file.exists():
            parent = ".".join(directory.parts[:-1])
            package_file.write_text(
                f"within{(' ' + parent) if parent else ''};\n\npackage {directory.name}\nend {directory.name};\n"
            )
    for directory in sorted(package_directories):
        order_file = repository / directory / "package.order"
        existing = order_file.read_text().splitlines() if order_file.exists() else []
        children = {
            path.stem
            for path in (repository / directory).glob("*.mo")
            if path.name != "package.mo"
        }
        children.update(
            path.name
            for path in (repository / directory).iterdir()
            if path.is_dir() and (path / "package.mo").exists()
        )
        order_file.write_text(
            "\n".join(existing + sorted(children - set(existing))) + "\n"
        )
    print(
        json.dumps(
            {
                "classes": len(generated),
                "source_files": len(files),
                "revision": revision,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path, required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    migrate(parser.parse_args())
