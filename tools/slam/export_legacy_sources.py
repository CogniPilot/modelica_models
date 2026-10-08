"""Export generated flat-name SLAM snapshots for existing slam_web consumers."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def export(args):
    repository = args.repository.resolve()
    output = args.output.resolve()
    if output.exists() or output.is_relative_to(repository):
        raise ValueError(
            "Choose a fresh snapshot directory outside the source checkout"
        )

    def read(path):
        return subprocess.check_output(
            ["git", "-C", str(repository), "show", f"{args.revision}:{path}"]
        )

    revision = subprocess.check_output(
        ["git", "-C", str(repository), "rev-parse", args.revision + "^{commit}"],
        text=True,
    ).strip()
    manifest = json.loads(read("tools/slam/source-manifest.json"))
    groups = {}
    for entry in manifest["classes"]:
        data = read(entry["destination"]).decode().split("\n", 1)[1]
        data = re.sub(r"^  import \w+ = [\w.]+;\n", "", data, flags=re.MULTILINE)
        if entry["source"].startswith("models/Examples/"):
            data = data.replace("SLAM.Examples.", "Examples.")
        groups.setdefault(entry["source"], []).append(data.strip())
    hashes = {}
    for relative, classes in groups.items():
        header = f"// Generated from CogniPilot/modelica_models {revision}; edit the canonical packages there.\n"
        within = "within Examples;\n" if relative.startswith("models/Examples/") else ""
        data = (within + header + "\n\n".join(classes) + "\n").encode()
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        hashes[relative] = hashlib.sha256(data).hexdigest()
    provenance = dict(
        repository="https://github.com/CogniPilot/modelica_models",
        revision=revision,
        license="Apache-2.0",
        generated=True,
        packaging="Compatibility snapshots assembled from canonical package classes; within/import adaptation restores the existing browser's flat-name interface. Do not edit these generated copies.",
        sources=hashes,
        canonical_classes=manifest["classes"],
    )
    (output / "slam-provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            {"revision": revision, "files": len(hashes), "output": str(output)},
            indent=2,
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository", type=Path, default=Path(__file__).resolve().parents[2]
    )
    parser.add_argument("--revision", required=True)
    parser.add_argument("--output", type=Path, required=True)
    export(parser.parse_args())
