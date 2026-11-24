"""Run OpenFOAM's extended mesh checks and propagate failed checks to the caller."""

import argparse
from pathlib import Path
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path, help="OpenFOAM case with constant/polyMesh and system dictionaries")
    parser.add_argument("--docker", action="store_true", help="use the OpenFOAM v2412 container")
    args = parser.parse_args()
    case = args.case.resolve()
    if not (case / "constant/polyMesh/boundary").is_file():
        parser.error(f"missing constant/polyMesh/boundary in {case}")
    if args.docker:
        cmd = ["docker", "run", "--rm", "-v", f"{case}:/case", "opencfd/openfoam-default:2412",
               "checkMesh", "-case", "/case"]
    else:
        cmd = ["checkMesh", "-case", str(case)]
    cmd.extend(["-constant", "-allTopology", "-allGeometry"])
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    except OSError as exc:
        print(exc, file=sys.stderr)
        return 1
    sys.stdout.write(result.stdout)
    return 0 if result.returncode == 0 and "Mesh OK." in result.stdout.splitlines() else 1


if __name__ == "__main__":
    sys.exit(main())
