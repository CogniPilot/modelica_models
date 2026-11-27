"""Measure the native Modelica mesh options on a shaft-aligned W2025 propeller STL."""
import argparse
import csv
import json
import math
import shutil
import subprocess
from pathlib import Path

METHODS = ("snappedCartesian", "thinWallCartesian", "levelSetCut", "prism")
GRIDS = ((20, 32, 32), (30, 48, 48), (40, 64, 64))
THICKNESSES = (0.000025, 0.00005, 0.0001)
OUTPUTS = ("cells", "shells", "wallArea", "referenceArea", "wallVolume",
           "referenceVolume", "minimumCellVolume")


def measure(args, method, level, case):
    case.mkdir(parents=True, exist_ok=True)
    settings = [f"surfacePath={json.dumps(str(args.surface))}"]
    fields = OUTPUTS
    if method == "prism":
        example = "SurfacePrismStudy"
        settings.append(f"thickness={THICKNESSES[level]}")
        fields += ("minHeight", "maxHeight")
    else:
        example = "PropellerMeshStudy"
        grid = ",".join(map(str, GRIDS[level]))
        settings += [f"method=Meshing.Method.{method}", f"grid={{{grid}}}"]
    if args.walls:
        settings.append('wallPath="wall.stl"')
    model = f"model Run extends Meshing.Examples.{example}({','.join(settings)}); end Run;"
    meshing = Path(__file__).resolve().parents[1] / "Meshing" / "package.mo"
    libraries = ['loadModel(Modelica);']
    if args.msl:
        libraries = [f"loadFile({json.dumps(str(args.msl / name))});"
                     for name in ("Complex.mo", "ModelicaServices/package.mo", "Modelica/package.mo")]
    commands = [*libraries, f"loadFile({json.dumps(str(meshing))});", f"loadString({json.dumps(model)});",
                f'simulate(Run,stopTime=0,simflags="-output={",".join(fields)}");', "getErrorString();"]
    (case / "study.mos").write_text("\n".join(commands)+"\n")
    result = subprocess.run([args.omc, "study.mos"], cwd=case, text=True,
                            stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
    log = case / "run.log"
    log.write_text(result.stdout)
    if result.returncode or "The simulation finished successfully" not in result.stdout:
        raise RuntimeError(f"Modelica run failed. See {log}")
    line = next(line for line in result.stdout.splitlines() if line.startswith("time=0,"))
    data = dict(pair.split("=", 1) for pair in line.split(","))
    if not all(math.isfinite(float(data[key])) for key in fields):
        raise ValueError(f"Nonfinite mesh metric in {log}")
    return {key: data[key] for key in fields}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("surface", type=Path, help="outward-wound ASCII STL in metres")
    parser.add_argument("output", type=Path, help="case directories and mesh-results.csv")
    parser.add_argument("--method", choices=METHODS, action="append", help="repeat to select multiple methods")
    parser.add_argument("--msl", type=Path, help="optional local Modelica Standard Library checkout")
    parser.add_argument("--omc", default="omc")
    parser.add_argument("--walls", action="store_true", help="also export each inner wall as wall.stl")
    args = parser.parse_args()
    args.surface = args.surface.resolve(strict=True)
    args.omc = shutil.which(args.omc)
    if not args.omc:
        parser.error("OpenModelica compiler not found")
    if args.msl:
        args.msl = args.msl.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=True)
    columns = ("method", "grid", "requestedThickness_m", *OUTPUTS, "minHeight", "maxHeight")
    with (args.output / "mesh-results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for method in dict.fromkeys(args.method or METHODS):
            for level, grid in enumerate(GRIDS):
                case = args.output / f"{method}-{level+1}"
                metrics = measure(args, method, level, case)
                writer.writerow(dict(method=method, grid="" if method=="prism" else "x".join(map(str,grid)),
                                     requestedThickness_m=THICKNESSES[level] if method=="prism" else "", **metrics))
                stream.flush()
                print(case, flush=True)


if __name__ == "__main__":
    main()
