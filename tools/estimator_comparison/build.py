#!/usr/bin/env python3
"""Export the actual estimator and FOH blocks, then build CSV replay drivers."""

import argparse
import subprocess
from pathlib import Path


def build(args):
    repo = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    blocks = [
        (
            "Estimation.StrapdownINS.UKF.Estimator",
            "Estimation/StrapdownINS/UKF/Estimator.mo",
        ),
        ("Vehicles.Rdd2.NavigationEstimator", "Vehicles/Rdd2/NavigationEstimator.mo"),
        ("Tests.PreintegrationReplay", "Tests/PreintegrationReplay.mo"),
    ]
    production = {}
    for name, source in blocks:
        export = output / name.replace(".", "_")
        subprocess.run(
            [
                args.rumoca,
                "compile",
                source,
                "--model",
                name,
                "--source-root",
                str(repo),
                "--target",
                "galec-production",
                "--output",
                str(export),
            ],
            cwd=repo,
            check=True,
        )
        production[name] = export / name.replace(".", "_") / "ProductionCode"
    preintegrator = production["Tests.PreintegrationReplay"]
    for name, binary, defines in [
        (blocks[0][0], "ukf_replay", ["-DCOMPARE_UKF"]),
        (blocks[1][0], "modelica_replay", []),
    ]:
        code = production[name]
        subprocess.run(
            [
                args.cc,
                "-O2",
                *defines,
                "-I" + str(code),
                "-I" + str(preintegrator),
                str(repo / "tools/estimator_comparison/replay.c"),
                str(code / (name.replace(".", "_") + ".c")),
                str(preintegrator / "Tests_PreintegrationReplay.c"),
                str(code / "rumoca_galec_kernels.c"),
                "-lm",
                "-o",
                str(output / binary),
            ],
            check=True,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rumoca", default="rumoca")
    parser.add_argument("--cc", default="cc")
    build(parser.parse_args())
