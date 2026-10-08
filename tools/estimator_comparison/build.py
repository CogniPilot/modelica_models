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
    ukf_name, estimator_name, preintegrator_name = [name for name, _ in blocks]
    if args.eskf_only:
        blocks = blocks[1:]
    predictor_name = "Estimation.FusionHorizon.OutputPredictor"
    queue_name = "Estimation.FusionHorizon.AidingBuffer"
    if args.horizon:
        blocks += [
            (predictor_name, "Estimation/FusionHorizon/OutputPredictor.mo"),
            (queue_name, "Estimation/FusionHorizon/AidingBuffer.mo"),
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
                *(
                    ["--cache-dir", str(args.rumoca_cache_dir)]
                    if args.rumoca_cache_dir
                    else []
                ),
            ],
            cwd=repo,
            check=True,
        )
        production[name] = export / name.replace(".", "_") / "ProductionCode"
    objects = {}
    for name, code in production.items():
        target = output / (name.replace(".", "_") + ".o")
        subprocess.run(
            [
                args.cc,
                "-pipe",
                "-O2",
                "-c",
                str(code / (name.replace(".", "_") + ".c")),
                "-o",
                str(target),
            ],
            check=True,
        )
        objects[name] = target
    kernel_bytes = (production[estimator_name] / "rumoca_galec_kernels.c").read_bytes()
    if any(
        (code / "rumoca_galec_kernels.c").read_bytes() != kernel_bytes
        for code in production.values()
    ):
        raise ValueError(
            "Generated blocks require different shared kernel implementations"
        )
    kernels = output / "rumoca_galec_kernels.o"
    subprocess.run(
        [
            args.cc,
            "-pipe",
            "-O2",
            "-c",
            str(production[estimator_name] / "rumoca_galec_kernels.c"),
            "-o",
            str(kernels),
        ],
        check=True,
    )
    variants = [(estimator_name, "modelica_replay", [])]
    if not args.eskf_only:
        variants.append((ukf_name, "ukf_replay", ["-DCOMPARE_UKF"]))
    if args.horizon:
        variants.append((estimator_name, "horizon_replay", ["-DCOMPARE_HORIZON"]))
    for name, binary, defines in variants:
        dependencies = [name, preintegrator_name]
        if binary == "horizon_replay":
            dependencies += [predictor_name, queue_name]
        if name == estimator_name:
            if args.semi_direct_bias:
                defines.append("-DSEMIDIRECT_BIAS")
            if args.stationary_imu:
                defines.append("-DSTATIONARY_IMU_MODEL")
            if args.square_root_covariance:
                defines.append("-DSQUARE_ROOT_COVARIANCE")
            if args.equivariant_magnetometer:
                defines.append("-DEQUIVARIANT_MAGNETOMETER")
            if args.geometric_alignment:
                defines.append("-DGEOMETRIC_ALIGNMENT")
            if args.declared_rest_barometer:
                defines.append("-DDECLARED_REST_BAROMETER")
            if args.barometer_bias_consider:
                defines.append("-DBAROMETER_BIAS_CONSIDER")
            if args.joint_barometer_bias:
                defines.append("-DJOINT_BAROMETER_BIAS")
        subprocess.run(
            [
                args.cc,
                "-pipe",
                "-O2",
                *defines,
                *["-I" + str(production[dependency]) for dependency in dependencies],
                str(repo / "tools/estimator_comparison/replay.c"),
                *[str(objects[dependency]) for dependency in dependencies],
                str(kernels),
                "-lm",
                "-o",
                str(output / binary),
            ],
            check=True,
        )

    if args.horizon:
        dependencies = [estimator_name, predictor_name, queue_name]
        executable = output / "horizon_adapter_test"
        subprocess.run(
            [
                args.cc,
                "-pipe",
                "-O2",
                *["-I" + str(production[name]) for name in dependencies],
                str(repo / "tools/estimator_comparison/horizon_adapter.c"),
                *[str(objects[name]) for name in dependencies],
                str(kernels),
                "-lm",
                "-o",
                str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rumoca", default="rumoca")
    parser.add_argument("--rumoca-cache-dir", type=Path)
    parser.add_argument("--cc", default="cc")
    parser.add_argument("--equivariant-magnetometer", action="store_true")
    parser.add_argument("--geometric-alignment", action="store_true")
    parser.add_argument("--declared-rest-barometer", action="store_true")
    parser.add_argument("--barometer-bias-consider", action="store_true")
    parser.add_argument("--joint-barometer-bias", action="store_true")
    parser.add_argument("--horizon", action="store_true")
    parser.add_argument("--semi-direct-bias", action="store_true")
    parser.add_argument("--stationary-imu", action="store_true")
    parser.add_argument("--square-root-covariance", action="store_true")
    parser.add_argument("--eskf-only", action="store_true")
    build(parser.parse_args())
