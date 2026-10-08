"""Add read-only EKF3 operation snapshots to a clean owned stable-release copy."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from instrument_native_covariance import instrument
from native_delay import replace_once


def observe(stage, axis="-1", gain="static_cast<const ftype*>(nullptr)", noise="0.0"):
    return f'OBSERVE_EKF3_COVARIANCE("{stage}", {axis}, {gain}, {noise});'


def instrument_stages(source):
    evidence = {"logging": instrument(source, "ardupilot"), "files": {}}
    core = source / "libraries/AP_NavEKF3/AP_NavEKF3_core.cpp"
    previous = core.read_text()
    updated = previous
    for operation in (
        "controlFilterModes()",
        "readIMUData(predict)",
        "UpdateStrapdownEquationsNED()",
        "CovariancePrediction(nullptr)",
        "runYawEstimatorPrediction()",
        "SelectMagFusion()",
        "SelectVelPosFusion()",
        "runYawEstimatorCorrection()",
        "SelectRngBcnFusion()",
        "SelectFlowFusion()",
        "SelectBodyOdomFusion()",
        "SelectTasFusion()",
        "SelectBetaDragFusion()",
        "updateFilterStatus()",
        "moveEKFOrigin()",
        "checkUpdateEarthField()",
        "calcOutputStates()",
    ):
        marker = operation + ";"
        label = operation.split("(")[0]
        updated = replace_once(
            updated,
            marker,
            observe(label + ".before")
            + "\n        "
            + marker
            + "\n        "
            + observe(label + ".after"),
        )
    paths = [(core, previous, updated)]
    fusion = source / "libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp"
    previous = fusion.read_text()
    begin = previous.index("void NavEKF3_core::FuseVelPosNED()")
    end = previous.index("void NavEKF3_core::selectHeightForFusion()", begin)
    body = previous[begin:end]
    arguments = ("obsIndex", "Kfusion", "R_OBS[obsIndex]")
    marker = "                if (healthyFusion) {\n"
    body = replace_once(
        body,
        marker,
        marker + "                    " + observe("scalar.before", *arguments) + "\n",
    )
    for operation in ("ForceSymmetry()", "ConstrainVariances()"):
        marker = "                    " + operation + ";"
        label = "scalar." + operation.split("(")[0]
        body = replace_once(
            body,
            marker,
            "                    "
            + observe(label + ".before", *arguments)
            + "\n"
            + marker
            + "\n                    "
            + observe(label + ".after", *arguments),
        )
    paths.append((fusion, previous, previous[:begin] + body + previous[end:]))
    header = Path(__file__).with_name("native_covariance_stage_dump.h")
    for path, previous, updated in paths:
        updated = '#include "native_covariance_stage_dump.h"\n' + updated
        evidence["files"][str(path.relative_to(source))] = {
            "before_sha256": hashlib.sha256(previous.encode()).hexdigest(),
            "after_sha256": hashlib.sha256(updated.encode()).hexdigest(),
        }
        path.write_text(updated)
    shutil.copyfile(header, core.with_name(header.name))
    evidence["observer_sha256"] = hashlib.sha256(header.read_bytes()).hexdigest()
    evidence["scope"] = (
        "Read-only native operation and scalar covariance/gain snapshots; "
        "no covariance repair. Published states and full logging snapshots "
        "must match the separate covariance observer replay."
    )
    return evidence


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    args.output.write_text(json.dumps(instrument_stages(args.source), indent=2) + "\n")
