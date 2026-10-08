"""Add read-only covariance observers to owned stable-release source copies."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from native_delay import replace_once
from native_release import SOURCE_PINS


PX4 = """
        if (std::getenv("NATIVE_COVARIANCE_PATH")) {
            const auto horizontal = getLocalHorizontalPosition();
            double observed[16] = {horizontal(0), horizontal(1),
                -_gpos.altitude() + getEkfGlobalOriginAltitude()};
            for (unsigned axis = 0; axis < 3; ++axis) {
                observed[3 + axis] = _state.vel(axis);
                observed[10 + axis] = _state.gyro_bias(axis);
                observed[13 + axis] = _state.accel_bias(axis);
            }
            for (unsigned component = 0; component < 4; ++component)
                observed[6 + component] = _state.quat_nominal(component);
            writeNativeCovariance(_time_latest_us, _time_delayed_us, 1.0,
                observed, [this](unsigned row, unsigned column) {
                    return P(row, column);
                });
        }
"""

ARDUPILOT = """
    if (core_index == 0 && std::getenv("NATIVE_COVARIANCE_PATH")) {
        double observed[16];
        for (unsigned axis = 0; axis < 3; ++axis) {
            observed[axis] = stateStruct.position[axis];
            observed[3 + axis] = stateStruct.velocity[axis];
            observed[10 + axis] = stateStruct.gyro_bias[axis];
            observed[13 + axis] = stateStruct.accel_bias[axis];
        }
        for (unsigned component = 0; component < 4; ++component)
            observed[6 + component] = stateStruct.quat[component];
        const auto origin_offset = public_origin.get_distance_NE_ftype(EKF_origin);
        observed[0] += origin_offset.x;
        observed[1] += origin_offset.y;
        Location observed_origin;
        if (getOriginLLH(observed_origin))
            observed[2] += (public_origin.alt - observed_origin.alt) * 0.01;
        writeNativeCovariance(time_us,
            static_cast<unsigned long long>(imuDataDelayed.time_ms) * 1000,
            dtEkfAvg, observed, [this](unsigned row, unsigned column) {
                return P[row][column];
            });
    }
"""


def instrument(source, name):
    pin = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    if pin != SOURCE_PINS[name]:
        raise ValueError("Use the verified stable native release")
    status = subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain"], text=True
    )
    if status:
        raise ValueError("Use a new clean owned source copy")
    relative = (
        "src/modules/ekf2/EKF/ekf.cpp"
        if name == "px4"
        else "libraries/AP_NavEKF3/AP_NavEKF3_Logging.cpp"
    )
    path = source / relative
    previous = path.read_text()
    marker = (
        "\n\t\treturn true;\n\t}\n\n\treturn false;\n}\n\nbool Ekf::initialiseFilter()"
        if name == "px4"
        else "void NavEKF3_core::Log_Write(uint64_t time_us)\n{"
    )
    observed = replace_once(
        previous,
        marker,
        (PX4 + marker) if name == "px4" else marker + ARDUPILOT,
    )
    observed = '#include "native_covariance_dump.h"\n' + observed
    header = Path(__file__).with_name("native_covariance_dump.h")
    shutil.copyfile(header, path.with_name(header.name))
    path.write_text(observed)
    return dict(
        native_revision=pin,
        file=relative,
        before_sha256=hashlib.sha256(previous.encode()).hexdigest(),
        after_sha256=hashlib.sha256(observed.encode()).hexdigest(),
        observer_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
        scope="Read-only state and full native covariance export; published-state parity must be verified separately.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filter", choices=SOURCE_PINS)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    args.output.write_text(
        json.dumps(instrument(args.source, args.filter), indent=2) + "\n"
    )
