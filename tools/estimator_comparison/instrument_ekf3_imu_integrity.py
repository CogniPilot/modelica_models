"""Observe EKF3 vertical integrity checks before its GPS velocity override."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

from instrument_native_covariance import instrument
from native_delay import replace_once


def snapshot(stage):
    return (
        '''
            if (core_index == 0) {
                const double observed_integrity[25] = {
                    static_cast<double>(dal.micros64()),
                    static_cast<double>(imuDataDelayed.time_ms) * 1000,
                    static_cast<double>(imuSampleTime_ms),
                    static_cast<double>(gpsDataDelayed.time_ms),
                    static_cast<double>(baroDataDelayed.time_ms),
                    hgtErr, velDErr, R_OBS[5], R_OBS[2], R_gain,
                    static_cast<double>(timeSinceLastBadIMU_ms),
                    static_cast<double>(badIMUdata_ms),
                    static_cast<double>(goodIMUdata_ms),
                    static_cast<double>(badIMUdata), stateStruct.velocity.z,
                    gpsDataDelayed.vel.z, velPosObs[2], stateStruct.position.z,
                    velPosObs[5], static_cast<double>(onGround),
                    static_cast<double>(dal.get_takeoff_expected()),
                    static_cast<double>(dal.get_touchdown_expected()),
                    static_cast<double>(PV_AidingMode),
                    static_cast<double>(frontend->sources.getPosZSource(core_index)),
                    static_cast<double>(gpsDataToFuse)
                };
                writeNativeImuIntegrity("'''
        + stage
        + """", observed_integrity);
            }
"""
    )


def instrument_integrity(source):
    evidence = {"logging": instrument(source, "ardupilot"), "files": {}}
    path = source / "libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp"
    previous = path.read_text()
    marker = "            if ((hgtErr*velDErr > 0.0f)"
    updated = replace_once(previous, marker, snapshot("before") + marker)
    marker = """            } else {
                badIMUdata = false;
            }
        }
"""
    updated = replace_once(
        updated,
        marker,
        marker.replace("\n        }\n", snapshot("after") + "        }\n"),
    )
    updated = '#include "native_imu_integrity_dump.h"\n' + updated
    path.write_text(updated)
    header = Path(__file__).with_name("native_imu_integrity_dump.h")
    shutil.copyfile(header, path.with_name(header.name))
    evidence["files"][str(path.relative_to(source))] = {
        "before_sha256": hashlib.sha256(previous.encode()).hexdigest(),
        "after_sha256": hashlib.sha256(updated.encode()).hexdigest(),
    }
    evidence["observer_sha256"] = hashlib.sha256(header.read_bytes()).hexdigest()
    evidence["scope"] = (
        "Read-only vertical integrity observations before and after native bad-IMU "
        "detection and GPS velocity override. Published-state and full-covariance "
        "logging parity must be checked separately."
    )
    return evidence


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    args.output.write_text(
        json.dumps(instrument_integrity(args.source), indent=2) + "\n"
    )
