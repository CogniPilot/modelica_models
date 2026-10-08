"""Observe actual native scalar corrections in owned stable-release copies."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from native_delay import replace_once
from native_release import SOURCE_PINS


def px4_edits(source):
    files = {}

    def edit(relative, before, after):
        path = source / "src/modules/ekf2/EKF" / relative
        previous = files.get(path, path.read_text())
        files[path] = replace_once(previous, before, after)

    call = """fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
						   State::{state}.idx + i);"""
    for relative, state, sensor, member in (
        ("velocity_fusion.cpp", "vel", "gps_velocity", "_aid_src_gnss_vel"),
        ("position_fusion.cpp", "pos", "gps_position", "_aid_src_gnss_pos"),
    ):
        path = source / "src/modules/ekf2/EKF" / relative
        text = path.read_text()
        start = text.index(
            "bool Ekf::fuseVelocity("
            if state == "vel"
            else "bool Ekf::fuseHorizontalPosition("
        )
        before = call.format(state=state)
        prefix = text[:start]
        tail = replace_once(
            text[start:],
            before,
            before
            + f"""
            writeNativeInnovation(_time_latest_us, _time_delayed_us, aid_src.timestamp_sample,
                &aid_src == &{member} ? "{sensor}" : "other_{state}", i, "fusion",
                aid_src.innovation[i], aid_src.innovation_variance[i],
                aid_src.observation_variance[i], aid_src.test_ratio[i], 1, true);""",
        )
        files[path] = prefix + tail
    before = """fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
					   State::pos.idx + 2);"""
    edit(
        "position_fusion.cpp",
        before,
        before
        + """
        writeNativeInnovation(_time_latest_us, _time_delayed_us, aid_src.timestamp_sample,
            &aid_src == &_aid_src_baro_hgt ? "barometer" :
            (&aid_src == &_aid_src_gnss_hgt ? "gps_height" :
            (&aid_src == &_aid_src_rng_hgt ? "range_height" : "other_height")),
            0, "fusion", aid_src.innovation, aid_src.innovation_variance,
            aid_src.observation_variance, aid_src.test_ratio, 1, true);""",
    )
    before = "measurementUpdate(Kfusion, H, aid_src.observation_variance[index], aid_src.innovation[index]);"
    edit(
        "aid_sources/magnetometer/mag_fusion.cpp",
        before,
        "const bool observed_fusion = "
        + before
        + """
        writeNativeInnovation(_time_latest_us, _time_delayed_us, aid_src.timestamp_sample,
            update_all_states ? (update_tilt ? "mag_axes" : "mag_heading_axes") : "mag_field_learning",
            index, "fusion", aid_src.innovation[index], aid_src.innovation_variance[index],
            aid_src.observation_variance[index], aid_src.test_ratio[index], observed_fusion,
            update_all_states);""",
    )
    before = """measurementUpdate(Kfusion, H, _aid_src_optical_flow.observation_variance[index],
				  _aid_src_optical_flow.innovation[index]);"""
    edit(
        "aid_sources/optical_flow/optical_flow_fusion.cpp",
        before,
        "const bool observed_fusion = "
        + before
        + """
        writeNativeInnovation(_time_latest_us, _time_delayed_us,
            _aid_src_optical_flow.timestamp_sample, "optical_flow", index, "fusion",
            _aid_src_optical_flow.innovation[index], _aid_src_optical_flow.innovation_variance[index],
            _aid_src_optical_flow.observation_variance[index], _aid_src_optical_flow.test_ratio[index],
            observed_fusion, true);""",
    )
    return files


def ardupilot_edits(source):
    directory = source / "libraries/AP_NavEKF3"
    files = {}
    for name in ("PosVelFusion", "OptFlowFusion", "MagFusion"):
        path = directory / ("AP_NavEKF3_" + name + ".cpp")
        files[path] = path.read_text()

    path = directory / "AP_NavEKF3_PosVelFusion.cpp"
    before = """                if (healthyFusion) {
                    // update the covariance matrix"""
    observed = """                if (core_index == 0) {
                    const bool gps_velocity = obsIndex <= 2 && PV_AidingMode == AID_ABSOLUTE
                        && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::GPS, core_index);
                    const bool gps_position = (obsIndex == 3 || obsIndex == 4) && PV_AidingMode == AID_ABSOLUTE
                        && frontend->sources.getPosXYSource(core_index) == AP_NavEKF_Source::SourceXY::GPS;
                    const char *sensor = gps_velocity ? "gps_velocity" : gps_position ? "gps_position" :
                        obsIndex < 3 ? "other_velocity" : obsIndex < 5 ? "other_position" :
                        activeHgtSource == AP_NavEKF_Source::SourceZ::BARO ? "barometer" :
                        activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER ? "range_height" :
                        activeHgtSource == AP_NavEKF_Source::SourceZ::GPS ? "gps_height" : "other_height";
                    const uint32_t sample_ms = gps_velocity || gps_position ? gpsDataDelayed.time_ms :
                        obsIndex == 5 && activeHgtSource == AP_NavEKF_Source::SourceZ::BARO ? baroDataDelayed.time_ms :
                        obsIndex == 5 && activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER ? rangeDataDelayed.time_ms :
                        obsIndex == 5 && activeHgtSource == AP_NavEKF_Source::SourceZ::GPS ? gpsDataDelayed.time_ms : 0;
                    writeNativeInnovation(1000ULL * imuSampleTime_ms, 1000ULL * imuDataDelayed.time_ms,
                        1000ULL * sample_ms, sensor, obsIndex < 3 ? obsIndex : obsIndex < 5 ? obsIndex - 3 : 0,
                        "fusion", innovVelPos[obsIndex], varInnovVelPos[obsIndex], R_OBS[obsIndex],
                        obsIndex < 3 ? velTestRatio : obsIndex < 5 ? posTestRatio : hgtTestRatio,
                        healthyFusion, true);
                }
"""
    files[path] = replace_once(files[path], before, observed + before)

    path = directory / "AP_NavEKF3_OptFlowFusion.cpp"
    before = """            if (healthyFusion) {
                // update the covariance matrix"""
    observed = """            if (core_index == 0) {
                writeNativeInnovation(1000ULL * imuSampleTime_ms, 1000ULL * imuDataDelayed.time_ms,
                    1000ULL * ofDataDelayed.time_ms, "optical_flow", obsIndex, "fusion",
                    flowInnov[obsIndex], flowVarInnov[obsIndex], R_LOS,
                    flowTestRatio[obsIndex], healthyFusion, really_fuse);
            }
"""
    files[path] = replace_once(files[path], before, observed + before)
    before = """        // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable"""
    observed = """        if (core_index == 0) {
            writeNativeInnovation(1000ULL * imuSampleTime_ms, 1000ULL * imuDataDelayed.time_ms,
                1000ULL * ofDataDelayed.time_ms, "optical_flow", obsIndex, "gate",
                flowInnov[obsIndex], flowVarInnov[obsIndex], R_LOS,
                flowTestRatio[obsIndex], -1, really_fuse);
        }
"""
    files[path] = replace_once(files[path], before, observed + before)

    path = directory / "AP_NavEKF3_MagFusion.cpp"
    start = files[path].index("bool NavEKF3_core::fuseEulerYaw(")
    end = files[path].index("\n}\n", start) + 3
    prefix, tail, suffix = (
        files[path][:start],
        files[path][start:end],
        files[path][end:],
    )
    before = (
        """    // Declare the magnetometer unhealthy if the innovation test fails"""
    )
    observed = """    const char *observed_yaw_source = method == yawFusionMethod::MAGNETOMETER ? "mag_heading" : "other_yaw";
    const uint32_t observed_yaw_ms = method == yawFusionMethod::MAGNETOMETER ? magDataDelayed.time_ms : 0;
    if (core_index == 0) {
        writeNativeInnovation(1000ULL * imuSampleTime_ms, 1000ULL * imuDataDelayed.time_ms,
            1000ULL * observed_yaw_ms, observed_yaw_source, 0, "gate", innovYaw,
            varInnov, R_YAW, yawTestRatio, -1, true);
    }
"""
    tail = replace_once(tail, before, observed + before)
    before = """    if (healthyFusion) {
        // update the covariance matrix"""
    observed = """    if (core_index == 0) {
        writeNativeInnovation(1000ULL * imuSampleTime_ms, 1000ULL * imuDataDelayed.time_ms,
            1000ULL * observed_yaw_ms, observed_yaw_source, 0, "fusion", innovYaw,
            varInnov, R_YAW, yawTestRatio, healthyFusion, true);
    }
"""
    tail = replace_once(tail, before, observed + before)
    files[path] = prefix + tail + suffix
    return files


def instrument(source, name):
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    status = subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain"], text=True
    )
    if revision != SOURCE_PINS[name] or status:
        raise ValueError("Use a fresh clean owned stable-release source copy")
    files = px4_edits(source) if name == "px4" else ardupilot_edits(source)
    header = Path(__file__).with_name("native_innovation_dump.h")
    records = []
    for path, content in files.items():
        observed = '#include "native_innovation_dump.h"\n' + content
        records.append(
            dict(
                file=str(path.relative_to(source)),
                before_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                after_sha256=hashlib.sha256(observed.encode()).hexdigest(),
            )
        )
        shutil.copyfile(header, path.with_name(header.name))
        path.write_text(observed)
    return dict(
        native_revision=revision,
        files=records,
        observer_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
        scope="Actual scalar fusion inputs; some EKF3 gate inputs also recorded. No full joint NIS or ungated-innovation coverage claim; require published-state parity.",
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
