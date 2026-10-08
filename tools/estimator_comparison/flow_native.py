"""Map canonical FLU exposure integrals to the pinned native flow interfaces."""

import numpy as np


def ardupilot_rates(flow, index):
    duration = flow["integration_time_s"][index]
    return (
        np.array(
            [
                -flow["integrated_los_x_rad"][index],
                flow["integrated_los_y_rad"][index],
                flow["integrated_gyro_x_rad"][index],
                -flow["integrated_gyro_y_rad"][index],
            ]
        )
        / duration
    )


def verify_dataflash(path, flow):
    from pymavlink import DFReader

    reader = DFReader.DFReader_binary(str(path))
    index, maximum = 0, 0.0
    while (message := reader.recv_match(type="ROFH")) is not None:
        if index >= len(flow["t_s"]):
            raise ValueError("Duplicate native optical-flow message")
        actual = np.array([message.FX, message.FY, message.GX, message.GY])
        expected = ardupilot_rates(flow, index)
        error = np.max(np.abs(actual - expected))
        if error > 2e-6 or message.Tms != round(flow["measurement_t_s"][index] * 1000):
            raise ValueError("Serialized EKF3 optical-flow units, axes or epoch differ")
        maximum = max(maximum, float(error))
        index += 1
    if index != len(flow["t_s"]):
        raise ValueError("Missing native optical-flow message")
    return dict(
        rows=index,
        max_rate_roundtrip_error_rad_s=maximum,
        note="DAL serialization checked. EKF3 estimates fusion epoch from reception, configured delay and its fixed exposure; Tms is not used for that estimate.",
    )


def px4_source(text, replace):
    text = replace(
        text,
        "struct FlowRow { double t, vx, vy, dist, quality; int valid; };",
        "struct FlowRow { double t, vx, vy, dist, quality; int valid; double midpoint, dt, losx, losy, gx, gy, gz; };",
    )
    text = replace(
        text,
        "for (auto &r : flow_raw) flow.push_back({r[ct], r[cvx], r[cvy], r[cd], r[cq], (int)r[cv]});",
        "for (auto &r : flow_raw) flow.push_back({r[ct], r[cvx], r[cvy], r[cd], r[cq], (int)r[cv], "
        'r[col(hf,"exposure_midpoint_s")], r[col(hf,"integration_time_s")], '
        'r[col(hf,"integrated_los_x_rad")], r[col(hf,"integrated_los_y_rad")], '
        'r[col(hf,"integrated_gyro_x_rad")], r[col(hf,"integrated_gyro_y_rad")], r[col(hf,"integrated_gyro_z_rad")]});',
    )
    text = replace(text, "fs.time_us = to_us(f.t);", "fs.time_us = to_us(f.midpoint);")
    text = replace(
        text,
        "const auto &measured_imu = imu.at((size_t)llround((f.t - t_start) / .00125));\n"
        "fs.gyro_rate = -Vector3f(measured_imu.g[0], -measured_imu.g[1], -measured_imu.g[2]);",
        "fs.gyro_rate = Vector3f(-f.gx/f.dt, f.gy/f.dt, f.gz/f.dt);",
    )
    text = replace(
        text,
        "fs.flow_rate = flow_comp + fs.gyro_rate.xy();",
        "fs.flow_rate = Vector2f(f.losx/f.dt, -f.losy/f.dt);\n"
        'fprintf(stderr,"FLOW_PACKET %.9f %.9f %.9f %.9f %.9f %.9f %.9f\\n", '
        "f.t, f.midpoint, (double)fs.flow_rate(0), (double)fs.flow_rate(1), "
        "(double)fs.gyro_rate(0), (double)fs.gyro_rate(1), (double)fs.gyro_rate(2));",
    )
    return text


def verify_px4(log, flow):
    actual = np.array(
        [
            [float(value) for value in line.split()[1:]]
            for line in log.splitlines()
            if line.startswith("FLOW_PACKET ")
        ]
    )
    if len(actual) != len(flow):
        raise ValueError("Missing PX4 flow API observation")
    duration = flow["integration_time_s"]
    expected = np.column_stack(
        (
            flow["t_s"],
            flow["exposure_midpoint_s"],
            flow["integrated_los_x_rad"] / duration,
            -flow["integrated_los_y_rad"] / duration,
            -flow["integrated_gyro_x_rad"] / duration,
            flow["integrated_gyro_y_rad"] / duration,
            flow["integrated_gyro_z_rad"] / duration,
        )
    )
    maximum = float(np.max(np.abs(actual - expected)))
    if maximum > 2e-6:
        raise ValueError("PX4 flow API units, axes or epoch differ")
    return dict(rows=len(actual), max_api_mapping_error=maximum)
