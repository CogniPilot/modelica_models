#!/usr/bin/env python3
"""Create explicit finite-exposure camera packets from a frozen noisy capture."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from generate import trajectory, write
from score import read


FIELDS = (
    "t_s",
    "vx_flu_m_s",
    "vy_flu_m_s",
    "dist_m",
    "quality",
    "valid",
    "exposure_midpoint_s",
    "integration_time_s",
    "integrated_los_x_rad",
    "integrated_los_y_rad",
    "integrated_gyro_x_rad",
    "integrated_gyro_y_rad",
    "integrated_gyro_z_rad",
    "los_variance_x_rad2",
    "los_variance_y_rad2",
    "gyro_variance_x_rad2",
    "gyro_variance_y_rad2",
    "gyro_variance_z_rad2",
)


def integrate_windows(times, values, endpoints, duration):
    times, values, endpoints = map(np.asarray, (times, values, endpoints))
    interval = np.diff(times)
    if (
        times.ndim != 1
        or values.ndim != 2
        or values.shape[0] != len(times)
        or not np.isfinite(values).all()
        or not np.isfinite(times).all()
        or not np.isfinite(duration)
        or duration <= 0
        or not np.all(interval > 0)
        or not np.isfinite(endpoints).all()
    ):
        raise ValueError("Invalid exposure samples or duration")
    starts = np.searchsorted(times, endpoints - duration - 1e-8)
    ends = np.searchsorted(times, endpoints - 1e-8)
    if np.any(starts < 0) or np.any(ends >= len(times)) or np.any(ends <= starts):
        raise ValueError("Exposure extends outside sample support")
    if not np.allclose(
        times[starts], endpoints - duration, atol=1e-8, rtol=0
    ) or not np.allclose(times[ends], endpoints, atol=1e-8, rtol=0):
        raise ValueError("Exposure endpoints must lie on the sample grid")
    increments = interval[:, None] * (values[:-1] + values[1:]) / 2
    integral = np.vstack((np.zeros(values.shape[1]), np.cumsum(increments, axis=0)))
    return integral[ends] - integral[starts]


def camera_rates(body_velocity, distance, angular_velocity):
    return (
        np.column_stack((-body_velocity[:, 1], body_velocity[:, 0])) / distance[:, None]
        - angular_velocity[:, :2]
    )


def generate(source, output):
    if output.exists():
        raise ValueError("Choose a new capture directory")
    origin = json.loads((source / "origin.json").read_text())
    flow = read(source / "flow.csv")
    imu = read(source / "imu.csv")
    if not np.allclose(np.diff(flow["t_s"]), 0.01, atol=1e-8, rtol=0):
        raise ValueError("Exposure generation requires a frozen 100 Hz flow capture")
    if not np.allclose(np.diff(imu["t_s"]), 0.00125, atol=1e-8, rtol=0):
        raise ValueError("Exposure generation requires a frozen 800 Hz IMU capture")
    speed, height = origin["speed"], origin.get("climb_height_m", 2)
    warmup_s = origin.get("arm_after_s", 13)
    flow_truth = trajectory(flow["t_s"], speed, height, warmup_s)
    imu_truth = trajectory(imu["t_s"], speed, height, warmup_s)
    measured_velocity = np.column_stack((flow["vx_flu_m_s"], flow["vy_flu_m_s"]))
    image_rates = camera_rates(measured_velocity, flow_truth[-1], flow_truth[4])
    rng = np.random.default_rng(np.random.SeedSequence([origin["seed"], 20271007, 1]))
    camera_gyro = imu_truth[4] + rng.normal(0, 0.0015, imu_truth[4].shape)
    endpoints = np.arange(0.1, imu["t_s"][-1] + 0.00001, 0.1)
    duration = 0.1
    centers = endpoints - duration / 2
    image = integrate_windows(flow["t_s"], image_rates, endpoints, duration)
    gyro = integrate_windows(imu["t_s"], camera_gyro, endpoints, duration)
    distance = np.interp(centers, flow["t_s"], flow["dist_m"])
    compensated = (image + gyro[:, :2]) / duration
    equivalent_velocity = (
        np.column_stack((compensated[:, 1], -compensated[:, 0])) * distance[:, None]
    )
    image_variance = []
    for end in endpoints:
        indices = np.flatnonzero(
            (flow["t_s"] >= end - duration - 1e-8) & (flow["t_s"] <= end + 1e-8)
        )
        weights = np.full(len(indices), 0.01)
        weights[[0, -1]] *= 0.5
        image_variance.append(np.sum((weights * 0.03 / flow_truth[-1][indices]) ** 2))
    gyro_variance = 0.0015**2 * 0.00125**2 * (80 - 0.5)
    packets = np.column_stack(
        (
            endpoints,
            equivalent_velocity,
            distance,
            np.ones((len(endpoints), 2)),
            centers,
            np.full(len(endpoints), duration),
            image,
            gyro,
            np.repeat(np.array(image_variance)[:, None], 2, axis=1),
            np.full((len(endpoints), 3), gyro_variance),
        )
    )
    output.mkdir(parents=True)
    for name in ("imu.csv", "gps.csv", "truth.csv", "baro_raw.csv"):
        shutil.copyfile(source / name, output / name)
    write(output, "flow", ",".join(FIELDS), packets)
    for name in ("mag", "baro"):
        with (source / (name + ".csv")).open() as stream:
            header = stream.readline().strip()
            values = np.loadtxt(stream, delimiter=",")
        write(output, name, header, values[::5])
    with (source / "modelica_input.csv").open() as stream:
        header = stream.readline().strip()
        merged = np.loadtxt(stream, delimiter=",")
    selected = np.searchsorted(endpoints, merged[:, 0] + 1e-8, side="right") - 1
    available = selected >= 0
    merged[~available, 15] = -1
    merged[available, 15:19] = packets[selected[available], :4]
    mag = read(output / "mag.csv")
    baro = read(output / "baro.csv")
    index = np.searchsorted(mag["t_s"], merged[:, 0] + 1e-8, side="right") - 1
    merged[:, 19] = np.isclose(merged[:, 0], mag["t_s"][index], atol=1e-8, rtol=0)
    merged[:, 20] = mag["t_s"][index]
    for column, name in enumerate(("bx_T", "by_T", "bz_T"), 21):
        merged[:, column] = mag[name][index]
    merged[:, 24] = baro["altitude_m"][index]
    write(output, "modelica_input", header, merged)
    origin.setdefault("sensor_rates_hz", dict(imu=800, gps=10)).update(
        flow=10, mag=10, baro=10
    )
    origin["flow_exposure"] = dict(
        duration_s=duration,
        packet_timestamp="exposure end",
        measurement_timestamp="exposure midpoint",
        camera_gyro="Independent calibrated unbiased sensor, 800 Hz, IID sigma .0015 rad/s; not the navigation IMU",
        image_noise="Frozen 100 Hz image translation noise: .03 m/s per axis divided by physical range",
        quadrature="Trapezoidal integration",
        range="Co-timed midpoint range from frozen independent .02 m noise",
        parent_sha256={
            name: hashlib.sha256((source / name).read_bytes()).hexdigest()
            for name in (
                "imu.csv",
                "flow.csv",
                "mag.csv",
                "baro.csv",
                "truth.csv",
                "modelica_input.csv",
                "origin.json",
            )
        },
    )
    (output / "origin.json").write_text(json.dumps(origin, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output)
