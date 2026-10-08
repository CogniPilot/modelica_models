#!/usr/bin/env python3
"""Estimate isotropic white-noise densities from a declared stationary IMU window."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def calibrate(samples, start, end):
    if not np.isfinite([start, end]).all() or start >= end:
        raise ValueError("Declare a finite, increasing stationary interval")
    if samples.ndim != 2 or samples.shape[1] != 7:
        raise ValueError(
            "IMU rows must contain time, three gyro and three accel values"
        )
    if not np.isfinite(samples).all() or np.any(np.diff(samples[:, 0]) <= 0):
        raise ValueError("IMU samples must be finite and strictly ordered")
    if not len(samples) or start < samples[0, 0] or end > samples[-1, 0]:
        raise ValueError("Stationary window must lie inside the capture")
    stationary = samples[(samples[:, 0] >= start) & (samples[:, 0] < end)]
    if len(stationary) < 200:
        raise ValueError("At least 200 stationary IMU samples are required")
    intervals = np.diff(stationary[:, 0])
    period = float(np.median(intervals))
    if np.max(np.abs(intervals - period)) > 1e-4 * period:
        raise ValueError("White-noise density estimation requires regular IMU sampling")
    sample_covariance = np.cov(np.diff(stationary[:, 1:], axis=0), rowvar=False) / 2
    densities = np.sqrt(
        [np.trace(sample_covariance[:3, :3]), np.trace(sample_covariance[3:, 3:])]
    ) * np.sqrt(period / 3)
    if not np.all((densities >= 1e-9) & (densities <= 100)):
        raise ValueError("Noise densities are outside the replay's usable range")
    return dict(
        stationary_window_s=[start, end],
        samples=len(stationary),
        sample_period_s=period,
        imu_noise_density=densities.tolist(),
        density_order=["gyroscope_rad_sqrt_s", "accelerometer_m_s_sqrt_s"],
        sample_covariance=sample_covariance.tolist(),
        method="Adjacent-difference covariance / 2, averaged over each sensor's three axes; continuous spectral density is sample variance times sample period.",
        limitations="The caller declares physical rest. Assumes independent white sample noise; does not estimate bias, bias random walk, vibration spectra, colored noise or temperature drift. Not an automatic stationary detector or flight tuning recommendation.",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--stationary-window", type=float, nargs=2, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new calibration filename")
    samples = np.loadtxt(args.imu, delimiter=",", skiprows=1)
    result = calibrate(samples, *args.stationary_window)
    result["input_sha256"] = hashlib.sha256(args.imu.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print("--imu-noise-density", *result["imu_noise_density"])
