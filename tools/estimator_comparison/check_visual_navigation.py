"""Replay identical IMU, GPS and fixed-map RGB-D data through both Modelica paths."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path

import numpy as np

from check_visual_coupling import exponential, logarithm
from check_visual_tangent import rotation


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def truth(times):
    frequencies = np.array([0.22, 0.31, 0.19])
    amplitudes = np.array([1.2, 0.9, 0.25])
    phase = times[:, None] * frequencies
    position = amplitudes * np.sin(phase) + [0, 0, 4]
    velocity = amplitudes * frequencies * np.cos(phase)
    acceleration = -amplitudes * frequencies**2 * np.sin(phase)
    heading = 0.2 * np.sin(0.25 * times)
    quaternion = np.column_stack(
        (np.cos(heading / 2), np.zeros((len(times), 2)), np.sin(heading / 2))
    )
    rate = np.column_stack((np.zeros((len(times), 2)), 0.05 * np.cos(0.25 * times)))
    return position, velocity, acceleration, quaternion, rate


def capture(seed, directory, duration_s=30.0, interval_s=0.01):
    rng = np.random.default_rng(seed)
    times = np.arange(1, round(duration_s / interval_s) + 1) * interval_s
    position, velocity, _, quaternion, _ = truth(times)
    _, _, acceleration_mid, quaternion_mid, rate_mid = truth(times - interval_s / 2)
    rotations = np.array([rotation(q) for q in quaternion])
    rotations_mid = np.array([rotation(q) for q in quaternion_mid])
    gyro_bias = rng.normal(0, 0.001, 3)
    accel_bias = rng.normal(0, 0.01, 3)
    density = np.array([0.003, 0.03])
    gyro = (
        rate_mid
        + gyro_bias
        + rng.normal(size=rate_mid.shape) * density[0] / np.sqrt(interval_s)
    )
    force = np.einsum("nji,nj->ni", rotations_mid, acceleration_mid + [0, 0, 9.81])
    force += accel_bias + rng.normal(size=force.shape) * density[1] / np.sqrt(
        interval_s
    )
    landmarks = np.array([[-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 1, 0]])
    extrinsic = np.diag([1.0, -1.0, -1.0])
    lever = np.array([0.1, 0.05, -0.04])
    intrinsics = np.array([350, 350, 320, 240])
    root = np.kron(
        np.eye(4), np.array([[0.7, 0, 0], [0.1, 0.8, 0], [0.003, -0.002, 0.03]])
    )
    common = np.tile([0.03, 0.03, 0.001], 4)
    noise = root @ root.T + np.outer(common, common)
    observed = np.empty((len(times), 4, 3))
    for tick, attitude in enumerate(rotations):
        body = (landmarks - position[tick]) @ attitude
        camera = (body - lever) @ extrinsic
        observed[tick, :, :2] = (
            camera[:, :2] / camera[:, 2:] * intrinsics[:2] + intrinsics[2:]
        )
        observed[tick, :, 2] = camera[:, 2]
    observed += rng.multivariate_normal(np.zeros(12), noise, len(times)).reshape(
        -1, 4, 3
    )
    visible = (
        (observed[:, :, 0] >= 0)
        & (observed[:, :, 0] < 640)
        & (observed[:, :, 1] >= 0)
        & (observed[:, :, 1] < 480)
        & (observed[:, :, 2] > 0)
    )
    if not visible.all():
        raise AssertionError("Fixed replay landmarks must remain visible in the image")
    gps_std = np.array([0.3, 0.05])
    gps_position = position + rng.normal(size=position.shape) * gps_std[0]
    gps_velocity = velocity + rng.normal(size=velocity.shape) * gps_std[1]
    scales = np.r_[
        np.full(3, 0.05),
        np.full(3, 0.03),
        np.full(3, 0.008),
        np.full(3, 0.001),
        np.full(3, 0.01),
    ]
    perturbation = rng.normal(size=15) * scales
    initial_position, initial_velocity, _, initial_quaternion, _ = truth(
        np.array([0.0])
    )
    increment = exponential(perturbation[:9])
    angle = np.linalg.norm(perturbation[6:9])
    delta_quaternion = np.r_[
        np.cos(angle / 2), perturbation[6:9] * np.sin(angle / 2) / angle
    ]
    initial = np.r_[
        initial_position[0] + increment[:3, 3],
        initial_velocity[0] + increment[:3, 4],
        delta_quaternion,
        gyro_bias + perturbation[9:12],
        accel_bias + perturbation[12:15],
    ]
    assert np.array_equal(initial_quaternion[0], [1, 0, 0, 0])
    data = dict(
        times=times,
        position=position,
        velocity=velocity,
        quaternion=quaternion,
        gyro=gyro,
        force=force,
        density=density,
        gyro_bias=gyro_bias,
        accel_bias=accel_bias,
        landmarks=landmarks,
        extrinsic=extrinsic,
        lever=lever,
        intrinsics=intrinsics,
        noise=noise,
        observations=observed,
        gps_position=gps_position,
        gps_velocity=gps_velocity,
        gps_std=gps_std,
        initial=initial,
        prior=np.diag(scales**2),
        interval_s=interval_s,
    )
    path = directory / f"capture-{seed}.npz"
    np.savez_compressed(path, **data)
    return data, path


class Replay:
    def __init__(self, path):
        self.library = ctypes.CDLL(str(path))
        self.pointer = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        self.library.visual_navigation_create.argtypes = [
            self.pointer,
            self.pointer,
            ctypes.c_int,
        ]
        self.library.visual_navigation_create.restype = ctypes.c_void_p
        self.library.visual_navigation_step.argtypes = [
            ctypes.c_void_p,
            self.pointer,
            self.pointer,
        ]
        self.library.visual_navigation_step.restype = None
        self.library.visual_navigation_destroy.argtypes = [ctypes.c_void_p]
        self.library.visual_navigation_destroy.restype = None

    def run(self, data, scenario):
        sessions = [
            self.library.visual_navigation_create(
                np.asarray(data["initial"], dtype=np.float32),
                np.asarray(data["prior"], dtype=np.float32),
                tight,
            )
            for tight in (0, 1)
        ]
        if not all(sessions):
            for session in sessions:
                if session:
                    self.library.visual_navigation_destroy(session)
            raise MemoryError("Cannot allocate generated replay state")
        outputs = np.empty((len(data["times"]), 2, 246), dtype=np.float32)
        availability = np.zeros((len(data["times"]), 2), dtype=bool)
        try:
            for tick, time_s in enumerate(data["times"]):
                gps = (
                    (tick + 1) % 20 == 0
                    and scenario != "denied"
                    and not (scenario == "transition" and 10 <= time_s < 20)
                )
                camera = (tick + 1) % 10 == 0 and not 14 <= time_s < 16
                availability[tick] = [gps, camera]
                fields = [
                    data["gyro"][tick],
                    data["force"][tick],
                    [data["interval_s"]],
                    data["density"],
                    [gps, camera],
                    data["gps_position"][tick],
                    data["gps_velocity"][tick],
                    data["gps_std"],
                    data["landmarks"],
                    data["observations"][tick],
                    data["noise"],
                    data["extrinsic"],
                    data["lever"],
                    data["intrinsics"],
                ]
                packet = np.concatenate(
                    [np.asarray(field).ravel() for field in fields]
                ).astype(np.float32)
                if packet.size != 203:
                    raise AssertionError(f"Unexpected packet layout: {packet.size}")
                for mode, session in enumerate(sessions):
                    self.library.visual_navigation_step(
                        session, packet, outputs[tick, mode]
                    )
        finally:
            for session in sessions:
                self.library.visual_navigation_destroy(session)
        return outputs, availability


def score(data, outputs, availability):
    if not np.isfinite(outputs).all() or np.any(outputs[:, :, 245]):
        raise AssertionError("Non-finite output or generated runtime error")
    if not np.array_equal(
        outputs[:, :, 241:243].astype(bool),
        np.repeat(availability[:, None, :], 2, axis=1),
    ):
        raise AssertionError(
            "Available measurement refused or unavailable measurement fused"
        )
    reports = {}
    for mode, name in enumerate(("loose", "tight")):
        nominal = outputs[:, mode, :16].astype(float)
        covariance = outputs[:, mode, 16:241].reshape(-1, 15, 15).astype(float)
        if not np.array_equal(covariance, covariance.transpose(0, 2, 1)):
            raise AssertionError("Raw generated covariance is asymmetric")
        np.linalg.cholesky(covariance)
        position_error = nominal[:, :3] - data["position"]
        velocity_error = nominal[:, 3:6] - data["velocity"]
        error = np.empty((len(nominal), 15))
        for tick, state in enumerate(nominal):
            attitude = rotation(state[6:10])
            relative = np.eye(5)
            relative[:3, :3] = attitude.T @ rotation(data["quaternion"][tick])
            relative[:3, 3] = -attitude.T @ position_error[tick]
            relative[:3, 4] = -attitude.T @ velocity_error[tick]
            error[tick, :9] = logarithm(relative)
            error[tick, 9:12] = data["gyro_bias"] - state[10:13]
            error[tick, 12:15] = data["accel_bias"] - state[13:16]
        nees = np.einsum(
            "ni,ni->n", error, np.linalg.solve(covariance, error[..., None])[..., 0]
        )
        windows = {}
        for label, selected in (
            ("flight", data["times"] >= 0),
            ("gps_outage", (data["times"] >= 10) & (data["times"] < 20)),
            ("gps_return", data["times"] >= 20),
            ("camera_outage", (data["times"] >= 14) & (data["times"] < 16)),
        ):
            windows[label] = dict(
                position_rms_m=float(
                    np.sqrt(np.mean(np.sum(position_error[selected] ** 2, axis=1)))
                ),
                velocity_rms_m_s=float(
                    np.sqrt(np.mean(np.sum(velocity_error[selected] ** 2, axis=1)))
                ),
                attitude_rms_rad=float(
                    np.sqrt(np.mean(np.sum(error[selected, 6:9] ** 2, axis=1)))
                ),
                mean_nees_15=float(np.mean(nees[selected])),
                maximum_position_error_m=float(
                    np.max(np.linalg.norm(position_error[selected], axis=1))
                ),
            )
        reports[name] = dict(
            windows=windows,
            covariance_rows=len(covariance),
            raw_covariance_spd=True,
            gps_updates=int(availability[:, 0].sum()),
            camera_updates=int(availability[:, 1].sum()),
            mean_gps_nis_6=float(np.mean(outputs[availability[:, 0], mode, 243]))
            if np.any(availability[:, 0])
            else None,
            mean_camera_nis=float(np.mean(outputs[availability[:, 1], mode, 244])),
            camera_nis_dimension=6 if name == "loose" else 12,
        )
    reports["maximum_loose_tight_nominal_difference"] = float(
        np.max(np.abs(outputs[:, 0, :16] - outputs[:, 1, :16]))
    )
    reports["maximum_loose_tight_covariance_difference"] = float(
        np.max(np.abs(outputs[:, 0, 16:241] - outputs[:, 1, 16:241]))
    )
    return reports


def check(args):
    work = args.output.parent
    captures = work / "captures"
    results = work / "replays"
    captures.mkdir()
    results.mkdir()
    replay = Replay(args.library.resolve())
    report = dict(
        complete=False,
        passed=False,
        independent_noise_draws=len(args.seeds),
        seeds=args.seeds,
        scenarios=["gps", "denied", "transition"],
        cases=[],
        library_sha256=digest(args.library),
        scope="Generated Modelica prediction, GPS and visual correction with a fixed known map, constant biases and zero transport delay; not live SLAM or native estimator comparison.",
    )
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    for seed in args.seeds:
        data, capture_path = capture(seed, captures)
        for scenario in report["scenarios"]:
            output, availability = replay.run(data, scenario)
            result_path = results / f"{seed}-{scenario}.npz"
            np.savez_compressed(result_path, output=output, availability=availability)
            case = dict(
                seed=seed,
                scenario=scenario,
                capture_sha256=digest(capture_path),
                replay_sha256=digest(result_path),
                all_observations_inside_image=True,
                image_size_pixels=[640, 480],
                observation_minimum=data["observations"].min(axis=(0, 1)).tolist(),
                observation_maximum=data["observations"].max(axis=(0, 1)).tolist(),
            )
            try:
                case.update(score(data, output, availability))
                case["passed"] = (
                    case["maximum_loose_tight_nominal_difference"] < 0.001
                    and case["maximum_loose_tight_covariance_difference"] < 1e-5
                )
            except (AssertionError, np.linalg.LinAlgError) as error:
                case.update(passed=False, error=str(error))
            report["cases"].append(case)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
            print(
                f"seed={seed} scenario={scenario} passed={case['passed']}", flush=True
            )
    report.update(complete=True, passed=all(case["passed"] for case in report["cases"]))
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--seeds", type=int, nargs="+", default=list(range(20271041, 20271049))
    )
    check(parser.parse_args())
