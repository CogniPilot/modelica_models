#!/usr/bin/env python3
"""Qualify and compare RDD2 strapdown estimators against truth feedback.

Five runs fly the same box: truth feedback, then ESKF and UKF with identical
optical-flow or GPS aiding. The truth run isolates the plant/controller, while
the paired aided runs expose estimator accuracy, covariance consistency, and
whole-mission computational cost under the same noise realization.
"""

from __future__ import annotations

import base64
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path
import time as wall_time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rumoca as rum


def repository_root() -> Path:
    configured = os.environ.get("MODELICA_MODELS_ROOT")
    candidates = ([Path(configured).expanduser()] if configured else []) + [
        Path.cwd(),
        *Path.cwd().parents,
        Path(__file__).resolve().parent,
        *Path(__file__).resolve().parents,
    ]
    for candidate in candidates:
        if (candidate / "Vehicles/Rdd2/Plant.mo").is_file():
            return candidate.resolve()
    raise RuntimeError("could not find the modelica_models repository")


ROOT = repository_root()
MODEL_DIR = ROOT / "Vehicles" / "Rdd2" / "Test"
ARTIFACT_DIR = ROOT / "artifacts" / "vehicles" / "rdd2"
MISSION_DURATION_S = 45.0
MISSION_PLOT_END_S = 26.0
CRUISE_ALTITUDE_M = 2.0
BOX_SIDE_M = 4.0
CORNER_TOLERANCE_M = 1.1
BOX_CORNERS = [
    ("east", (BOX_SIDE_M, 0.0)),
    ("northeast", (BOX_SIDE_M, BOX_SIDE_M)),
    ("north", (0.0, BOX_SIDE_M)),
    ("home", (0.0, 0.0)),
]
SCENARIOS = {
    "truth": MODEL_DIR / "rumoca-scenario.waypoint-truth.toml",
    "eskf_optical_flow": MODEL_DIR / "rumoca-scenario.waypoint-local.toml",
    "eskf_gps": MODEL_DIR / "rumoca-scenario.waypoint-global.toml",
    "ukf_optical_flow": MODEL_DIR / "rumoca-scenario.waypoint-local-ukf.toml",
    "ukf_gps": MODEL_DIR / "rumoca-scenario.waypoint-global-ukf.toml",
}
TRACE_NAMES = [
    "time_s",
    "imuSamplePeriod_s",
    *[f"position_m[{index}]" for index in range(1, 4)],
    *[f"velocity_m_s[{index}]" for index in range(1, 4)],
    *[f"euler_rad[{index}]" for index in range(1, 4)],
    "thrust_N",
    *[f"motorCommand[{index}]" for index in range(1, 5)],
    *[f"avionics.reference.position[{index}]" for index in range(1, 4)],
    "avionics.reference.trajectoryTime",
    *[f"geodetic[{index}]" for index in range(1, 4)],
    "missionPhase",
    "navigationError_m",
    "controllerEstimatorFeedbackError_m",
    "estimatorUpdatePeriod_s",
    *[f"gpsPositionNoise_m[{index}]" for index in range(1, 4)],
    *[f"gpsVelocityNoise_m_s[{index}]" for index in range(1, 4)],
    *[f"opticalFlowIntegratedNoise_rad[{index}]" for index in range(1, 3)],
    *[
        f"opticalFlowGyroscopeIntegratedNoise_rad[{index}]"
        for index in range(1, 3)
    ],
    "opticalFlowGroundDistanceNoise_m",
    "opticalFlowIdealGroundDistance_m",
    "opticalFlowSurfaceVisible",
    *[f"imuAngularVelocityNoise_rad_s[{index}]" for index in range(1, 4)],
    *[f"imuSpecificForceNoise_m_s2[{index}]" for index in range(1, 4)],
    *[f"imuPreintegratedDeltaAngle_rad[{index}]" for index in range(1, 4)],
    *[f"imuPreintegratedDeltaVelocity_m_s[{index}]" for index in range(1, 4)],
    "imuPreintegrationTime_s",
    *[f"imuGyroscopeBias_rad_s[{index}]" for index in range(1, 4)],
    *[f"imuAccelerometerBias_m_s2[{index}]" for index in range(1, 4)],
    *[f"imuAngularVelocityNoiseVariance_rad2_s2[{index}]" for index in range(1, 4)],
    *[f"imuSpecificForceNoiseVariance_m2_s4[{index}]" for index in range(1, 4)],
    *[
        f"imuGyroscopeBiasIncrementVariance_rad2_s2[{index}]"
        for index in range(1, 4)
    ],
    *[
        f"imuAccelerometerBiasIncrementVariance_m2_s4[{index}]"
        for index in range(1, 4)
    ],
    *[
        f"estimator.gps.positionCovarianceWorld_m2[{index},{index}]"
        for index in range(1, 4)
    ],
    *[
        f"estimator.gps.velocityCovarianceWorld_m2_s2[{index},{index}]"
        for index in range(1, 4)
    ],
    *[
        f"estimator.opticalFlow.integratedLineOfSightCovariance_rad2[{index},{index}]"
        for index in range(1, 3)
    ],
    *[
        f"estimator.opticalFlow.integratedGyroscopeCovariance_rad2[{index},{index}]"
        for index in range(1, 3)
    ],
    "estimator.opticalFlow.groundDistanceVariance_m2",
    "estimator.gps.fresh",
    "estimator.opticalFlow.fresh",
    *[f"estimator.estimate.positionWorldEnu_m[{index}]" for index in range(1, 4)],
    *[f"estimator.estimate.velocityWorldEnu_m_s[{index}]" for index in range(1, 4)],
    *[
        f"estimator.estimate.rotationWorldBody[{row},{column}]"
        for row in range(1, 4)
        for column in range(1, 4)
    ],
    *[
        f"estimator.navigationCovarianceLocal[{row},{column}]"
        for row in range(1, 7)
        for column in range(1, 7)
    ],
    "estimator.status.initialized",
    "estimator.status.predictionAccepted",
    "estimator.status.gpsPositionCorrectionAccepted",
    "estimator.status.gpsVelocityCorrectionAccepted",
    "estimator.status.opticalFlowCorrectionAccepted",
    "estimator.status.anchorSource",
    "estimator.status.correctionSource",
    "estimator.status.normalizedInnovationSquared",
    "estimator.estimate.valid",
]


def columns(result: "rum.Result") -> dict[str, list[float]]:
    """Copy only qualification signals, then release Rumoca's full trace."""
    values = {"time": [float(value) for value in result.time]}
    for requested in TRACE_NAMES:
        matches = (
            [requested]
            if requested in result.names
            else [name for name in result.names if name.endswith(f".{requested}")]
        )
        if len(matches) != 1:
            raise KeyError(
                f"Rumoca result has no unambiguous signal {requested!r}: {matches}"
            )
        values[requested] = [float(value) for value in result[matches[0]]]
    return values


def signal(values: dict[str, list[float]], name: str) -> list[float]:
    if name in values:
        return values[name]
    suffix_matches = [column for column in values if column.endswith(f".{name}")]
    if len(suffix_matches) == 1:
        return values[suffix_matches[0]]
    raise KeyError(f"Rumoca result has no unambiguous signal {name!r}")


def source_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def simulate(scenario: Path) -> dict[str, list[float]]:
    print(f"simulating {scenario.name}...", flush=True)
    _session, model, simulation = rum.Session.from_scenario(str(scenario))
    started = wall_time.perf_counter()
    result = model.simulate(t=(0.0, MISSION_DURATION_S), config=simulation)
    values = columns(result)
    elapsed = wall_time.perf_counter() - started
    print(f"completed {scenario.name} in {elapsed:.3f} s", flush=True)
    values["_simulation_wall_time_s"] = [elapsed]
    return values


def aiding_event_indices(
    values: dict[str, list[float]], expected_source: int
) -> list[int]:
    """Indices on the estimator clock where the expected source was attempted."""
    sources = signal(values, "estimator.status.correctionSource")
    period = signal(values, "estimatorUpdatePeriod_s")[0]
    return [
        index
        for index in native_sample_indices(values, period)
        if abs(sources[index] - expected_source) < 0.5
    ]


def native_sample_indices(
    values: dict[str, list[float]], period_s: float
) -> list[int]:
    """Return one post-event trace row for every native sensor tick."""
    times = np.asarray(signal(values, "time_s"))
    selected: dict[int, int] = {}
    for index, sample_time in enumerate(times):
        tick = round(sample_time / period_s)
        if abs(sample_time / period_s - tick) < 1.0e-7:
            # Rumoca can retain multiple event rows at the same timestamp.
            # Keeping the last row observes discrete random-walk updates after
            # the sensor clock event and never samples one packet repeatedly on
            # the faster trace/output clock.
            selected[tick] = index
    return [selected[tick] for tick in sorted(selected)]


def corner_error(
    time: list[float], x: list[float], y: list[float], target: tuple[float, float]
) -> float:
    return min(
        math.hypot(x[index] - target[0], y[index] - target[1])
        for index in range(len(time))
    )


def navigation_nees(values: dict[str, list[float]]) -> tuple[list[float], list[float]]:
    """Return time and six-state position/velocity NEES in local tangent axes."""
    times = signal(values, "time_s")
    valid = signal(values, "estimator.estimate.valid")
    mission_phase = signal(values, "missionPhase")
    truth_position = np.column_stack(
        [signal(values, f"position_m[{index}]") for index in range(1, 4)]
    )
    truth_velocity = np.column_stack(
        [signal(values, f"velocity_m_s[{index}]") for index in range(1, 4)]
    )
    estimate_position = np.column_stack(
        [
            signal(values, f"estimator.estimate.positionWorldEnu_m[{index}]")
            for index in range(1, 4)
        ]
    )
    estimate_velocity = np.column_stack(
        [
            signal(values, f"estimator.estimate.velocityWorldEnu_m_s[{index}]")
            for index in range(1, 4)
        ]
    )

    nees_time: list[float] = []
    nees_values: list[float] = []
    estimator_period_s = signal(values, "estimatorUpdatePeriod_s")[0]
    for sample_index in native_sample_indices(values, estimator_period_s):
        sample_time = times[sample_index]
        if valid[sample_index] <= 0.5 or mission_phase[sample_index] <= 0.5:
            continue
        rotation = np.array(
            [
                [
                    signal(
                        values,
                        f"estimator.estimate.rotationWorldBody[{row},{column}]",
                    )[sample_index]
                    for column in range(1, 4)
                ]
                for row in range(1, 4)
            ]
        )
        covariance = np.array(
            [
                [
                    signal(
                        values,
                        f"estimator.navigationCovarianceLocal[{row},{column}]",
                    )[sample_index]
                    for column in range(1, 7)
                ]
                for row in range(1, 7)
            ]
        )
        position_error = rotation.T @ (
            truth_position[sample_index] - estimate_position[sample_index]
        )
        velocity_error = rotation.T @ (
            truth_velocity[sample_index] - estimate_velocity[sample_index]
        )
        error = np.concatenate((position_error, velocity_error))
        covariance = 0.5 * (covariance + covariance.T)
        try:
            nees = float(error @ np.linalg.solve(covariance, error))
        except np.linalg.LinAlgError:
            continue
        if math.isfinite(nees) and nees >= 0.0:
            nees_time.append(sample_time)
            nees_values.append(nees)
    return nees_time, nees_values


def innovation_nis(
    values: dict[str, list[float]], expected_source: int
) -> tuple[list[float], list[float], int]:
    """Return NIS only on ticks that attempted the expected aiding source."""
    times = signal(values, "time_s")
    sources = signal(values, "estimator.status.correctionSource")
    nis = signal(values, "estimator.status.normalizedInnovationSquared")
    mission_phase = signal(values, "missionPhase")
    selected = [
        index
        for index in aiding_event_indices(values, expected_source)
        if abs(sources[index] - expected_source) < 0.5
        and mission_phase[index] > 0.5
    ]
    if not selected:
        return [], [], 0
    degree = 6 if expected_source == 2 else 2
    return (
        [times[index] for index in selected],
        [nis[index] for index in selected],
        degree,
    )


def chi_square_95_bounds(degrees_of_freedom: int) -> tuple[float, float]:
    bounds = {
        2: (0.0506356, 7.37776),
        3: (0.215795, 9.34840),
        6: (1.23734, 14.4494),
    }
    return bounds[degrees_of_freedom]


def fraction_in_bounds(samples: list[float], lower: float, upper: float) -> float:
    if not samples:
        return 0.0
    return sum(lower <= sample <= upper for sample in samples) / len(samples)


def maximum_noise_correlation(sample_sets: list[np.ndarray]) -> float:
    """Maximum absolute lag-one or cross-channel correlation."""
    correlations = []
    for samples in sample_sets:
        correlations.append(abs(float(np.corrcoef(samples[:-1], samples[1:])[0, 1])))
    for first in range(len(sample_sets)):
        for second in range(first + 1, len(sample_sets)):
            correlations.append(
                abs(float(np.corrcoef(sample_sets[first], sample_sets[second])[0, 1]))
            )
    return max(correlations, default=0.0)


def empirical_whiteness_limit(sample_sets: list[np.ndarray]) -> float:
    """Three-sigma finite-record bound, retaining the historical 0.06 floor."""
    sample_count = min((len(samples) for samples in sample_sets), default=0)
    return max(0.06, 3.0 / math.sqrt(max(sample_count - 1, 1)))


def evaluate(values: dict[str, list[float]], feedback_mode: str) -> dict[str, object]:
    time = signal(values, "time_s")
    x = signal(values, "position_m[1]")
    y = signal(values, "position_m[2]")
    altitude = signal(values, "position_m[3]")
    vertical_speed = signal(values, "velocity_m_s[3]")
    roll_deg = [math.degrees(value) for value in signal(values, "euler_rad[1]")]
    pitch_deg = [math.degrees(value) for value in signal(values, "euler_rad[2]")]
    motors = [signal(values, f"motorCommand[{index}]") for index in range(1, 5)]
    thrust = signal(values, "thrust_N")
    navigation_error = signal(values, "navigationError_m")
    feedback_error = signal(values, "controllerEstimatorFeedbackError_m")
    estimator_initialized = signal(values, "estimator.status.initialized")
    prediction_accepted = signal(values, "estimator.status.predictionAccepted")
    gps_position_accepted = signal(
        values, "estimator.status.gpsPositionCorrectionAccepted"
    )
    gps_velocity_accepted = signal(
        values, "estimator.status.gpsVelocityCorrectionAccepted"
    )
    optical_flow_accepted = signal(
        values, "estimator.status.opticalFlowCorrectionAccepted"
    )
    anchor_source = signal(values, "estimator.status.anchorSource")
    estimate_valid = signal(values, "estimator.estimate.valid")

    expected_sources = {"truth": 0.0, "gps": 1.0, "optical_flow": 2.0}
    if feedback_mode not in expected_sources:
        raise ValueError(f"unsupported feedback mode {feedback_mode!r}")

    active_indices = [
        index
        for index, phase in enumerate(signal(values, "missionPhase"))
        if phase > 0.5
    ]

    corner_errors = {
        name: corner_error(time, x, y, target) for name, target in BOX_CORNERS
    }
    max_tilt = max(max(map(abs, roll_deg)), max(map(abs, pitch_deg)))
    traced = [
        time,
        x,
        y,
        altitude,
        vertical_speed,
        roll_deg,
        pitch_deg,
        thrust,
        *motors,
    ]
    finite_trace = all(
        math.isfinite(sample) for samples in traced for sample in samples
    )
    minimum_motor = min(min(samples) for samples in motors)
    maximum_motor = max(max(samples) for samples in motors)

    metrics = {
        "simulated_seconds": time[-1],
        "simulation_wall_time_s": values["_simulation_wall_time_s"][0],
        "max_altitude_m": max(altitude),
        "max_tilt_deg": max_tilt,
        "hover_thrust_n": max(thrust),
        **{f"corner_{name}_error_m": error for name, error in corner_errors.items()},
        "final_altitude_m": altitude[-1],
        "final_vertical_speed_m_s": vertical_speed[-1],
        "final_horizontal_error_m": math.hypot(x[-1], y[-1]),
        "minimum_motor_command": minimum_motor,
        "maximum_motor_command": maximum_motor,
        "max_navigation_error_m": max(
            navigation_error[index] for index in active_indices
        )
        if active_indices
        else max(navigation_error),
        "max_controller_estimator_feedback_error_m": max(feedback_error),
        "rms_navigation_error_m": math.sqrt(
            sum(navigation_error[index] ** 2 for index in active_indices)
            / len(active_indices)
        )
        if active_indices
        else math.sqrt(sum(value**2 for value in navigation_error) / len(navigation_error)),
    }
    checks = {
        "finite_trace": finite_trace,
        "bounded_motors": minimum_motor >= -1.0e-9 and maximum_motor <= 1.0 + 1.0e-9,
        "takeoff": metrics["max_altitude_m"] >= CRUISE_ALTITUDE_M - 0.5,
        **{
            f"box_corner_{name}": error <= CORNER_TOLERANCE_M
            for name, error in corner_errors.items()
        },
        "bounded_tilt": max_tilt <= 45.0,
        "landing_altitude": metrics["final_altitude_m"] <= 0.30,
        "landing_speed": abs(metrics["final_vertical_speed_m_s"]) <= 0.50,
        "landing_position": metrics["final_horizontal_error_m"] <= 1.0,
        "bounded_navigation_error": metrics["max_navigation_error_m"]
        <= (2.0 if feedback_mode == "optical_flow" else 0.5),
    }

    if feedback_mode == "truth":
        all_corrections = (
            gps_position_accepted + gps_velocity_accepted + optical_flow_accepted
        )
        checks.update(
            {
                "controller_feedback_is_plant_truth": metrics["max_navigation_error_m"]
                <= 1.0e-9,
                "truth_baseline_is_distinct_from_estimator": metrics[
                    "max_controller_estimator_feedback_error_m"
                ]
                >= 0.02,
                "aiding_never_accepted": not any(
                    sample > 0.5 for sample in all_corrections
                ),
                "no_aiding_anchor_selected": all(
                    abs(sample) < 0.5 for sample in anchor_source
                ),
            }
        )
    else:
        if feedback_mode == "optical_flow":
            selected_correction = optical_flow_accepted
            unexpected_corrections = gps_position_accepted + gps_velocity_accepted
            expected_anchor = 3.0
        else:
            selected_correction = gps_position_accepted + gps_velocity_accepted
            unexpected_corrections = optical_flow_accepted
            expected_anchor = 2.0

        nees_time, nees = navigation_nees(values)
        nis_time, nis, nis_degrees = innovation_nis(values, int(expected_anchor))
        nees_lower, nees_upper = chi_square_95_bounds(6)
        if nis_degrees > 0:
            nis_lower, nis_upper = chi_square_95_bounds(nis_degrees)
        else:
            nis_lower, nis_upper = (0.0, 0.0)
        mean_nees = float(np.mean(nees)) if nees else 0.0
        mean_nis = float(np.mean(nis)) if nis else 0.0

        if feedback_mode == "optical_flow":
            noise_names = [
                *[
                    f"opticalFlowIntegratedNoise_rad[{index}]"
                    for index in range(1, 3)
                ],
                *[
                    f"opticalFlowGyroscopeIntegratedNoise_rad[{index}]"
                    for index in range(1, 3)
                ],
                "opticalFlowGroundDistanceNoise_m",
            ]
            covariance_names = [
                *[
                    f"estimator.opticalFlow.integratedLineOfSightCovariance_rad2[{index},{index}]"
                    for index in range(1, 3)
                ],
                *[
                    f"estimator.opticalFlow.integratedGyroscopeCovariance_rad2[{index},{index}]"
                    for index in range(1, 3)
                ],
                "estimator.opticalFlow.groundDistanceVariance_m2",
            ]
        else:
            noise_names = [
                *[f"gpsPositionNoise_m[{index}]" for index in range(1, 4)],
                *[f"gpsVelocityNoise_m_s[{index}]" for index in range(1, 4)],
            ]
            covariance_names = [
                *[
                    f"estimator.gps.positionCovarianceWorld_m2[{index},{index}]"
                    for index in range(1, 4)
                ],
                *[
                    f"estimator.gps.velocityCovarianceWorld_m2_s2[{index},{index}]"
                    for index in range(1, 4)
                ],
            ]
        measurement_period_s = 0.01 if feedback_mode == "optical_flow" else 0.1
        measurement_indices = native_sample_indices(values, measurement_period_s)
        raw_imu_period_s = signal(values, "imuSamplePeriod_s")[0]
        imu_indices = native_sample_indices(values, raw_imu_period_s)
        variance_ratios = []
        measurement_noise_samples = []
        for noise_name, covariance_name in zip(noise_names, covariance_names):
            noise_samples = np.array(signal(values, noise_name))[measurement_indices]
            measurement_noise_samples.append(noise_samples)
            assumed_variance = signal(values, covariance_name)[0]
            empirical_variance = float(np.var(noise_samples, ddof=1))
            variance_ratios.append(empirical_variance / assumed_variance)

        imu_variance_ratios = []
        imu_noise_samples = []
        for noise_prefix, variance_prefix in (
            (
                "imuAngularVelocityNoise_rad_s",
                "imuAngularVelocityNoiseVariance_rad2_s2",
            ),
            (
                "imuSpecificForceNoise_m_s2",
                "imuSpecificForceNoiseVariance_m2_s4",
            ),
        ):
            for component in range(1, 4):
                noise_samples = np.array(
                    signal(values, f"{noise_prefix}[{component}]")
                )[imu_indices]
                imu_noise_samples.append(noise_samples)
                assumed_variance = signal(
                    values, f"{variance_prefix}[{component}]"
                )[0]
                imu_variance_ratios.append(
                    float(np.var(noise_samples, ddof=1))
                    / assumed_variance
                )

        bias_increment_variance_ratios = []
        bias_increment_samples = []
        imu_sample_times = np.array(signal(values, "time_s"))[imu_indices]
        bias_increment_periods = np.diff(imu_sample_times) / raw_imu_period_s
        for bias_prefix, variance_prefix in (
            (
                "imuGyroscopeBias_rad_s",
                "imuGyroscopeBiasIncrementVariance_rad2_s2",
            ),
            (
                "imuAccelerometerBias_m_s2",
                "imuAccelerometerBiasIncrementVariance_m2_s4",
            ),
        ):
            for component in range(1, 4):
                increments = np.diff(
                    np.array(signal(values, f"{bias_prefix}[{component}]"))[
                        imu_indices
                    ]
                )
                assumed_increment_variance = signal(
                    values, f"{variance_prefix}[{component}]"
                )[0]
                normalized_increments = increments / np.sqrt(
                    assumed_increment_variance * bias_increment_periods
                )
                bias_increment_samples.append(normalized_increments)
                bias_increment_variance_ratios.append(
                    float(np.var(normalized_increments, ddof=1))
                )

        metrics.update(
            {
                "mean_navigation_nees_6dof": mean_nees,
                "navigation_nees_fraction_in_95_percent_interval": fraction_in_bounds(
                    nees, nees_lower, nees_upper
                ),
                "mean_innovation_nis": mean_nis,
                "innovation_nis_degrees_of_freedom": nis_degrees,
                "innovation_nis_fraction_in_95_percent_interval": fraction_in_bounds(
                    nis, nis_lower, nis_upper
                ),
                "minimum_empirical_to_assumed_noise_variance_ratio": min(
                    variance_ratios
                ),
                "maximum_empirical_to_assumed_noise_variance_ratio": max(
                    variance_ratios
                ),
                "minimum_imu_noise_variance_ratio": min(imu_variance_ratios),
                "maximum_imu_noise_variance_ratio": max(imu_variance_ratios),
                "minimum_imu_bias_increment_variance_ratio": min(
                    bias_increment_variance_ratios
                ),
                "maximum_imu_bias_increment_variance_ratio": max(
                    bias_increment_variance_ratios
                ),
                "maximum_measurement_noise_correlation": maximum_noise_correlation(
                    measurement_noise_samples
                ),
                "maximum_imu_noise_correlation": maximum_noise_correlation(
                    imu_noise_samples
                ),
                "maximum_imu_bias_increment_correlation": maximum_noise_correlation(
                    bias_increment_samples
                ),
                "nees_samples": len(nees_time),
                "nis_samples": len(nis_time),
            }
        )

        metrics["selected_aiding_corrections_observed"] = sum(
            sample > 0.5 for sample in selected_correction
        )
        checks.update(
            {
                "kalman_filter_initialized": estimator_initialized[-1] > 0.5,
                "kalman_prediction_accepted": any(
                    sample > 0.5 for sample in prediction_accepted
                ),
                "estimate_valid_during_flight": bool(active_indices)
                and all(estimate_valid[index] > 0.5 for index in active_indices),
                "selected_aiding_correction_observed": any(
                    sample > 0.5 for sample in selected_correction
                ),
                "unexpected_aiding_never_accepted": not any(
                    sample > 0.5 for sample in unexpected_corrections
                ),
                "expected_aiding_anchor_observed": any(
                    abs(sample - expected_anchor) < 0.5 for sample in anchor_source
                ),
                "controller_feedback_is_kalman_estimate": metrics[
                    "max_controller_estimator_feedback_error_m"
                ]
                <= 1.0e-9,
                "controller_feedback_is_not_plant_truth": metrics[
                    "max_navigation_error_m"
                ]
                >= 0.02,
                "noise_variance_matches_filter_assumption": all(
                    0.70 <= ratio <= 1.30 for ratio in variance_ratios
                ),
                "imu_noise_matches_filter_process_assumption": all(
                    0.70 <= ratio <= 1.30 for ratio in imu_variance_ratios
                ),
                "imu_bias_walk_matches_filter_process_assumption": all(
                    0.70 <= ratio <= 1.30
                    for ratio in bias_increment_variance_ratios
                ),
                "generated_noise_is_empirically_white": all(
                    maximum_noise_correlation(samples)
                    <= empirical_whiteness_limit(samples)
                    for samples in (
                        measurement_noise_samples,
                        imu_noise_samples,
                        bias_increment_samples,
                    )
                ),
                "nees_available": bool(nees),
                "nis_available": bool(nis),
                "mean_nees_near_expected_dimension": bool(nees)
                and 0.65 * 6.0 <= mean_nees <= 1.35 * 6.0,
                "nees_95_percent_coverage_is_plausible": bool(nees)
                and 0.80 <= fraction_in_bounds(
                    nees, nees_lower, nees_upper
                ) <= 0.995,
                "mean_nis_near_expected_dimension": bool(nis)
                and 0.70 * nis_degrees <= mean_nis <= 1.30 * nis_degrees,
                "nis_95_percent_coverage_is_plausible": bool(nis)
                and 0.80 <= fraction_in_bounds(
                    nis, nis_lower, nis_upper
                ) <= 0.995,
            }
        )

    return {"passed": all(checks.values()), "checks": checks, "metrics": metrics}


def max_track_difference_m(
    reference: dict[str, list[float]], candidate: dict[str, list[float]]
) -> float:
    max_difference = 0.0
    for name in ("position_m[1]", "position_m[2]", "position_m[3]"):
        reference_samples = signal(reference, name)
        candidate_samples = signal(candidate, name)
        count = min(len(reference_samples), len(candidate_samples))
        for index in range(count):
            max_difference = max(
                max_difference,
                abs(reference_samples[index] - candidate_samples[index]),
            )
    return max_difference


def baseline_comparison(
    truth: dict[str, list[float]],
    aided: dict[str, dict[str, list[float]]],
) -> dict[str, object]:
    """Compare each estimator-in-loop flight with the truth-feedback flight."""
    differences = {
        name: max_track_difference_m(truth, values)
        for name, values in aided.items()
    }
    checks = {
        f"{name}_track_matches_truth_baseline": difference
        <= (1.5 if name.endswith("optical_flow") else 0.75)
        for name, difference in differences.items()
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            f"{name}_max_track_difference_m": difference
            for name, difference in differences.items()
        },
    }


def algorithm_comparison(
    traces: dict[str, dict[str, list[float]]],
    reports: dict[str, dict[str, object]],
) -> dict[str, object]:
    """Compare paired ESKF/UKF accuracy, consistency, and whole-run cost."""
    eskf_rms = float(
        np.mean(
            [
                reports["eskf_optical_flow"]["metrics"]["rms_navigation_error_m"],
                reports["eskf_gps"]["metrics"]["rms_navigation_error_m"],
            ]
        )
    )
    ukf_rms = float(
        np.mean(
            [
                reports["ukf_optical_flow"]["metrics"]["rms_navigation_error_m"],
                reports["ukf_gps"]["metrics"]["rms_navigation_error_m"],
            ]
        )
    )
    eskf_runtime = float(
        np.mean(
            [
                reports["eskf_optical_flow"]["metrics"]["simulation_wall_time_s"],
                reports["eskf_gps"]["metrics"]["simulation_wall_time_s"],
            ]
        )
    )
    ukf_runtime = float(
        np.mean(
            [
                reports["ukf_optical_flow"]["metrics"]["simulation_wall_time_s"],
                reports["ukf_gps"]["metrics"]["simulation_wall_time_s"],
            ]
        )
    )
    track_differences = {
        aiding: max_track_difference_m(
            traces[f"eskf_{aiding}"], traces[f"ukf_{aiding}"]
        )
        for aiding in ("optical_flow", "gps")
    }
    runtime_ratio = ukf_runtime / max(eskf_runtime, 1.0e-9)
    all_algorithms_pass = all(
        bool(reports[name]["passed"])
        for name in (
            "eskf_optical_flow",
            "eskf_gps",
            "ukf_optical_flow",
            "ukf_gps",
        )
    )
    ukf_accuracy_improvement = 1.0 - ukf_rms / max(eskf_rms, 1.0e-9)
    recommend_ukf = (
        all_algorithms_pass
        and ukf_accuracy_improvement >= 0.20
        and runtime_ratio <= 2.0
    )
    recommendation = "UKF" if recommend_ukf else "ESKF"
    recommendation_basis = (
        "UKF selected: at least 20% lower paired RMS error for no more than "
        "2x whole-simulation cost."
        if recommend_ukf
        else "ESKF selected: the UKF does not provide at least 20% lower paired "
        "RMS error within 2x whole-simulation cost; the ESKF also propagates one "
        "nominal state per tick instead of 31 sigma points."
    )
    checks = {
        "paired_optical_flow_tracks_are_comparable":
            track_differences["optical_flow"] <= 1.0,
        "paired_gps_tracks_are_comparable": track_differences["gps"] <= 0.75,
        "both_algorithms_pass_both_aiding_modes": all_algorithms_pass,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "eskf_mean_rms_navigation_error_m": eskf_rms,
            "ukf_mean_rms_navigation_error_m": ukf_rms,
            "eskf_mean_simulation_wall_time_s": eskf_runtime,
            "ukf_mean_simulation_wall_time_s": ukf_runtime,
            "ukf_to_eskf_wall_time_ratio": runtime_ratio,
            "ukf_rms_error_improvement_fraction": ukf_accuracy_improvement,
            "eskf_nominal_propagations_per_tick": 1.0,
            "ukf_sigma_propagations_per_tick": 31.0,
            "optical_flow_eskf_ukf_track_difference_m":
                track_differences["optical_flow"],
            "gps_eskf_ukf_track_difference_m": track_differences["gps"],
        },
        "recommended_for_flight": recommendation,
        "recommendation_basis": recommendation_basis,
    }


def write_trace(values: dict[str, list[float]], path: Path) -> None:
    selected = {name: signal(values, name) for name in TRACE_NAMES}
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_NAMES)
        writer.writeheader()
        for index in range(len(selected["time_s"])):
            writer.writerow({name: selected[name][index] for name in TRACE_NAMES})


def read_trace(path: Path, runtime_s: float) -> dict[str, list[float]]:
    """Load a previously written qualification trace for local iteration."""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    values = {
        name: [float(row[name]) for row in rows]
        for name in TRACE_NAMES
    }
    values["_simulation_wall_time_s"] = [runtime_s]
    return values


def previous_runtimes() -> dict[str, float]:
    report_path = ARTIFACT_DIR / "waypoint-report.json"
    if not report_path.is_file():
        return {}
    report = json.loads(report_path.read_text(encoding="utf-8"))
    return {
        name: float(report[name]["metrics"]["simulation_wall_time_s"])
        for name in SCENARIOS
        if name in report and "simulation_wall_time_s" in report[name]["metrics"]
    }


def plot_missions(
    truth: dict[str, list[float]],
    optical_flow: dict[str, list[float]],
    gps: dict[str, list[float]],
    ukf_optical_flow: dict[str, list[float]] | None = None,
    ukf_gps: dict[str, list[float]] | None = None,
) -> Path:
    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    figure.suptitle("RDD2 Waypoint Navigation Qualification", fontsize=16)

    corner_x = [0.0, BOX_SIDE_M, BOX_SIDE_M, 0.0, 0.0]
    corner_y = [0.0, 0.0, BOX_SIDE_M, BOX_SIDE_M, 0.0]
    axes[0, 0].plot(corner_x, corner_y, "k--", label="commanded box")
    axes[0, 0].plot(
        signal(truth, "position_m[1]"),
        signal(truth, "position_m[2]"),
        label="truth-feedback baseline",
    )
    axes[0, 0].plot(
        signal(optical_flow, "position_m[1]"),
        signal(optical_flow, "position_m[2]"),
        linestyle="--",
        label="optical-flow ESKF",
    )
    axes[0, 0].plot(
        signal(gps, "position_m[1]"),
        signal(gps, "position_m[2]"),
        linestyle=":",
        label="GPS ESKF",
    )
    if ukf_optical_flow is not None:
        axes[0, 0].plot(
            signal(ukf_optical_flow, "position_m[1]"),
            signal(ukf_optical_flow, "position_m[2]"),
            linestyle="-.",
            label="optical-flow UKF",
        )
    if ukf_gps is not None:
        axes[0, 0].plot(
            signal(ukf_gps, "position_m[1]"),
            signal(ukf_gps, "position_m[2]"),
            linestyle=(0, (1, 1)),
            label="GPS UKF",
        )
    axes[0, 0].set_title("Ground Track")
    axes[0, 0].set_xlabel("east [m]")
    axes[0, 0].set_ylabel("north [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True)
    axes[0, 0].legend(loc="best")

    axes[0, 1].plot(
        signal(truth, "time_s"),
        signal(truth, "position_m[3]"),
        label="truth-feedback altitude",
    )
    axes[0, 1].plot(
        signal(truth, "time_s"),
        signal(truth, "avionics.reference.position[3]"),
        linestyle="--",
        label="target",
    )
    axes[0, 1].plot(
        signal(optical_flow, "time_s"),
        signal(optical_flow, "navigationError_m"),
        label="flow navigation error",
    )
    axes[0, 1].plot(
        signal(gps, "time_s"),
        signal(gps, "navigationError_m"),
        label="GPS navigation error",
    )
    axes[0, 1].set_title("Altitude and Navigation Error")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("up [m] / error [m]")
    axes[0, 1].set_xlim(0.0, MISSION_PLOT_END_S)
    axes[0, 1].grid(True)
    axes[0, 1].legend(loc="best")

    axes[1, 0].plot(
        signal(gps, "geodetic[2]"),
        signal(gps, "geodetic[1]"),
        label="GPS mission track",
    )
    axes[1, 0].set_title("Geodetic Track (local pose lifted through origin)")
    axes[1, 0].set_xlabel("longitude [deg]")
    axes[1, 0].set_ylabel("latitude [deg]")
    axes[1, 0].grid(True)
    axes[1, 0].ticklabel_format(useOffset=False)

    for motor in range(4):
        axes[1, 1].plot(
            signal(optical_flow, "time_s"),
            signal(optical_flow, f"motorCommand[{motor + 1}]"),
            label=f"motor {motor}",
        )
    axes[1, 1].set_title("Motor Commands (optical-flow ESKF)")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("normalized command")
    axes[1, 1].set_xlim(0.0, MISSION_PLOT_END_S)
    axes[1, 1].grid(True)
    axes[1, 1].legend(loc="best", ncols=2)

    path = ARTIFACT_DIR / "rdd2-waypoint-summary.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(f"wrote {path}")
    return path


def plot_state_histories(
    truth: dict[str, list[float]],
    optical_flow: dict[str, list[float]],
    gps: dict[str, list[float]],
    ukf_optical_flow: dict[str, list[float]] | None = None,
    ukf_gps: dict[str, list[float]] | None = None,
) -> list[Path]:
    modes = {
        "truth feedback": truth,
        "optical-flow ESKF": optical_flow,
        "GPS ESKF": gps,
    }
    if ukf_optical_flow is not None:
        modes["optical-flow UKF"] = ukf_optical_flow
    if ukf_gps is not None:
        modes["GPS UKF"] = ukf_gps
    paths: list[Path] = []
    for quantity, units, labels in (
        ("position_m", "m", ("east position", "north position", "altitude")),
        ("velocity_m_s", "m/s", ("east velocity", "north velocity", "up velocity")),
    ):
        figure, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        title = "Position" if quantity == "position_m" else "Velocity"
        figure.suptitle(f"RDD2 {title} History", fontsize=16)
        for component, axis in enumerate(axes, start=1):
            for mode_label, values in modes.items():
                axis.plot(
                    signal(values, "time_s"),
                    signal(values, f"{quantity}[{component}]"),
                    label=mode_label,
                )
            axis.set_ylabel(f"{labels[component - 1]} [{units}]")
            axis.set_xlim(0.0, MISSION_PLOT_END_S)
            axis.grid(True)
            axis.legend(loc="best")
        axes[-1].set_xlabel("time [s]")
        path = (
            ARTIFACT_DIR
            / f"rdd2-{quantity.replace('_m_s', '').replace('_m', '')}-history.png"
        )
        figure.tight_layout()
        figure.savefig(path, dpi=160)
        plt.close(figure)
        print(f"wrote {path}")
        paths.append(path)
    return paths


def plot_consistency(
    optical_flow: dict[str, list[float]], gps: dict[str, list[float]]
) -> Path:
    figure, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    figure.suptitle("RDD2 ESKF Covariance Consistency", fontsize=16)
    for column, (label, values, source) in enumerate(
        (("optical flow", optical_flow, 3), ("GPS", gps, 2))
    ):
        nees_time, nees = navigation_nees(values)
        nees_lower, nees_upper = chi_square_95_bounds(6)
        axes[0, column].plot(nees_time, nees, linewidth=0.8, label="6-DoF NEES")
        axes[0, column].axhspan(
            nees_lower,
            nees_upper,
            color="tab:green",
            alpha=0.15,
            label="95% χ² interval",
        )
        axes[0, column].axhline(
            6.0, color="black", linestyle="--", label="expected mean"
        )
        axes[0, column].set_title(f"{label}: navigation NEES")
        axes[0, column].set_ylabel("NEES")
        axes[0, column].set_yscale("log")
        axes[0, column].set_xlim(0.0, MISSION_PLOT_END_S)
        axes[0, column].grid(True)
        axes[0, column].legend(loc="best")

        nis_time, nis, degrees = innovation_nis(values, source)
        nis_lower, nis_upper = chi_square_95_bounds(degrees)
        axes[1, column].plot(nis_time, nis, ".", markersize=2.5, label="NIS")
        axes[1, column].axhspan(
            nis_lower, nis_upper, color="tab:green", alpha=0.15, label="95% χ² interval"
        )
        axes[1, column].axhline(
            degrees, color="black", linestyle="--", label="expected mean"
        )
        axes[1, column].set_title(f"{label}: innovation NIS ({degrees} DoF)")
        axes[1, column].set_xlabel("time [s]")
        axes[1, column].set_ylabel("NIS")
        axes[1, column].set_xlim(0.0, MISSION_PLOT_END_S)
        axes[1, column].grid(True)
        axes[1, column].legend(loc="best")

    path = ARTIFACT_DIR / "rdd2-nees-nis-consistency.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(f"wrote {path}")
    return path


def plot_estimator_consistency(
    traces: dict[str, dict[str, list[float]]],
) -> Path:
    """Put ESKF and UKF NEES/NIS on one directly viewable CI figure."""
    cases = (
        ("ESKF optical", traces["eskf_optical_flow"], 3),
        ("UKF optical", traces["ukf_optical_flow"], 3),
        ("ESKF GPS", traces["eskf_gps"], 2),
        ("UKF GPS", traces["ukf_gps"], 2),
    )
    figure, axes = plt.subplots(2, 4, figsize=(22, 9), constrained_layout=True)
    figure.suptitle("RDD2 StrapdownINS Estimator Consistency", fontsize=16)
    for column, (label, values, source) in enumerate(cases):
        nees_time, nees = navigation_nees(values)
        nees_lower, nees_upper = chi_square_95_bounds(6)
        axes[0, column].plot(nees_time, nees, linewidth=0.7)
        axes[0, column].axhspan(nees_lower, nees_upper, color="tab:green", alpha=0.15)
        axes[0, column].axhline(6.0, color="black", linestyle="--")
        axes[0, column].set_title(f"{label}: 6-DoF NEES")
        axes[0, column].set_yscale("log")
        axes[0, column].set_xlim(0.0, MISSION_PLOT_END_S)
        axes[0, column].grid(True)

        nis_time, nis, degrees = innovation_nis(values, source)
        nis_lower, nis_upper = chi_square_95_bounds(degrees)
        axes[1, column].plot(nis_time, nis, ".", markersize=2.0)
        axes[1, column].axhspan(nis_lower, nis_upper, color="tab:green", alpha=0.15)
        axes[1, column].axhline(degrees, color="black", linestyle="--")
        axes[1, column].set_title(f"{label}: NIS ({degrees} DoF)")
        axes[1, column].set_xlabel("time [s]")
        axes[1, column].set_xlim(0.0, MISSION_PLOT_END_S)
        axes[1, column].grid(True)
    axes[0, 0].set_ylabel("NEES")
    axes[1, 0].set_ylabel("NIS")
    path = ARTIFACT_DIR / "rdd2-estimator-consistency.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(f"wrote {path}")
    return path


def plot_estimator_comparison(
    reports: dict[str, dict[str, object]],
    comparison: dict[str, object],
) -> Path:
    """Summarize accuracy, consistency, and measured cost by algorithm."""
    names = ("eskf_optical_flow", "ukf_optical_flow", "eskf_gps", "ukf_gps")
    labels = ("ESKF\noptical", "UKF\noptical", "ESKF\nGPS", "UKF\nGPS")
    colors = ("tab:blue", "tab:orange", "tab:blue", "tab:orange")
    rms = [float(reports[name]["metrics"]["rms_navigation_error_m"]) for name in names]
    runtimes = [float(reports[name]["metrics"]["simulation_wall_time_s"]) for name in names]
    nees = [float(reports[name]["metrics"]["mean_navigation_nees_6dof"]) for name in names]
    normalized_nis = [
        float(reports[name]["metrics"]["mean_innovation_nis"])
        / float(reports[name]["metrics"]["innovation_nis_degrees_of_freedom"])
        for name in names
    ]
    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    figure.suptitle(
        "RDD2 ESKF vs UKF — recommended: "
        + str(comparison["recommended_for_flight"]),
        fontsize=16,
    )
    for axis, samples, title, units in (
        (axes[0, 0], rms, "RMS navigation error", "m"),
        (axes[0, 1], runtimes, "Whole-mission simulation time", "s"),
        (axes[1, 0], nees, "Mean 6-DoF NEES (ideal mean 6)", "NEES"),
        (axes[1, 1], normalized_nis, "Mean NIS / measurement DoF (ideal 1)", "ratio"),
    ):
        axis.bar(labels, samples, color=colors)
        axis.set_title(title)
        axis.set_ylabel(units)
        axis.grid(True, axis="y")
    axes[1, 0].axhline(6.0, color="black", linestyle="--")
    axes[1, 1].axhline(1.0, color="black", linestyle="--")
    path = ARTIFACT_DIR / "rdd2-estimator-comparison.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(f"wrote {path}")
    return path


def write_reports(report: dict[str, object], plot_paths: list[Path]) -> None:
    report_path = ARTIFACT_DIR / "waypoint-report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# RDD2 waypoint navigation qualification",
        "",
        f"Result: **{'PASS' if report['passed'] else 'FAIL'}**",
        "",
        f"Diagnostic interpretation: **{report['diagnostic_interpretation']}**",
        "",
    ]
    for mode in (
        "truth",
        "eskf_optical_flow",
        "eskf_gps",
        "ukf_optical_flow",
        "ukf_gps",
        "baseline_comparison",
        "algorithm_comparison",
    ):
        section = report[mode]
        assert isinstance(section, dict)
        checks = section["checks"]
        metrics = section["metrics"]
        assert isinstance(checks, dict) and isinstance(metrics, dict)
        lines.extend([f"## {mode}", "", "| Check | Result |", "| --- | --- |"])
        lines.extend(
            f"| `{name}` | {'PASS' if passed else 'FAIL'} |"
            for name, passed in checks.items()
        )
        lines.extend(["", "| Metric | Value |", "| --- | ---: |"])
        lines.extend(f"| `{name}` | {value:.6g} |" for name, value in metrics.items())
        lines.append("")
    lines.extend(
        [
            "## Mission plot",
            "",
            *[f"- `{plot_path.name}`" for plot_path in plot_paths],
            "",
            "The truth-feedback baseline isolates the controller and plant. The "
            "ESKF and UKF optical-flow/GPS missions fly on their estimator output "
            "with identical sensor noise and are compared with that baseline.",
            "",
            "NEES/NIS are time-correlated diagnostics from one deterministic "
            "mission, not a formal Monte Carlo consistency proof. Green bands "
            "are per-sample 95% chi-square intervals; the tables also report "
            "mean-versus-dimension and empirical interval coverage.",
            "",
            f"Flight recommendation: **{report['recommended_for_flight']}**.",
            str(report["algorithm_comparison"]["recommendation_basis"]),
        ]
    )
    markdown = "\n".join(lines) + "\n"
    (ARTIFACT_DIR / "waypoint-summary.md").write_text(markdown, encoding="utf-8")

    image_html = "".join(
        "<h2>" + html.escape(plot_path.stem.replace("-", " ").title()) + "</h2>"
        "<img src='data:image/png;base64,"
        + base64.b64encode(plot_path.read_bytes()).decode("ascii")
        + "' alt='"
        + html.escape(plot_path.stem)
        + "'>"
        for plot_path in plot_paths
    )
    (ARTIFACT_DIR / "waypoint-report.html").write_text(
        "<!doctype html><meta charset='utf-8'>"
        "<title>RDD2 Waypoint Navigation Qualification</title>"
        "<style>body{font-family:system-ui,sans-serif;margin:2rem;color:#111827}"
        "pre{white-space:pre-wrap}img{max-width:100%;height:auto;"
        "border:1px solid #d1d5db}</style>"
        f"<body><pre>{html.escape(markdown)}</pre>"
        f"{image_html}"
        "</body>\n",
        encoding="utf-8",
    )


def main() -> None:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    requested = {
        name.strip()
        for name in os.environ.get("RDD2_ONLY_SCENARIOS", "").split(",")
        if name.strip()
    }
    cached_runtimes = previous_runtimes()
    traces: dict[str, dict[str, list[float]]] = {}
    for mode, scenario in SCENARIOS.items():
        trace_path = ARTIFACT_DIR / f"waypoint-{mode}.csv"
        if requested and mode not in requested:
            print(f"replaying {trace_path.name}", flush=True)
            traces[mode] = read_trace(trace_path, cached_runtimes.get(mode, 0.0))
        else:
            traces[mode] = simulate(scenario)
            write_trace(traces[mode], trace_path)

    truth_report = evaluate(traces["truth"], "truth")
    aided_reports = {
        "eskf_optical_flow": evaluate(traces["eskf_optical_flow"], "optical_flow"),
        "eskf_gps": evaluate(traces["eskf_gps"], "gps"),
        "ukf_optical_flow": evaluate(traces["ukf_optical_flow"], "optical_flow"),
        "ukf_gps": evaluate(traces["ukf_gps"], "gps"),
    }
    comparison = baseline_comparison(
        traces["truth"],
        {name: traces[name] for name in aided_reports},
    )
    estimator_comparison = algorithm_comparison(traces, aided_reports)
    plot_paths = [
        plot_missions(
            traces["truth"], traces["eskf_optical_flow"], traces["eskf_gps"],
            traces["ukf_optical_flow"], traces["ukf_gps"]
        ),
        *plot_state_histories(
            traces["truth"], traces["eskf_optical_flow"], traces["eskf_gps"],
            traces["ukf_optical_flow"], traces["ukf_gps"]
        ),
        plot_consistency(traces["eskf_optical_flow"], traces["eskf_gps"]),
        plot_estimator_consistency(traces),
        plot_estimator_comparison(aided_reports, estimator_comparison),
    ]

    if not truth_report["passed"]:
        diagnostic_interpretation = "controller/plant/planning baseline failure"
    elif not all(bool(section["passed"]) for section in aided_reports.values()):
        diagnostic_interpretation = "estimator or aiding-path failure"
    elif not comparison["passed"] or not estimator_comparison["passed"]:
        diagnostic_interpretation = "estimator-to-baseline trajectory mismatch"
    else:
        diagnostic_interpretation = "all feedback paths passed"

    report = {
        "passed": truth_report["passed"]
        and all(bool(section["passed"]) for section in aided_reports.values())
        and comparison["passed"]
        and estimator_comparison["passed"],
        "diagnostic_interpretation": diagnostic_interpretation,
        "recommended_for_flight": estimator_comparison["recommended_for_flight"],
        "truth": truth_report,
        **aided_reports,
        "baseline_comparison": comparison,
        "algorithm_comparison": estimator_comparison,
        "provenance": {
            "mission": {
                "path": "Vehicles/Rdd2/Test/WaypointMission.mo",
                "sha256": source_digest(MODEL_DIR / "WaypointMission.mo"),
            },
            "truth_baseline": {
                "path": "Vehicles/Rdd2/Test/TruthWaypointMission.mo",
                "sha256": source_digest(MODEL_DIR / "TruthWaypointMission.mo"),
            },
            "guidance": {
                "path": "Vehicles/Rdd2/GuidanceController.mo",
                "sha256": source_digest(
                    ROOT / "Vehicles" / "Rdd2" / "GuidanceController.mo"
                ),
            },
            "geodesy": {
                "path": "Geodesy/package.mo",
                "sha256": source_digest(ROOT / "Geodesy" / "package.mo"),
            },
            "estimator": {
                "eskf_path": "Estimation/StrapdownINS/ESKF/Estimator.mo",
                "eskf_sha256": source_digest(
                    ROOT / "Estimation" / "StrapdownINS" / "ESKF" / "Estimator.mo"
                ),
                "ukf_path": "Estimation/StrapdownINS/UKF/Estimator.mo",
                "ukf_sha256": source_digest(
                    ROOT / "Estimation" / "StrapdownINS" / "UKF" / "Estimator.mo"
                ),
            },
            "rumoca": getattr(rum, "__version__", "workspace"),
        },
    }
    write_reports(report, plot_paths)
    print(f"wrote {ARTIFACT_DIR / 'waypoint-summary.md'}")
    print(f"wrote {ARTIFACT_DIR / 'waypoint-report.json'}")
    if not report["passed"]:
        failures = []
        for mode in (
            "truth",
            "eskf_optical_flow",
            "eskf_gps",
            "ukf_optical_flow",
            "ukf_gps",
            "baseline_comparison",
            "algorithm_comparison",
        ):
            section = report[mode]
            assert isinstance(section, dict)
            failures.extend(
                f"{mode}.{name}"
                for name, passed in section["checks"].items()
                if not passed
            )
        raise SystemExit("RDD2 waypoint qualification failed: " + ", ".join(failures))


if __name__ == "__main__":
    main()
