"""Reconstruct native bad-IMU checks and distinguish trigger residuals from overrides."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from manifest import digest


FIELDS = (
    "publication_us",
    "fusion_us",
    "imu_sample_ms",
    "gps_sample_ms",
    "baro_sample_ms",
    "height_error_m",
    "vertical_velocity_error_m_s",
    "height_observation_variance",
    "velocity_observation_variance",
    "threshold_multiplier",
    "time_since_bad_ms",
    "last_bad_ms",
    "last_good_ms",
    "bad_imu_data",
    "estimated_velocity_down_m_s",
    "gps_velocity_down_m_s",
    "observed_velocity_down_m_s",
    "estimated_position_down_m",
    "observed_position_down_m",
    "on_ground",
    "takeoff_expected",
    "touchdown_expected",
    "aiding_mode",
    "height_source",
    "gps_data_to_fuse",
)
HOLD_MS = 10000


def trigger(row):
    return (
        row["height_error_m"] * row["vertical_velocity_error_m_s"] > 0
        and row["height_error_m"] ** 2
        > row["threshold_multiplier"] * row["height_observation_variance"]
        and row["vertical_velocity_error_m_s"] ** 2
        > row["threshold_multiplier"] * row["velocity_observation_variance"]
    )


def event(before, after):
    return {
        "publication_s": before["publication_us"] / 1e6,
        "fusion_s": before["fusion_us"] / 1e6,
        "gps_sample_s": before["gps_sample_ms"] / 1000,
        "baro_sample_s": before["baro_sample_ms"] / 1000,
        "height_error_m": before["height_error_m"],
        "vertical_velocity_error_m_s": before["vertical_velocity_error_m_s"],
        "height_error_observation_sigma": before["height_error_m"]
        / np.sqrt(before["height_observation_variance"]),
        "velocity_error_observation_sigma": before["vertical_velocity_error_m_s"]
        / np.sqrt(before["velocity_observation_variance"]),
        "threshold_multiplier": before["threshold_multiplier"],
        "time_since_bad_ms": before["time_since_bad_ms"],
        "last_bad_ms_before": before["last_bad_ms"],
        "last_bad_ms_after": after["last_bad_ms"],
        "bad_imu_before": bool(before["bad_imu_data"]),
        "bad_imu_after": bool(after["bad_imu_data"]),
        "trigger_condition": trigger(before),
        "on_ground": bool(before["on_ground"]),
        "takeoff_expected": bool(before["takeoff_expected"]),
        "aiding_mode": int(before["aiding_mode"]),
        "height_source": int(before["height_source"]),
        "gps_data_to_fuse": bool(before["gps_data_to_fuse"]),
    }


def diagnose(path):
    with path.open() as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != ("stage", *FIELDS):
            raise ValueError("Require complete native pre/post integrity observations")
        rows = list(reader)
    if not rows or len(rows) % 2:
        raise ValueError("Require complete before/after pairs")
    events = []
    previous_time = -1
    for index in range(0, len(rows), 2):
        before_raw, after_raw = rows[index : index + 2]
        if before_raw["stage"] != "before" or after_raw["stage"] != "after":
            raise ValueError("Reordered integrity snapshots")
        before, after = (
            {key: float(row[key]) for key in FIELDS} for row in (before_raw, after_raw)
        )
        if not all(
            np.isfinite(value) for row in (before, after) for value in row.values()
        ):
            raise ValueError("Nonfinite integrity observation")
        if (
            before["publication_us"] < previous_time
            or before["fusion_us"] > before["publication_us"]
        ):
            raise ValueError("Invalid native integrity epoch")
        previous_time = before["publication_us"]
        if (
            min(
                before["height_observation_variance"],
                before["velocity_observation_variance"],
            )
            <= 0
        ):
            raise ValueError("Invalid integrity observation variance")
        for key in FIELDS:
            if (
                key
                not in (
                    "last_bad_ms",
                    "last_good_ms",
                    "bad_imu_data",
                    "estimated_velocity_down_m_s",
                )
                and before[key] != after[key]
            ):
                raise ValueError("Unexpected mutation inside integrity check")
        elapsed = (int(before["imu_sample_ms"]) - int(before["last_bad_ms"])) % (2**32)
        threshold = 9 if elapsed > 2 * HOLD_MS else 4 if elapsed > 1.5 * HOLD_MS else 1
        condition = trigger(before)
        expected_bad = before["imu_sample_ms"] if condition else before["last_bad_ms"]
        expected_good = before["last_good_ms"] if condition else before["imu_sample_ms"]
        expected_velocity = (
            before["gps_velocity_down_m_s"]
            if elapsed < HOLD_MS
            else before["estimated_velocity_down_m_s"]
        )
        if (
            before["time_since_bad_ms"] != elapsed
            or before["threshold_multiplier"] != threshold
            or after["last_bad_ms"] != expected_bad
            or after["last_good_ms"] != expected_good
            or bool(after["bad_imu_data"]) != (elapsed < HOLD_MS)
            or after["estimated_velocity_down_m_s"] != expected_velocity
        ):
            raise ValueError(
                "Native integrity branch differs from reconstructed operation"
            )
        if (
            abs(
                before["height_error_m"]
                - (
                    before["estimated_position_down_m"]
                    - before["observed_position_down_m"]
                )
            )
            > 1e-12
        ):
            raise ValueError("Height trigger residual differs from pre-override state")
        if (
            abs(
                before["vertical_velocity_error_m_s"]
                - (
                    before["estimated_velocity_down_m_s"]
                    - before["observed_velocity_down_m_s"]
                )
            )
            > 1e-12
        ):
            raise ValueError(
                "Velocity trigger residual differs from pre-override state"
            )
        events.append(event(before, after))
    triggered = [row for row in events if row["trigger_condition"]]
    activated = [
        row for row in events if not row["bad_imu_before"] and row["bad_imu_after"]
    ]
    return {
        "trace_sha256": digest(path),
        "checked_pairs": len(events),
        "publication_window_s": [
            events[0]["publication_s"],
            events[-1]["publication_s"],
        ],
        "trigger_conditions": len(triggered),
        "initial_trace_event": events[0],
        "first_trigger_condition": triggered[0] if triggered else None,
        "first_false_to_true_activation": activated[0] if activated else None,
        "activations": activated,
        "trigger_events": triggered,
        "bad_imu_after_checks": sum(row["bad_imu_after"] for row in events),
        "scope": (
            "Read-only native vertical integrity branch before GPS velocity override; "
            "every recorded branch and override is reconstructed. Residuals are "
            "normalized by observation variance used in native detection, not total "
            "innovation variance; these values are not Kalman NIS. No native repair "
            "or causal counterfactual trajectory is substituted."
        ),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    args.output.write_text(
        json.dumps(diagnose(args.trace), indent=2, allow_nan=False) + "\n"
    )
