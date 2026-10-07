#!/usr/bin/env python3
"""Score against independent truth, retaining invalid outputs and all axes."""

import argparse
import json
from pathlib import Path
import numpy as np

POSITION = ("e_m", "n_m", "u_m")
VELOCITY = ("ve_m_s", "vn_m_s", "vu_m_s")
QUATERNION = ("qw", "qx", "qy", "qz")


def read(path):
    d = np.atleast_1d(np.genfromtxt(path, names=True, delimiter=","))
    if not len(d) or not np.all(np.diff(d["t_s"]) > 0):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    return d


def errors(estimate, truth):
    t = estimate["t_s"]
    if t[0] < truth["t_s"][0] or t[-1] > truth["t_s"][-1]:
        raise ValueError("Estimate outside truth support; extrapolation forbidden")

    def aligned(names):
        return np.column_stack([np.interp(t, truth["t_s"], truth[k]) for k in names])

    def values(names):
        return np.column_stack([estimate[k] for k in names])

    p = values(POSITION) - aligned(POSITION)
    v = values(VELOCITY) - aligned(VELOCITY)
    q = values(QUATERNION)
    qt = aligned(QUATERNION)
    q = q / np.linalg.norm(q, axis=1)[:, None]
    qt = qt / np.linalg.norm(qt, axis=1)[:, None]
    angle = 2 * np.arccos(np.clip(np.abs(np.sum(q * qt, axis=1)), 0, 1))

    def yaw(x):
        w, i, j, k = x.T
        return np.arctan2(2 * (w * k + i * j), 1 - 2 * (j * j + k * k))

    dy = yaw(q) - yaw(qt)
    dy = np.arctan2(np.sin(dy), np.cos(dy))
    return p, v, np.rad2deg(angle), np.rad2deg(dy)


def metrics(d, truth, start, end):
    selected = d[(d["t_s"] >= start) & (d["t_s"] < end)]
    if not len(selected):
        raise ValueError("No outputs in scoring window")
    p, v, att, yaw = errors(selected, truth)
    finite = np.isfinite(np.column_stack((p, v, att, yaw))).all(axis=1)
    result = dict(
        start_s=start,
        end_s=end,
        rows=len(selected),
        row_coverage=min(1.0, len(selected) / ((end - start) * 100)),
        position_valid_fraction=float(np.mean(selected["pos_valid"])),
        attitude_valid_fraction=float(np.mean(selected["att_valid"])),
        finite_fraction=float(np.mean(finite)),
        max_output_gap_s=float(max(np.diff(selected["t_s"]))),
    )
    # Invalid finite states remain in every accuracy metric. A nonfinite state
    # makes the accuracy result fail, rather than disappearing from its RMSE.
    if finite.all():
        result.update(
            horizontal_position_rmse_m=float(
                np.sqrt(np.mean(np.sum(p[:, :2] ** 2, axis=1)))
            ),
            vertical_position_rmse_m=float(np.sqrt(np.mean(p[:, 2] ** 2))),
            position_3d_rmse_m=float(np.sqrt(np.mean(np.sum(p**2, axis=1)))),
            velocity_3d_rmse_m_s=float(np.sqrt(np.mean(np.sum(v**2, axis=1)))),
            attitude_rmse_deg=float(np.sqrt(np.mean(att**2))),
            yaw_rmse_deg=float(np.sqrt(np.mean(yaw**2))),
            max_horizontal_error_m=float(np.max(np.linalg.norm(p[:, :2], axis=1))),
            terminal_horizontal_error_m=float(np.linalg.norm(p[-1, :2])),
        )
    for k in selected.dtype.names:
        if k.endswith("_fused"):
            result[k + "_accepted_hz"] = float(
                np.count_nonzero(selected[k]) / (end - start)
            )
        elif k.endswith("_active"):
            result[k + "_fraction"] = float(np.mean(selected[k]))
    if "step_status" in selected.dtype.names:
        result["nonzero_step_status_rows"] = int(
            np.count_nonzero(selected["step_status"])
        )
    if "prediction_accepted" in selected.dtype.names:
        result["prediction_accepted_fraction"] = float(
            np.mean(selected["prediction_accepted"])
        )
    return result


def transition(d, truth):
    d = d[(d["t_s"] >= 25) & (d["t_s"] < 46)]
    p, *_ = errors(d, truth)
    h = np.linalg.norm(p[:, :2], axis=1)
    result = {}
    flag = "gps_fused" if "gps_fused" in d.dtype.names else "gps_active"
    returned = np.flatnonzero((d["t_s"] >= 40) & (d[flag] > 0.5))
    result["first_gps_use_after_return_s"] = (
        float(d["t_s"][returned[0]] - 40) if len(returned) else None
    )
    result["recovery_to_025m_for_1s_s"] = None
    for i in np.flatnonzero(d["t_s"] >= 40):
        finish = np.searchsorted(d["t_s"], d["t_s"][i] + 1)
        if finish < len(d) and np.all(h[i : finish + 1] < 0.25):
            result["recovery_to_025m_for_1s_s"] = float(d["t_s"][i] - 40)
            break
    around = (d["t_s"] >= 39) & (d["t_s"] < 45)
    result["max_return_error_jump_m"] = float(
        np.max(np.linalg.norm(np.diff(p[around], axis=0), axis=1))
    )
    return result


def score(directory, data_root, lower_rates=False):
    records = json.loads((directory / "runs.json").read_text())
    results = []
    for record in records:
        tag = f"{record['speed']}_{record['seed']}"
        data = data_root / ("lower_" + tag if lower_rates else tag)
        truth = read(data / "truth.csv")
        d = read(directory / record["csv"])
        result = dict(
            **record, profile="25/25/25 Hz" if lower_rates else "100/50/50 Hz"
        )
        result["flight"] = metrics(d, truth, 13, 60)
        result["outage_window"] = metrics(d, truth, 25, 40)
        result["before_outage"] = metrics(d, truth, 18, 25)
        result["after_return"] = metrics(d, truth, 40, 60)
        aligned = np.flatnonzero(d["att_valid"] > 0.5)
        result["first_attitude_valid_s"] = (
            float(d["t_s"][aligned[0]]) if len(aligned) else None
        )
        if record["case"] == "transition":
            result["transition"] = transition(d, truth)
        results.append(result)
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("results", type=Path)
    p.add_argument("data_root", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--lower-rates", action="store_true")
    a = p.parse_args()
    result = score(a.results, a.data_root, a.lower_rates)
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Scored {len(result)} runs against independent truth.")
