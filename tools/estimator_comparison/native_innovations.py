"""Summarize scalar native innovation statistics without concealing invalid rows."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


WINDOWS = (
    ("before_takeoff", 10, 13),
    ("flight", 13, 59.7),
    ("outage", 25, 40),
    ("after_return", 40, 59.7),
)


def summarize(rows):
    residual = np.array([float(row["innovation"]) for row in rows])
    variance = np.array([float(row["innovation_variance"]) for row in rows])
    noise = np.array([float(row["observation_variance"]) for row in rows])
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        scalar_nis = residual**2 / variance
    valid = (
        np.isfinite(residual)
        & np.isfinite(variance)
        & (variance > 0)
        & np.isfinite(noise)
        & (noise >= 0)
        & np.isfinite(scalar_nis)
    )
    nis = scalar_nis[valid]
    return dict(
        rows=len(rows),
        invalid_rows=int((~valid).sum()),
        valid=bool(valid.all()),
        fused_rows=sum(int(row["fused"]) == 1 for row in rows),
        rejected_fusion_rows=sum(int(row["fused"]) == 0 for row in rows),
        fusion_not_yet_decided_rows=sum(int(row["fused"]) == -1 for row in rows),
        unknown_native_sample_time_rows=sum(int(row["sample_us"]) == 0 for row in rows),
        median_observation_variance=float(np.median(noise[valid]))
        if valid.any()
        else None,
        minimum_observation_variance=float(noise[valid].min()) if valid.any() else None,
        maximum_observation_variance=float(noise[valid].max()) if valid.any() else None,
        median_innovation_variance=float(np.median(variance[valid]))
        if valid.any()
        else None,
        mean_scalar_nis=float(nis.mean()) if len(nis) else None,
        median_scalar_nis=float(np.median(nis)) if len(nis) else None,
        p95_scalar_nis=float(np.quantile(nis, 0.95)) if len(nis) else None,
        maximum_scalar_nis=float(nis.max()) if len(nis) else None,
        fraction_below_chisq_1d_95=float((nis < 3.841458820694124).mean())
        if len(nis)
        else None,
    )


def check(args):
    clock_offset_s = 1.0 if args.filter == "px4" else 0.0
    with args.innovations.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("Missing actual native innovation observations")
    fields = {
        "publication_us",
        "fusion_us",
        "sample_us",
        "sensor",
        "axis",
        "stage",
        "innovation",
        "innovation_variance",
        "observation_variance",
        "gate_ratio",
        "fused",
        "navigation",
    }
    if set(rows[0]) != fields:
        raise ValueError("Incomplete native innovation schema")
    groups = {}
    for row in rows:
        publication, fusion, sample = (
            int(row[key]) for key in ("publication_us", "fusion_us", "sample_us")
        )
        if min(publication, fusion, sample) < 0 or int(row["axis"]) < 0:
            raise ValueError("Invalid native observation clock or axis")
        if row["stage"] not in ("fusion", "gate") or int(row["fused"]) not in (
            -1,
            0,
            1,
        ):
            raise ValueError("Invalid native observation stage or decision")
        if (row["stage"] == "gate") != (int(row["fused"]) == -1):
            raise ValueError(
                "Gate candidates and actual fusion decisions must stay separate"
            )
        if int(row["navigation"]) not in (0, 1):
            raise ValueError("Invalid native update mode")
        if fusion >= 10_000_000 and fusion > publication:
            raise ValueError("Native innovation is labelled at a future fusion epoch")
        for window, start, end in getattr(args, "windows", WINDOWS):
            if start * 1e6 <= fusion - clock_offset_s * 1e6 < end * 1e6:
                key = (
                    row["sensor"],
                    int(row["axis"]),
                    row["stage"],
                    bool(int(row["navigation"])),
                    window,
                )
                groups.setdefault(key, []).append(row)
    records = [
        dict(
            sensor=key[0],
            axis=key[1],
            stage=key[2],
            navigation=key[3],
            window=key[4],
            **summarize(values),
        )
        for key, values in sorted(groups.items())
    ]
    result = dict(
        filter=args.filter,
        native_clock_offset_s=clock_offset_s,
        raw_rows=len(rows),
        records=records,
        observer_csv_sha256=hashlib.sha256(args.innovations.read_bytes()).hexdigest(),
        scope="Per-axis scalar NIS at native fusion epochs, not joint vector NIS. Fusion rows are selection-conditioned; gate rows are separate. Correlated temporal samples are descriptive, not independent chi-square trials. Invalid rows remain disclosed and do not contribute finite NIS statistics.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--filter", choices=("px4", "ekf3"), required=True)
    parser.add_argument("--innovations", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    check(parser.parse_args())
