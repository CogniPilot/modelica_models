#!/usr/bin/env python3
"""Report delayed-state consistency without assigning its covariance to now."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path

from check_horizon_consistency import WINDOWS


def report(args):
    paths = [args.output.with_suffix(suffix) for suffix in (".txt", ".csv")]
    if any(path.exists() for path in paths):
        raise ValueError("Choose new report paths")
    records = []
    configuration = None
    for dataset, path in (("original", args.original), ("held-out", args.held_out)):
        data = json.loads(path.read_text())
        profile = (
            data["source_sha256"],
            data["replay_sha256"],
            data["imu_noise_density"],
        )
        if configuration is not None and configuration != profile:
            raise ValueError(
                "Horizon implementation or calibration differs between motion sets"
            )
        configuration = profile
        seeds = {row["seed"] for row in data["scores"]}
        expected = set(
            itertools.product(seeds, (2, 4), ("gps", "denied", "transition"))
        )
        actual = {
            (row["seed"], row["climb_height_m"], row["scenario"])
            for row in data["scores"]
        }
        if (
            len(seeds) != 3
            or expected != actual
            or len(data["scores"]) != len(expected)
        ):
            raise ValueError("Incomplete horizon consistency matrix")
        if data["state_epoch"] != "fusion":
            raise ValueError("Covariance must be scored at the fusion epoch")
        for row in data["scores"]:
            if not row["primary_output_identical"]:
                raise ValueError("Covariance recording changed primary output")
            if [
                (w["consistency"]["start_s"], w["consistency"]["end_s"])
                for w in row["windows"]
            ] != list(WINDOWS):
                raise ValueError("Inconsistent covariance windows")
            for window in row["windows"]:
                accuracy, covariance = window["accuracy"], window["consistency"]
                if (
                    accuracy["finite_fraction"] != 1
                    or covariance["max_covariance_asymmetry"] != 0
                ):
                    raise ValueError("Invalid delayed state or asymmetric covariance")
                if not math.isfinite(covariance["mean_nees_15d"]):
                    raise ValueError("Nonfinite consistency statistic")
                records.append(
                    dict(
                        dataset=dataset,
                        seed=row["seed"],
                        height_m=row["climb_height_m"],
                        scenario=row["scenario"],
                        start_s=covariance["start_s"],
                        end_s=covariance["end_s"],
                        rows=covariance["rows"],
                        mean_nees_15d=covariance["mean_nees_15d"],
                        attitude_rmse_deg=accuracy["attitude_rmse_deg"],
                        velocity_rmse_m_s=accuracy["velocity_3d_rmse_m_s"],
                        covariance_asymmetry=covariance["max_covariance_asymmetry"],
                    )
                )
    ablation = json.loads(args.magnetic_ablation.read_text())
    if ablation["imu_noise_density"] != configuration[2]:
        raise ValueError("Magnetic ablation and horizon calibrations differ")
    expected = set(
        itertools.product(
            (7, 41), ("retrodiction", "horizon"), ("symmetric", "predicted")
        )
    )
    actual = {(row["seed"], row["mode"], row["variant"]) for row in ablation["scores"]}
    if (
        expected != actual
        or len(ablation["scores"]) != 8
        or sum(row["control_identical"] for row in ablation["scores"]) != 4
    ):
        raise ValueError("Incomplete magnetic diagnostic ablation")
    text = """Fusion-horizon covariance and stationary startup investigation
7 October 2026

36 covariance-enabled horizon replays cover six original/held-out seeds,
two heights and GPS/denied/loss-return with nominal delays and high-rate aiding.
Every primary output hash and delivered-packet trace matches the preceding
altitude experiment. No Modelica filter or tuning change is made in this stage.

Covariance is scored with the delayed ESKF state and truth at fusion_t_s,
not with the predictor's current output. The driver records publication time,
fusion time and readiness separately. consistency.py requires explicit
--fusion-horizon for these files and refuses mismatched/nonmonotonic epochs.
The moving-truth regression catches substituting current position or current
truth time. Warmup before a real nonnegative fusion epoch is excluded; none
of the declared windows intersects it. The flight interval ends at 59.7 s
because the final delayed state precedes the 60 s publication endpoint.

All selected 15-state covariances admit Cholesky factorization and are exactly
symmetric in their recorded precision. These properties are necessary but do
not prove calibration. NEES values below are descriptive: each run's time
samples are correlated, and six fixed-bias synthetic captures are not an
independent Monte Carlo consistency study. The nominal dimension is 15;
neither closeness to 15 nor positive definiteness alone is a pass criterion.

Each row: seed / height / scenario / mean NEES at 10..13 s /
mean NEES at 13..59.7 s / delayed flight attitude RMSE degrees

"""
    for dataset in ("original", "held-out"):
        text += dataset + "\n"
        for seed, height, scenario in sorted(
            {
                (r["seed"], r["height_m"], r["scenario"])
                for r in records
                if r["dataset"] == dataset
            }
        ):
            group = [
                r
                for r in records
                if (r["dataset"], r["seed"], r["height_m"], r["scenario"])
                == (dataset, seed, height, scenario)
            ]
            rest = next(r for r in group if r["start_s"] == 10)
            flight = next(r for r in group if r["start_s"] == 13 and r["end_s"] == 59.7)
            text += f"  {seed:3d} {height}m {scenario:10s} {rest['mean_nees_15d']:10.3f} {flight['mean_nees_15d']:10.3f} {flight['attitude_rmse_deg']:10.4f}\n"
        text += "\n"
    flights = [r for r in records if r["start_s"] == 13 and r["end_s"] == 59.7]
    selected = [
        r
        for r in records
        if (r["dataset"], r["seed"], r["height_m"], r["scenario"])
        == ("original", 41, 2, "denied")
    ]
    early = next(r for r in selected if r["start_s"] == 1)
    late = next(r for r in selected if r["start_s"] == 10)
    flight = next(r for r in selected if r["start_s"] == 13 and r["end_s"] == 59.7)
    a = next(
        r
        for r in ablation["scores"]
        if (r["seed"], r["mode"], r["variant"]) == (41, "retrodiction", "symmetric")
    )
    b = next(
        r
        for r in ablation["scores"]
        if (r["seed"], r["mode"], r["variant"]) == (41, "retrodiction", "predicted")
    )
    text += f"""The horizon filter's flight-average NEES ranges from {min(r["mean_nees_15d"] for r in flights):.3f} to {max(r["mean_nees_15d"] for r in flights):.3f} in
these cases. Startup remains overconfident: for example seed 41, 2 m denied,
has mean NEES {early["mean_nees_15d"]:.2f} at 1..5 s and {late["mean_nees_15d"]:.2f} at 10..13 s despite a small attitude
error. Its delayed flight mean is {flight["mean_nees_15d"]:.3f}. This does not certify its uncertainty
or remove the need to improve the stationary model.

The magnetic-Jacobian hypothesis was tested separately in eight generated-code
diagnostic replays: symmetric versus predicted-field linearization, both
filter modes, original seeds 7 and 41, 2 m denied. Four unchanged controls
match historical hashes. Seed-41 retrodiction at 10..13 s has attitude RMSE
{a["windows"][2]["accuracy"]["attitude_rmse_deg"]:.3f} degrees and NEES {a["windows"][2]["consistency"]["mean_nees_15d"]:.2f} with the symmetric Jacobian, versus {b["windows"][2]["accuracy"]["attitude_rmse_deg"]:.3f} and
{b["windows"][2]["consistency"]["mean_nees_15d"]:.2f} with the predicted-field version. The latter does not fix startup;
its flight horizontal RMSE also worsens from {a["windows"][-1]["accuracy"]["horizontal_position_rmse_m"]:.4f} to {b["windows"][-1]["accuracy"]["horizontal_position_rmse_m"]:.4f} m. It is not
adopted. See stationary-magnetic-ablation.json for all windows and both seeds.

The independent stationary-observability check exhibits 96 finite equivalent
attitude/bias configurations and verifies the associated dynamics/observation
null direction over 24 orientations. This supports treating the stationary
attitude/bias ambiguity explicitly; it does not identify the current filter's
root cause. Explicit slant-range/terrain attitude constraints are outside that
small observation model. Maneuvering can break the ambiguity.

Next algorithm work should test a stationary IMU model that uses the declared
rest constraint and preserves the coupled attitude/bias uncertainty, with
careful accounting for any reused IMU noise. Merely replacing the magnetic
Jacobian or inflating covariance is not a validated solution. Retrodiction
startup, native covariance comparability, exact sampled FOH process noise,
sensor-time fidelity, disturbances and target resource qualification remain
open. This stage does not establish an all-scenario estimator win.

Reproduce each motion set with check_horizon_consistency.py --help, the frozen
altitude captures/reference and unchanged vector candidate. Build replay.c
with the existing horizon flags; diagnostic CSV is the positional argument
after foh. report_horizon_consistency.py regenerates this text and CSV.
check_stationary_observability.py independently regenerates its JSON.
The manifest preserves current source, reference and output/control hashes.
"""
    with paths[1].open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    paths[0].write_text(text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("original", "held-out", "magnetic-ablation", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    report(parser.parse_args())
