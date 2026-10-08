"""Audit every scored navigation/datum covariance without repair or row removal."""

import argparse
import json
from pathlib import Path

import numpy as np

from compare_exposure import digest
from score import read


def check(args):
    records = []
    for startup in ("default", "rest"):
        study_path = getattr(args, startup + "_study")
        study = json.loads(study_path.read_text())
        work = getattr(args, startup + "_work")
        if not study.get("complete") or len(study["scores"]) != 48:
            raise ValueError("Require the complete frozen pressure comparison")
        for score in study["scores"]:
            label = f"{score['frequency']}_{score['seed']}_{score['climb_height_m']}m_{score['scenario']}_explicit"
            path = (
                work
                / label
                / (score["name"] + "_" + score["variant"])
                / "covariance.csv"
            )
            values = read(path)
            epoch = values["fusion_t_s" if score["name"] == "horizon" else "t_s"]
            scored = values[(epoch >= 10) & (epoch < 59.7)]
            if len(scored) != 4970:
                raise ValueError("A joint covariance audit lost a scored epoch")
            matrices = np.empty((len(scored), 16, 16))
            matrices[:, :15, :15] = np.column_stack(
                [
                    scored[f"p{row}_{column}"]
                    for row in range(15)
                    for column in range(15)
                ]
            ).reshape(-1, 15, 15)
            cross = np.column_stack(
                [scored[f"barometer_cross_{row}"] for row in range(15)]
            )
            matrices[:, :15, 15] = cross
            matrices[:, 15, :15] = cross
            matrices[:, 15, 15] = scored["barometer_bias_variance_m2"]
            if not np.isfinite(matrices).all():
                raise ValueError("Nonfinite joint covariance; audit cannot conceal it")
            failed = []
            try:
                np.linalg.cholesky(matrices)
            except np.linalg.LinAlgError:
                for index, matrix in enumerate(matrices):
                    try:
                        np.linalg.cholesky(matrix)
                    except np.linalg.LinAlgError:
                        failed.append(index)
            schur = matrices[:, 15, 15] - np.sum(
                cross
                * np.linalg.solve(matrices[:, :15, :15], cross[:, :, None])[:, :, 0],
                axis=1,
            )
            records.append(
                dict(
                    startup=startup,
                    estimator=score["name"],
                    variant=score["variant"],
                    **{
                        field: score[field]
                        for field in ("frequency", "seed", "climb_height_m", "scenario")
                    },
                    rows=len(scored),
                    cholesky_failures=len(failed),
                    first_failure_publication_s=float(scored["t_s"][failed[0]])
                    if failed
                    else None,
                    minimum_datum_schur_variance_m2=float(schur.min()),
                    covariance_sha256=digest(path),
                    study_sha256=digest(study_path),
                )
            )
    if len(records) != 96:
        raise ValueError("Incomplete joint covariance audit")
    result = dict(
        records=records,
        complete=True,
        scored_rows=sum(row["rows"] for row in records),
        cholesky_failures=sum(row["cholesky_failures"] for row in records),
        note="Full 16D navigation/datum covariance at the scored fusion epoch. Every row retained; no symmetrization, jitter, eigenvalue clipping or covariance repair.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "records"}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("default-study", "rest-study", "default-work", "rest-work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    check(parser.parse_args())
