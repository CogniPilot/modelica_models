"""Reconstruct joint ESKF NIS from the actual per-update residual and covariance."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from manifest import digest


DIMENSIONS = {
    "gps": 6,
    "gps_position": 3,
    "gps_velocity": 3,
    "optical_flow": 2,
    "barometer": 1,
    "magnetic_vector": 3,
    "magnetic_heading": 1,
}
FIELDS = {
    "sensor",
    "state_epoch_s",
    "sample_epoch_s",
    "age_s",
    "dimension",
    "computed",
    "accepted",
    "outcome",
    "nis",
    "gate",
    *(f"r{axis}" for axis in range(6)),
    *(f"noise{axis}" for axis in range(6)),
    *(f"s{row}_{column}" for row in range(6) for column in range(6)),
}


def reconstruct(row):
    if set(row) != FIELDS:
        raise ValueError("Incomplete ESKF innovation schema")
    dimension = int(row["dimension"])
    if DIMENSIONS.get(row["sensor"]) != dimension:
        raise ValueError("Sensor and joint innovation dimension disagree")
    computed, accepted, outcome = (
        int(row[key]) for key in ("computed", "accepted", "outcome")
    )
    if computed not in (0, 1) or accepted not in (0, 1):
        raise ValueError("Invalid observation decision")
    if outcome not in range(1, 7) or bool(accepted) != (outcome == 1):
        raise ValueError("Correction decision and rejection outcome disagree")
    numeric = {key: float(value) for key, value in row.items() if key != "sensor"}
    if not np.isfinite(list(numeric.values())).all():
        raise ValueError("Non-finite observation; cannot reconstruct its NIS")
    state, sample, age = (
        numeric[key] for key in ("state_epoch_s", "sample_epoch_s", "age_s")
    )
    if abs(float(np.float32(sample + age)) - state) > 1e-6:
        raise ValueError("Innovation epochs do not agree with the recorded age")
    if not computed:
        if accepted or outcome not in (2, 5, 6):
            raise ValueError("Missing covariance for a linear correction")
        return dict(computed=False, outcome=outcome, accepted=False)
    residual = np.array([numeric[f"r{axis}"] for axis in range(dimension)])
    noise = np.array([numeric[f"noise{axis}"] for axis in range(dimension)])
    covariance = np.array(
        [[numeric[f"s{r}_{c}"] for c in range(dimension)] for r in range(dimension)]
    )
    if not np.array_equal(covariance, covariance.T):
        raise ValueError("Actual innovation covariance is not symmetric")
    if np.any(noise <= 0):
        raise ValueError("Non-positive observation variance")
    try:
        root = np.linalg.cholesky(covariance)
    except np.linalg.LinAlgError as error:
        raise ValueError(
            "Actual innovation covariance is not positive definite"
        ) from error
    whitened = np.linalg.solve(root, residual)
    nis = float(whitened @ whitened)
    if not np.isclose(nis, numeric["nis"], rtol=5e-5, atol=2e-6):
        raise ValueError("Reconstructed joint NIS differs from the generated update")
    passed = numeric["gate"] <= 0 or numeric["nis"] <= numeric["gate"] * dimension
    if (accepted and not passed) or (outcome == 3 and passed):
        raise ValueError("NIS, dimension and actual gate decision disagree")
    return dict(
        computed=True,
        accepted=bool(accepted),
        outcome=outcome,
        nis=nis,
        normalized_nis=nis / dimension,
        reconstruction_error=abs(nis - numeric["nis"]),
        whitened=whitened,
        noise=noise,
    )


def summarize(rows):
    computed = [value for value in rows if value["computed"]]
    result = dict(
        attempted_rows=len(rows),
        prelinear_rejected_rows=len(rows) - len(computed),
        accepted_rows=sum(value["accepted"] for value in rows),
        outcomes={str(k): sum(v["outcome"] == k for v in rows) for k in range(1, 7)},
        computed_rows=len(computed),
    )
    for decision in ("all_candidates", "accepted", "rejected"):
        selected = [
            v
            for v in computed
            if decision == "all_candidates" or v["accepted"] == (decision == "accepted")
        ]
        statistics = dict(rows=len(selected))
        if selected:
            nis = np.array([v["normalized_nis"] for v in selected])
            whitened = np.stack([v["whitened"] for v in selected])
            noise = np.stack([v["noise"] for v in selected])
            statistics.update(
                mean_joint_nis_per_dimension=float(nis.mean()),
                median_joint_nis_per_dimension=float(np.median(nis)),
                p95_joint_nis_per_dimension=float(np.quantile(nis, 0.95)),
                max_reconstruction_error=max(
                    v["reconstruction_error"] for v in selected
                ),
                mean_cholesky_whitened_residual=whitened.mean(axis=0).tolist(),
                rms_cholesky_whitened_residual=np.sqrt(
                    (whitened**2).mean(axis=0)
                ).tolist(),
                median_effective_noise_diagonal=np.median(noise, axis=0).tolist(),
            )
        result[decision] = statistics
    return result


def check(args):
    with args.innovations.open() as stream:
        reader = csv.DictReader(stream)
        if set(reader.fieldnames or ()) != FIELDS:
            raise ValueError("Incomplete ESKF innovation schema")
        rows = list(reader)
    if not rows:
        raise ValueError("Missing ESKF innovation observations")
    groups, failures, seen, repeats = {}, [], {}, {}
    for number, row in enumerate(rows, 2):
        try:
            value = reconstruct(row)
        except ValueError as error:
            failures.append(
                dict(csv_line=number, sensor=row["sensor"], reason=str(error))
            )
            continue
        key = tuple(row[k] for k in ("sensor", "state_epoch_s", "sample_epoch_s"))
        if key in seen:
            if row != seen[key]:
                failures.append(
                    dict(
                        csv_line=number,
                        sensor=row["sensor"],
                        reason="Conflicting evaluations of the same sensor and epochs",
                    )
                )
            else:
                repeats[row["sensor"]] = repeats.get(row["sensor"], 0) + 1
            continue
        seen[key] = row
        for window, start, end in args.windows:
            if start <= float(row["state_epoch_s"]) < end:
                groups.setdefault((row["sensor"], window), []).append(value)
    result = dict(
        valid=not failures,
        raw_rows=len(rows),
        unique_candidates=len(seen),
        repeated_identical_evaluations=repeats,
        invalid_rows=len(failures),
        failures=failures,
        innovations_sha256=digest(args.innovations),
        analyzer_sha256=digest(Path(__file__)),
        records=[
            dict(sensor=sensor, window=window, **summarize(values))
            for (sensor, window), values in sorted(groups.items())
        ],
        scope="Joint candidate NIS reconstructed in float64 from actual float32 r and S. Every raw function evaluation is checked; repeated evaluations with identical sensor/state/sample epochs must match every field and contribute once to statistics. Raw traces and repeat counts remain retained. Rumoca may repeat pure-function evaluation while assembling downstream record arguments; this is not evidence of repeated physical fusion. State epoch is sample plus age (fusion epoch in horizon mode), not publication time. Prelinear timestamp rejections never count as zero NIS. Accepted and rejected selections stay separate. Temporal samples are correlated; summaries are descriptive, not independent chi-square confidence tests. Native sequential scalar NIS is a different statistic. Invalid observations remain disclosed and invalidate this reconstruction.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    if failures:
        raise ValueError(
            f"{len(failures)} innovation observations failed reconstruction"
        )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--innovations", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.windows = json.loads(args.reference.read_text())["mission"]["windows"]
    check(args)
