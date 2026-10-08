"""Transform full native covariance into the shared 15D right-error marginal."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from check_imu_coning import quaternion_product
from check_preintegration_noise import rotation
from consistency import consistency
from score import POSITION, QUATERNION, VELOCITY, read


NED_TO_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
FRD_TO_FLU = np.diag([1, -1, -1])


def common_state_and_jacobian(state, name, bias_period):
    state = np.asarray(state, dtype=float)
    if state.shape != (16,) or not np.isfinite(state).all():
        raise ValueError("Invalid native navigation state")
    if name not in ("px4", "ekf3") or not np.isfinite(bias_period) or bias_period <= 0:
        raise ValueError("Invalid native covariance convention")
    norm = np.linalg.norm(state[6:10])
    if abs(norm - 1) > 1e-3:
        raise ValueError("Native quaternion is not normalized")
    quaternion = state[6:10] / norm
    world_to_local = FRD_TO_FLU @ rotation(quaternion).T
    common = np.r_[
        NED_TO_ENU @ state[:3],
        NED_TO_ENU @ state[3:6],
        quaternion_product(
            quaternion_product(np.array([0, 2**-0.5, 2**-0.5, 0]), quaternion),
            np.array([0, 1, 0, 0]),
        ),
        FRD_TO_FLU @ state[10:13] / bias_period,
        FRD_TO_FLU @ state[13:16] / bias_period,
    ]
    jacobian = np.zeros((15, 24))
    if name == "px4":
        if bias_period != 1:
            raise ValueError("PX4 bias states are rates, not integrated increments")
        for block, index in enumerate((6, 3, 0, 9, 12)):
            jacobian[3 * block : 3 * block + 3, index : index + 3] = (
                world_to_local if block < 3 else FRD_TO_FLU
            )
    else:
        w, x, y, z = quaternion
        right_log_derivative = (
            2 * np.array([[-x, w, z, -y], [-y, -z, w, x], [-z, y, -x, w]]) / norm
        )
        jacobian[:3, 7:10] = world_to_local
        jacobian[3:6, 4:7] = world_to_local
        jacobian[6:9, :4] = FRD_TO_FLU @ right_log_derivative
        jacobian[9:12, 10:13] = FRD_TO_FLU / bias_period
        jacobian[12:15, 13:16] = FRD_TO_FLU / bias_period
    return common, jacobian


def check(args):
    raw = np.atleast_1d(np.genfromtxt(args.covariance, delimiter=",", names=True))
    expected = (
        "publication_us",
        "fusion_us",
        "bias_period_s",
        *(f"s{component}" for component in range(16)),
        *(f"p{row}_{column}" for row in range(24) for column in range(24)),
    )
    if raw.dtype.names != expected or not len(raw):
        raise ValueError("Missing complete native state/covariance export")
    if any(not np.isfinite(raw[key]).all() for key in expected):
        raise ValueError("Nonfinite native covariance export; retain the failed run")
    raw_count = len(raw)
    offset = 1e6 if args.filter == "px4" else 0
    early = raw["fusion_us"] < offset + 10e6
    early_epochs = raw["fusion_us"][early]
    unscored_repeated_epochs = int(np.count_nonzero(np.diff(early_epochs) == 0))
    raw = raw[~early]
    publication = raw["publication_us"]
    fusion = raw["fusion_us"]
    if np.any(fusion > publication) or np.any(np.diff(fusion) < 0):
        raise ValueError("Invalid native fusion epoch")
    repeated = np.flatnonzero(np.diff(fusion) == 0) + 1
    changed_posteriors = [
        int(index)
        for index in repeated
        if any(raw[key][index] != raw[key][index - 1] for key in expected[2:])
    ]
    fields = ("t_s", *POSITION, *VELOCITY, *QUATERNION)
    estimate = np.zeros(len(raw), dtype=[(key, float) for key in fields])
    diagnostics = np.zeros(
        len(raw),
        dtype=[
            (key, float)
            for key in (
                "t_s",
                "bgx",
                "bgy",
                "bgz",
                "bax",
                "bay",
                "baz",
                *(f"p{row}_{column}" for row in range(15) for column in range(15)),
            )
        ],
    )
    estimate["t_s"] = diagnostics["t_s"] = (raw["fusion_us"] - offset) / 1e6
    for index, row in enumerate(raw):
        state = np.array([row[f"s{component}"] for component in range(16)])
        common, jacobian = common_state_and_jacobian(
            state, args.filter, row["bias_period_s"]
        )
        covariance = np.array(
            [[row[f"p{r}_{c}"] for c in range(24)] for r in range(24)]
        )
        marginal = jacobian @ covariance @ jacobian.T
        for field, value in zip(fields[1:], common[:10], strict=True):
            estimate[field][index] = value
        for field, value in zip(diagnostics.dtype.names[1:7], common[10:], strict=True):
            diagnostics[field][index] = value
        for r in range(15):
            for c in range(15):
                diagnostics[f"p{r}_{c}"][index] = marginal[r, c]
    windows = {}
    for name, start, end in (
        ("before_takeoff", 10, 13),
        ("flight", 13, 59.7),
        ("outage", 25, 40),
        ("after_return", 40, 59.7),
    ):
        windows[name] = consistency(
            estimate,
            diagnostics,
            read(args.truth),
            [0.0008, -0.0005, 0.0004],
            [0.02, -0.01, 0.015],
            start,
            end,
        )
    result = dict(
        filter=args.filter,
        raw_rows=raw_count,
        unscored_early_rows=int(np.count_nonzero(early)),
        unscored_early_repeated_epochs=unscored_repeated_epochs,
        scored_observer_rows=len(raw),
        unique_fusion_epochs=len(np.unique(fusion)),
        repeated_identical_rows=len(repeated) - len(changed_posteriors),
        repeated_changed_posterior_rows=len(changed_posteriors),
        changed_posterior_publication_s=[
            float(publication[i] / 1e6) for i in changed_posteriors
        ],
        fusion_delay_s=[
            float(np.min((publication - fusion) / 1e6)),
            float(np.max((publication - fusion) / 1e6)),
        ],
        covariance_sha256=hashlib.sha256(args.covariance.read_bytes()).hexdigest(),
        truth_sha256=hashlib.sha256(args.truth.read_bytes()).hexdigest(),
        windows=windows,
        scope="Full native 24x24 covariance transformed into the common 15x15 navigation/bias marginal; not 24D NEES or NIS.",
        sampling="Every observer row at/after fusion time 10 s is retained, including repeated epochs and changed posteriors. NEES uses each row's fusion epoch; repeated observations are correlated. Startup rows are retained in the raw trace but outside the declared scoring windows.",
        conventions={
            "px4": "Native attitude error is LEFT/NED (confirmed in fuse and symbolic derivation); position/velocity NED and bias rates FRD.",
            "ekf3": "Native additive quaternion covariance mapped through normalized right-log differential; bias increments divided by dtEkfAvg. Fusion epoch has millisecond resolution.",
        }[args.filter],
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({name: row["mean_nees_15d"] for name, row in windows.items()}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filter", choices=("px4", "ekf3"))
    for name in ("covariance", "truth", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Choose a new evidence path")
    check(args)
