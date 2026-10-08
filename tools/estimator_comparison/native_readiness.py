"""Count actual native GPS corrections on the physical capture clock."""

import csv


def check(path, name, scenario, mission):
    if name not in ("px4", "ekf3") or scenario not in ("gps", "denied", "transition"):
        raise ValueError("Unknown native filter or scenario")
    offset_s = 1.0 if name == "px4" else 0.0
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("Missing native innovation observer rows")
    aids = {}
    for sensor in ("gps_position", "gps_velocity"):
        epochs = [
            int(row["fusion_us"]) / 1e6 - offset_s
            for row in rows
            if row["sensor"] == sensor
            and row["stage"] == "fusion"
            and int(row["fused"]) == 1
            and int(row["axis"]) == 0
        ]
        aids[sensor] = dict(
            first_fusion_s=min(epochs) if epochs else None,
            before_takeoff=sum(t < mission.warmup_s for t in epochs),
            before_loss=sum(mission.warmup_s <= t < mission.loss_s for t in epochs),
            after_return=sum(
                mission.return_s <= t < mission.end_s - 0.3 for t in epochs
            ),
            total=len(epochs),
        )
    qualified = (
        all(row["total"] == 0 for row in aids.values())
        if scenario == "denied"
        else all(
            row["before_takeoff"] and row["before_loss"] and row["after_return"]
            for row in aids.values()
        )
    )
    return dict(
        native_clock_offset_s=offset_s,
        gps=aids,
        qualified=bool(qualified),
        scope="Actual accepted horizontal-axis position and velocity corrections; physical fusion epochs. This readiness check does not establish matched covariance, priors or magnetic policies.",
    )
