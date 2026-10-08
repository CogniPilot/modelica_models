"""Map a frozen continuous IMU density into the pinned native rate convention."""

import math


PREDICTION_PERIOD_S = {"px4": 0.01, "ekf3": 0.0125}


def native_noise(densities):
    if len(densities) != 2 or not all(
        math.isfinite(value) and 1e-9 <= value <= 100 for value in densities
    ):
        raise ValueError("Declare finite positive gyro/accel noise densities")
    result = {
        name: [value / math.sqrt(period) for value in densities]
        for name, period in PREDICTION_PERIOD_S.items()
    }
    if result["ekf3"][0] > 1 or result["ekf3"][1] > 5:
        raise ValueError(
            "Density would exceed the pinned EKF3 native process-noise clamp"
        )
    return result
