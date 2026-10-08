"""Fixed sensor specification for a comparison above native noise floors."""

from dataclasses import dataclass


COMMON_NOISE_PROFILE = "common-native-floors-v1"


@dataclass(frozen=True)
class SensorNoise:
    gps_position_m: tuple = (0.2, 0.2, 0.35)
    gps_velocity_m_s: tuple = (0.05, 0.05, 0.075)
    barometer_m: float = 0.1
    magnetic_T: float = 1e-6
    range_m: float = 0.02
    flow_rate_rad_s: float = 0.05


COMMON_SENSOR_NOISE = SensorNoise()
