"""Declare sensor-informed native tuning, retaining native enforced noise floors."""

import math

from native_noise import PREDICTION_PERIOD_S
from sensor_noise import COMMON_NOISE_PROFILE, COMMON_SENSOR_NOISE


PARAMETER_GROUPS = {
    "gps": {
        "px4": ("ekf2_gps_p_noise", "ekf2_gps_v_noise"),
        "ekf3": ("EK3_POSNE_M_NSE", "EK3_VELNE_M_NSE", "EK3_VELD_M_NSE"),
    },
    "barometer": {"px4": ("ekf2_baro_noise",), "ekf3": ("EK3_ALT_M_NSE",)},
    "range": {"px4": ("ekf2_rng_noise",), "ekf3": ("EK3_RNG_M_NSE",)},
    "magnetometer": {
        "px4": ("ekf2_mag_noise", "ekf2_head_noise"),
        "ekf3": ("EK3_MAG_M_NSE", "EK3_YAW_M_NSE"),
    },
    "flow": {"px4": ("ekf2_of_n_min", "ekf2_of_n_max"), "ekf3": ("EK3_FLOW_M_NSE",)},
    "bias": {
        "px4": ("ekf2_gyr_b_noise", "ekf2_acc_b_noise"),
        "ekf3": ("EK3_GBIAS_P_NSE", "EK3_ABIAS_P_NSE"),
    },
}


def sensor_informed_noise(groups=None):
    bias_psd = (1e-10, 1e-6)
    bias = {
        name: [math.sqrt(value / period) for value in bias_psd]
        for name, period in PREDICTION_PERIOD_S.items()
    }
    profile = dict(
        name="sensor-informed-native-floors",
        targets=dict(
            gps_horizontal_position_std_m=0.2,
            gps_velocity_std_m_s=0.05,
            pressure_std_m=0.1,
            range_variance_m2=0.02**2,
            magnetic_std_T=1e-6,
            flow_rate_std_rad_s=0.05,
            heading_std_rad=0.05,
            gyro_bias_psd_rad2_s3=bias_psd[0],
            accel_bias_psd_m2_s5=bias_psd[1],
        ),
        px4={
            "ekf2_gps_p_noise": 0.2,
            "ekf2_gps_v_noise": 0.05,
            "ekf2_baro_noise": 0.1,
            "ekf2_rng_noise": 0.02,
            "ekf2_mag_noise": 0.01,
            "ekf2_head_noise": 0.05,
            "ekf2_of_n_min": 0.05,
            "ekf2_of_n_max": 0.05,
            "ekf2_gyr_b_noise": bias["px4"][0],
            "ekf2_acc_b_noise": bias["px4"][1],
        },
        ekf3={
            "EK3_POSNE_M_NSE": 0.2,
            "EK3_VELNE_M_NSE": 0.05,
            "EK3_VELD_M_NSE": 0.05,
            "EK3_ALT_M_NSE": 0.1,
            "EK3_RNG_M_NSE": 0.02**2,
            "EK3_MAG_M_NSE": 0.01,
            "EK3_YAW_M_NSE": 0.05,
            "EK3_FLOW_M_NSE": 0.05,
            "EK3_GBIAS_P_NSE": bias["ekf3"][0],
            "EK3_ABIAS_P_NSE": bias["ekf3"][1],
        },
        limitations=[
            "Native 0.05 rad/s flow and 1 microtesla magnetic floors exceed this capture's physical noise; ESKF covariance still follows the physical packet metadata.",
            "EKF3 terrain fusion uses EK3_RNG_M_NSE directly as variance; its height fusion squares a clamped value. The variance mapping requires barometric height and disabled range height, as in this campaign. It is not a deployment parameter recommendation.",
            "Initial priors, height/magnetic source selection, terrain models, dynamic noise additions, state inhibition and gating remain native policies, not matched quantities.",
            "Bias process PSD mapping uses the observed prediction periods; it does not override native state inhibition or covariance constraints.",
        ],
    )
    if groups is None:
        return profile
    if len(set(groups)) != len(groups) or not set(groups).issubset(PARAMETER_GROUPS):
        raise ValueError("Require distinct declared native noise parameter groups")
    profile["name"] += ":" + (",".join(groups) if groups else "native-default-aiding")
    profile["selected_groups"] = list(groups)
    profile["full_profile_targets"] = profile.pop("targets")
    for estimator in ("px4", "ekf3"):
        names = {
            name for group in groups for name in PARAMETER_GROUPS[group][estimator]
        }
        profile[estimator] = {
            name: value for name, value in profile[estimator].items() if name in names
        }
    return profile


def configured_noise(args):
    profile = sensor_informed_noise(getattr(args, "noise_groups", None))
    if getattr(args, "noise_profile", None) == COMMON_NOISE_PROFILE:
        if getattr(args, "noise_groups", None) is not None:
            raise ValueError("Common sensor noise requires all declared groups")
        profile["name"] = COMMON_NOISE_PROFILE
        profile["ekf3"]["EK3_VELD_M_NSE"] = COMMON_SENSOR_NOISE.gps_velocity_m_s[2]
        profile["targets"]["gps_vertical_velocity_std_m_s"] = (
            COMMON_SENSOR_NOISE.gps_velocity_m_s[2]
        )
        profile["limitations"][0] = (
            "The common capture uses the native flow/magnetic floors and PX4's "
            "1.5 vertical GPS velocity standard-deviation factor. Dynamic magnetic "
            "noise, barometer ground-effect inflation, range propagation, height "
            "source selection and initial priors remain native policies."
        )
    return profile
