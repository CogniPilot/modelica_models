#!/usr/bin/env python3
"""Generate a seeded analytic trajectory and common noisy sensor CSVs.

World ENU, body FLU, quaternion wxyz. Truth is never read by a replay driver.
No sensor latency in this controlled baseline; the stationary pad is real
noisy data, not an estimator-specific fabricated warm-up.
"""

import argparse
import json
from pathlib import Path
import numpy as np
from mission import Mission

FIELD = np.array([-1.59e-6, 20.04e-6, -47.91e-6])


def ramp(t, duration):
    x = np.clip(t / duration, 0, 1)
    active = (t > 0) & (t < duration)
    return (
        x**3 * (10 - 15 * x + 6 * x * x),
        np.where(active, 30 * x * x * (1 - x) ** 2 / duration, 0),
        np.where(active, 60 * x * (1 - x) * (1 - 2 * x) / duration**2, 0),
    )


def trajectory(t, speed, climb_height_m=2.0, warmup_s=13.0):
    if not np.isfinite(climb_height_m) or not 0 < climb_height_m <= 8:
        raise ValueError("Climb height must be finite and between 0 and 8 m")
    tau = np.maximum(t - warmup_s, 0)
    r, dr, ddr = ramp(tau, 5)
    w = speed
    signal = np.column_stack((4 * np.sin(w * tau), 3 * (1 - np.cos(w * tau))))
    first = np.column_stack((4 * w * np.cos(w * tau), 3 * w * np.sin(w * tau)))
    second = np.column_stack(
        (-4 * w * w * np.sin(w * tau), 3 * w * w * np.cos(w * tau))
    )
    z, dz, ddz = ramp(tau, 3)
    p = np.column_stack((r[:, None] * signal, climb_height_m * z))
    v = np.column_stack(
        (dr[:, None] * signal + r[:, None] * first, climb_height_m * dz)
    )
    a = np.column_stack(
        (
            ddr[:, None] * signal + 2 * dr[:, None] * first + r[:, None] * second,
            climb_height_m * ddz,
        )
    )
    angles = r[:, None] * np.column_stack(
        (0.10 * np.sin(0.6 * tau), 0.12 * np.sin(0.4 * tau), 0.3 * np.sin(0.18 * tau))
    )
    rates = dr[:, None] * np.column_stack(
        (0.10 * np.sin(0.6 * tau), 0.12 * np.sin(0.4 * tau), 0.3 * np.sin(0.18 * tau))
    ) + r[:, None] * np.column_stack(
        (
            0.06 * np.cos(0.6 * tau),
            0.048 * np.cos(0.4 * tau),
            0.054 * np.cos(0.18 * tau),
        )
    )
    ro, pi, ya = angles.T
    sr, cr, sp, cp, sy, cy = (
        np.sin(ro),
        np.cos(ro),
        np.sin(pi),
        np.cos(pi),
        np.sin(ya),
        np.cos(ya),
    )
    R = np.stack(
        (
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            -sp,
            cp * sr,
            cp * cr,
        ),
        axis=1,
    ).reshape(-1, 3, 3)
    rd, pd, yd = rates.T
    gyro = np.column_stack(
        (rd - yd * sp, pd * cr + yd * sr * cp, -pd * sr + yd * cr * cp)
    )
    qr, qp, qy = ro / 2, pi / 2, ya / 2
    sr, cr, sp, cp, sy, cy = (
        np.sin(qr),
        np.cos(qr),
        np.sin(qp),
        np.cos(qp),
        np.sin(qy),
        np.cos(qy),
    )
    quat = np.column_stack(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )
    force = np.einsum("nji,nj->ni", R, a + [0, 0, 9.81])
    body_v = np.einsum("nji,nj->ni", R, v)
    mag = np.einsum("nji,j->ni", R, FIELD)
    distance = (p[:, 2] + 1) / R[:, 2, 2]
    return p, v, a, quat, gyro, force, body_v, mag, distance


def write(directory, name, header, data):
    np.savetxt(
        directory / f"{name}.csv",
        data,
        delimiter=",",
        header=header,
        comments="",
        fmt="%.12g",
    )


def generate(
    directory, seed=7, speed=0.25, lower_rates=False, climb_height_m=2.0, warmup_s=13.0
):
    if not np.isfinite(climb_height_m) or not 0 < climb_height_m <= 8:
        raise ValueError("Climb height must be finite and between 0 and 8 m")
    mission = Mission(warmup_s)
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    t = np.arange(0, mission.end_s + 0.000625, 1 / 800)
    p, v, a, quat, gyro, force, bv, mag, distance = trajectory(
        t, speed, climb_height_m, warmup_s
    )
    gm = gyro + [0.0008, -0.0005, 0.0004] + rng.normal(0, 0.0015, gyro.shape)
    am = force + [0.02, -0.01, 0.015] + rng.normal(0, 0.03, force.shape)
    write(
        directory,
        "truth",
        "t_s,e_m,n_m,u_m,ve_m_s,vn_m_s,vu_m_s,qw,qx,qy,qz",
        np.column_stack((t, p, v, quat)),
    )
    write(
        directory,
        "imu",
        "t_s,gx_rad_s,gy_rad_s,gz_rad_s,ax_m_s2,ay_m_s2,az_m_s2",
        np.column_stack((t, gm, am)),
    )
    g = np.arange(0, len(t), 80)
    pg = p[g] + rng.normal(0, [0.2, 0.2, 0.35], (len(g), 3))
    vg = v[g] + rng.normal(0, 0.05, (len(g), 3))
    lat, lon, alt = 40.4237, -86.9212, 200.0
    geod = np.column_stack(
        (
            lat + np.rad2deg(pg[:, 1] / 6378137),
            lon + np.rad2deg(pg[:, 0] / (6378137 * np.cos(np.deg2rad(lat)))),
            alt + pg[:, 2],
        )
    )
    write(
        directory,
        "gps",
        "t_s,lat_deg,lon_deg,alt_msl_m,alt_ell_m,vn_m_s,ve_m_s,vd_m_s,hacc_m,vacc_m,sacc_m_s,fix_type,sats_used,pos_valid,vel_valid,vel_down_valid,e_m,n_m,u_m",
        np.column_stack(
            (
                t[g],
                geod,
                geod[:, 2],
                vg[:, 1],
                vg[:, 0],
                -vg[:, 2],
                np.full((len(g), 3), [0.2, 0.35, 0.05]),
                np.full((len(g), 5), [3, 14, 1, 1, 1]),
                pg,
            )
        ),
    )
    f = np.arange(0, len(t), 8)
    vf = bv[f, :2] + rng.normal(0, 0.03, (len(f), 2))
    df = np.maximum(distance[f] + rng.normal(0, 0.02, len(f)), 0.1)
    write(
        directory,
        "flow",
        "t_s,vx_flu_m_s,vy_flu_m_s,dist_m,quality,valid",
        np.column_stack((t[f], vf, df, np.ones((len(f), 2)))),
    )
    m = np.arange(0, len(t), 16)
    mm = mag[m] + rng.normal(0, 0.3e-6, (len(m), 3))
    bm = p[m, 2] + 0.3 + 0.05 * np.sin(0.03 * t[m]) + rng.normal(0, 0.1, len(m))
    # All cores receive the same ground-calibrated pressure altitude. ArduPilot
    # DAL is downstream of its barometer calibration, while the other APIs can
    # estimate a datum themselves. Calibrate from common stationary readings,
    # never from a moving truth trace or a different per-estimator warm-up.
    write(
        directory,
        "baro_raw",
        "t_s,altitude_m,valid",
        np.column_stack((t[m], bm, np.ones(len(m)))),
    )
    baro_datum = float(np.mean(bm[t[m] < 5]))
    bm = bm - baro_datum
    if lower_rates:
        # Downsample the same capture, preserving its original noise draws.
        f, vf, df = f[::4], vf[::4], df[::4]
        m, mm, bm = m[::2], mm[::2], bm[::2]
        write(
            directory,
            "flow",
            "t_s,vx_flu_m_s,vy_flu_m_s,dist_m,quality,valid",
            np.column_stack((t[f], vf, df, np.ones((len(f), 2)))),
        )
    write(
        directory,
        "mag",
        "t_s,bx_T,by_T,bz_T,valid",
        np.column_stack((t[m], mm, np.ones(len(m)))),
    )
    write(
        directory,
        "baro",
        "t_s,altitude_m,valid",
        np.column_stack((t[m], bm, np.ones(len(m)))),
    )
    # A 100 Hz merged record for the generated Modelica estimator. Each IMU
    # packet is the mean of the preceding eight measured 800 Hz samples.
    # Fresh flags identify the same aiding samples given to the native cores.
    j = np.arange(0, len(t), 8)
    mean_g = np.vstack((gm[0], gm[1 : len(t)].reshape(-1, 8, 3).mean(axis=1)))
    mean_a = np.vstack((am[0], am[1 : len(t)].reshape(-1, 8, 3).mean(axis=1)))
    gi = np.searchsorted(g, j, side="right") - 1
    mi = np.searchsorted(m, j, side="right") - 1
    fi = np.searchsorted(f, j, side="right") - 1
    packed = np.column_stack(
        (
            t[j],
            mean_g,
            mean_a,
            (j % 80 == 0),
            t[g[gi]],
            pg[gi],
            vg[gi],
            t[f[fi]],
            vf[fi],
            df[fi],
            np.isin(j, m),
            t[m[mi]],
            mm[mi],
            bm[mi],
        )
    )
    write(
        directory,
        "modelica_input",
        "t_s,gx,gy,gz,ax,ay,az,gps_fresh,gps_t,e,n,u,ve,vn,vu,flow_t,vx,vy,range,mag_fresh,mag_t,bx,by,bz,baro",
        packed,
    )
    (directory / "origin.json").write_text(
        json.dumps(
            dict(
                lat_deg=lat,
                lon_deg=lon,
                alt_msl_m=alt,
                declination_deg=float(np.rad2deg(np.arctan2(FIELD[0], FIELD[1]))),
                gps_latency_ms=0,
                flow_latency_ms=0,
                seed=seed,
                speed=speed,
                sensor_rates_hz=dict(
                    imu=800,
                    gps=10,
                    flow=25 if lower_rates else 100,
                    mag=25 if lower_rates else 50,
                    baro=25 if lower_rates else 50,
                ),
                arm_after_s=13 if warmup_s == 13 else warmup_s,
                ground_plane_offset_m=-1,
                baro_startup_datum_m=baro_datum,
                world_magnetic_field_T=FIELD.tolist(),
                **({"climb_height_m": climb_height_m} if climb_height_m != 2 else {}),
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--speed", type=float, default=0.25)
    parser.add_argument("--lower-rates", action="store_true")
    parser.add_argument("--climb-height-m", type=float, default=2.0)
    parser.add_argument("--warmup-s", type=float, default=13.0)
    args = parser.parse_args()
    generate(
        args.output,
        args.seed,
        args.speed,
        args.lower_rates,
        args.climb_height_m,
        args.warmup_s,
    )
