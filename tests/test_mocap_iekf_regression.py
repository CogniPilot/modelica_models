from __future__ import annotations

import csv
import math
import os
import subprocess
import textwrap
from pathlib import Path

import rumoca as rm

from tests.rust_codegen import normalize_rumoca_rust


ROOT = Path(__file__).resolve().parents[1]
MODEL_FILE = ROOT / "Estimation" / "Examples" / "MocapExternalOdometryIEKF.mo"
MODEL_NAME = "Estimation.Examples.MocapExternalOdometryIEKF"
MISSION_S = 20.0
MOCAP_HZ = 240.0
DROPOUT_WINDOWS = [(4.0, 5.0), (8.5, 9.5), (13.0, 14.0), (17.0, 18.0)]


def test_mocap_iekf_tracks_figure_eight_with_fixed_one_second_dropouts(tmp_path: Path) -> None:
    model = rm.Session(roots=[str(ROOT)]).load(MODEL_FILE, model=MODEL_NAME)
    codegen = model.codegen("rust-solve")

    generated_path = tmp_path / "mocap_iekf_generated.rs"
    generated_path.write_text(normalize_rumoca_rust(codegen.files[0].content))

    dropout_windows = DROPOUT_WINDOWS
    harness_path = tmp_path / "mocap_iekf_regression.rs"
    harness_path.write_text(_rust_harness(dropout_windows))
    exe_path = tmp_path / "mocap_iekf_regression"

    subprocess.run(
        ["rustc", "-Awarnings", "-O", str(harness_path), "-o", str(exe_path)],
        cwd=tmp_path,
        check=True,
    )

    artifact_dir = Path(
        os.environ.get("MOCAP_IEKF_ARTIFACT_DIR", tmp_path / "mocap_iekf_artifacts")
    )
    artifact_dir.mkdir(parents=True, exist_ok=True)
    csv_path = artifact_dir / "mocap_iekf_errors.csv"
    svg_path = artifact_dir / "mocap_iekf_errors.svg"

    subprocess.run([str(exe_path), str(csv_path)], check=True)
    rows = _read_rows(csv_path)
    _write_svg(svg_path, rows, dropout_windows)

    warm_rows = [row for row in rows if row["time_s"] >= 1.0]
    valid_rows = [row for row in warm_rows if row["measurement_valid"] == 1.0]
    dropout_rows = [row for row in warm_rows if row["measurement_valid"] == 0.0]

    assert len(rows) == int(MISSION_S * MOCAP_HZ) + 1
    dropout_duration_s = sum(end - start for start, end in dropout_windows)
    assert len(dropout_rows) >= int(dropout_duration_s * MOCAP_HZ * 0.95)
    assert all(row["finite"] == 1.0 for row in rows)
    assert all(row["initialized"] == 1.0 for row in warm_rows)

    valid_position_rms = _rms(row["position_error_m"] for row in valid_rows)
    valid_velocity_rms = _rms(row["velocity_error_m_s"] for row in valid_rows)
    dropout_position_max = max(row["position_error_m"] for row in dropout_rows)
    dropout_velocity_max = max(row["velocity_error_m_s"] for row in dropout_rows)
    dropout_attitude_max = max(row["attitude_error_rad"] for row in dropout_rows)

    assert valid_position_rms < 0.08
    assert valid_velocity_rms < 0.35
    assert dropout_position_max < 0.75
    assert dropout_velocity_max < 1.25
    assert dropout_attitude_max < 0.25

    for start_s, end_s in dropout_windows:
        recovery_rows = [
            row for row in rows if end_s + 0.20 <= row["time_s"] <= end_s + 0.70
        ]
        assert recovery_rows
        assert min(row["position_error_m"] for row in recovery_rows) < 0.12


def _read_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as csv_file:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(csv_file)
        ]


def _rms(values) -> float:
    values = list(values)
    return math.sqrt(sum(value * value for value in values) / len(values))


def _write_svg(
    path: Path, rows: list[dict[str, float]], dropouts: list[tuple[float, float]]
) -> None:
    width = 1200
    height = 520
    margin_left = 72
    margin_right = 24
    margin_top = 28
    margin_bottom = 48
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    t_min = 0.0
    t_max = MISSION_S
    y_max = max(
        0.15,
        max(row["position_error_m"] for row in rows),
        max(row["attitude_error_rad"] for row in rows),
        max(row["velocity_error_m_s"] / 4.0 for row in rows),
    )
    y_max *= 1.15

    def x(t: float) -> float:
        return margin_left + (t - t_min) / (t_max - t_min) * plot_w

    def y(value: float) -> float:
        return margin_top + plot_h - value / y_max * plot_h

    def points(name: str, scale: float = 1.0) -> str:
        return " ".join(
            f"{x(row['time_s']):.2f},{y(row[name] * scale):.2f}" for row in rows
        )

    dropout_rects = "\n".join(
        f'<rect x="{x(start):.2f}" y="{margin_top}" '
        f'width="{x(end) - x(start):.2f}" height="{plot_h}" '
        'fill="#f2c14e" opacity="0.24" />'
        for start, end in dropouts
    )

    svg = f"""\
    <svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
      <rect width="100%" height="100%" fill="#ffffff"/>
      <text x="{margin_left}" y="20" font-family="sans-serif" font-size="16" fill="#222">Mocap IEKF figure-eight regression, 240 Hz, fixed one-second dropouts</text>
      {dropout_rects}
      <line x1="{margin_left}" y1="{margin_top + plot_h}" x2="{margin_left + plot_w}" y2="{margin_top + plot_h}" stroke="#222"/>
      <line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" stroke="#222"/>
      <polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{points('position_error_m')}"/>
      <polyline fill="none" stroke="#d62728" stroke-width="2" points="{points('attitude_error_rad')}"/>
      <polyline fill="none" stroke="#2ca02c" stroke-width="2" points="{points('velocity_error_m_s', 0.25)}"/>
      <text x="{margin_left}" y="{height - 16}" font-family="sans-serif" font-size="12" fill="#222">time [s]</text>
      <text x="14" y="{margin_top + 16}" font-family="sans-serif" font-size="12" fill="#222">error</text>
      <text x="{width - 360}" y="46" font-family="sans-serif" font-size="12" fill="#1f77b4">position [m]</text>
      <text x="{width - 252}" y="46" font-family="sans-serif" font-size="12" fill="#d62728">attitude [rad]</text>
      <text x="{width - 136}" y="46" font-family="sans-serif" font-size="12" fill="#2ca02c">velocity / 4</text>
    </svg>
    """
    path.write_text(textwrap.dedent(svg))


def _rust_harness(dropouts: list[tuple[float, float]]) -> str:
    dropout_literal = ", ".join(f"({start:.9}, {end:.9})" for start, end in dropouts)
    return textwrap.dedent(
        f"""
        use std::env;
        use std::fs::File;
        use std::io::{{BufWriter, Write}};

        #[path = "mocap_iekf_generated.rs"]
        mod generated;

        const ATTITUDE_OFFSET: usize = 0;
        const LINEAR_VELOCITY_OFFSET: usize = 4;
        const POSITION_OFFSET: usize = 7;
        const ANGULAR_VELOCITY_OFFSET: usize = 10;
        const COVARIANCE_OFFSET: usize = 13;
        const TANGENT_LEN: usize = 12;
        const MEAS_LEN: usize = 6;
        const DT_S: f64 = 1.0 / {MOCAP_HZ};
        const STEPS: usize = ({MISSION_S} * {MOCAP_HZ}) as usize;
        const DROPOUTS: [(f64, f64); {len(dropouts)}] = [{dropout_literal}];

        type State = [f64; generated::Y_LEN];
        type Params = [f64; generated::P_LEN];
        type Derivative = [f64; generated::DERIVATIVE_LEN];
        type Mat12 = [[f64; TANGENT_LEN]; TANGENT_LEN];
        type Mat6 = [[f64; MEAS_LEN]; MEAS_LEN];
        type Mat12x6 = [[f64; MEAS_LEN]; TANGENT_LEN];

        struct Estimator {{
            y: State,
            p: Params,
            dydt: Derivative,
            initialized: bool,
            last_t: f64,
            last_valid_t: f64,
        }}

        impl Estimator {{
            fn new() -> Self {{
                assert_eq!(generated::Y_LEN, COVARIANCE_OFFSET + TANGENT_LEN * TANGENT_LEN);
                assert_eq!(generated::DERIVATIVE_LEN, generated::Y_LEN);
                let mut p = [0.0; generated::P_LEN];
                p[0] = 1.0e-5;
                p[1] = 2.0e-2;
                p[2] = 1.0e-6;
                p[3] = 5.0e-3;
                Self {{
                    y: [0.0; generated::Y_LEN],
                    p,
                    dydt: [0.0; generated::DERIVATIVE_LEN],
                    initialized: false,
                    last_t: 0.0,
                    last_valid_t: 0.0,
                }}
            }}

            fn update(&mut self, t: f64, valid: bool, position: [f64; 3], attitude: [f64; 4]) {{
                if !self.initialized {{
                    if valid {{
                        self.initialize(t, position, attitude);
                    }}
                    return;
                }}
                self.predict_to(t);
                if valid {{
                    let residual = self.residual(position, attitude);
                    self.correct(residual);
                    self.last_valid_t = t;
                }}
            }}

            fn initialize(&mut self, t: f64, position: [f64; 3], attitude: [f64; 4]) {{
                self.y.fill(0.0);
                self.y[ATTITUDE_OFFSET..ATTITUDE_OFFSET + 4].copy_from_slice(&normalized_quat(attitude));
                self.y[POSITION_OFFSET..POSITION_OFFSET + 3].copy_from_slice(&position);
                let mut p = [[0.0; TANGENT_LEN]; TANGENT_LEN];
                for i in 0..3 {{
                    p[i][i] = 0.03_f64.powi(2);
                    p[3 + i][3 + i] = 3.0_f64.powi(2);
                    p[6 + i][6 + i] = 0.004_f64.powi(2);
                    p[9 + i][9 + i] = 1.0_f64.powi(2);
                }}
                self.write_covariance(&p);
                self.initialized = true;
                self.last_t = t;
                self.last_valid_t = t;
            }}

            fn predict_to(&mut self, t: f64) {{
                if !self.initialized || t <= self.last_t {{
                    self.last_t = self.last_t.max(t);
                    return;
                }}
                let dt = t - self.last_t;
                let steps = (dt / 0.01).ceil().max(1.0) as usize;
                let step = dt / steps as f64;
                for _ in 0..steps {{
                    generated::derivative_rhs(0.0, &self.y, &self.p, &mut self.dydt);
                    for i in 0..generated::DERIVATIVE_LEN {{
                        self.y[i] += self.dydt[i] * step;
                    }}
                    self.add_process_noise(step);
                    self.normalize_attitude();
                    self.symmetrize_covariance();
                }}
                self.last_t = t;
            }}

            fn add_process_noise(&mut self, dt: f64) {{
                let q = [1.0e-5, 2.0e-2, 1.0e-6, 5.0e-3];
                for axis in 0..3 {{
                    self.y[cov_index(axis, axis)] += q[0] * dt;
                    self.y[cov_index(3 + axis, 3 + axis)] += q[1] * dt;
                    self.y[cov_index(6 + axis, 6 + axis)] += q[2] * dt;
                    self.y[cov_index(9 + axis, 9 + axis)] += q[3] * dt;
                }}
            }}

            fn residual(&self, position: [f64; 3], attitude: [f64; 4]) -> [f64; MEAS_LEN] {{
                let dq = quat_multiply(quat_conjugate(self.attitude()), normalized_quat(attitude));
                let attitude_error = quat_log(dq);
                [
                    attitude_error[0],
                    attitude_error[1],
                    attitude_error[2],
                    position[0] - self.position()[0],
                    position[1] - self.position()[1],
                    position[2] - self.position()[2],
                ]
            }}

            fn correct(&mut self, residual: [f64; MEAS_LEN]) {{
                let p = self.covariance();
                let obs = [0_usize, 1, 2, 6, 7, 8];
                let mut s = [[0.0; MEAS_LEN]; MEAS_LEN];
                for a in 0..MEAS_LEN {{
                    for b in 0..MEAS_LEN {{
                        s[a][b] = p[obs[a]][obs[b]];
                    }}
                }}
                for i in 0..3 {{
                    s[i][i] += 0.010_f64.powi(2);
                    s[3 + i][3 + i] += 0.004_f64.powi(2);
                }}
                let Some(s_inv) = inverse6(s) else {{
                    return;
                }};
                let mut k = [[0.0; MEAS_LEN]; TANGENT_LEN];
                for row in 0..TANGENT_LEN {{
                    for meas in 0..MEAS_LEN {{
                        for j in 0..MEAS_LEN {{
                            k[row][meas] += p[row][obs[j]] * s_inv[j][meas];
                        }}
                    }}
                }}
                let mut dx = [0.0; TANGENT_LEN];
                for row in 0..TANGENT_LEN {{
                    for meas in 0..MEAS_LEN {{
                        dx[row] += k[row][meas] * residual[meas];
                    }}
                }}
                self.inject(dx);

                let mut ks = [[0.0; MEAS_LEN]; TANGENT_LEN];
                for row in 0..TANGENT_LEN {{
                    for b in 0..MEAS_LEN {{
                        for a in 0..MEAS_LEN {{
                            ks[row][b] += k[row][a] * s[a][b];
                        }}
                    }}
                }}
                let mut next = p;
                for row in 0..TANGENT_LEN {{
                    for col in 0..TANGENT_LEN {{
                        let mut delta = 0.0;
                        for a in 0..MEAS_LEN {{
                            delta += ks[row][a] * k[col][a];
                        }}
                        next[row][col] -= delta;
                    }}
                }}
                stabilize_covariance(&mut next);
                self.write_covariance(&next);
            }}

            fn inject(&mut self, dx: [f64; TANGENT_LEN]) {{
                let dq = quat_exp([dx[0], dx[1], dx[2]]);
                let q = quat_multiply(self.attitude(), dq);
                self.y[ATTITUDE_OFFSET..ATTITUDE_OFFSET + 4].copy_from_slice(&normalized_quat(q));
                for i in 0..3 {{
                    self.y[LINEAR_VELOCITY_OFFSET + i] += dx[3 + i];
                    self.y[POSITION_OFFSET + i] += dx[6 + i];
                    self.y[ANGULAR_VELOCITY_OFFSET + i] += dx[9 + i];
                }}
            }}

            fn normalize_attitude(&mut self) {{
                let q = normalized_quat(self.attitude());
                self.y[ATTITUDE_OFFSET..ATTITUDE_OFFSET + 4].copy_from_slice(&q);
            }}

            fn symmetrize_covariance(&mut self) {{
                let mut p = self.covariance();
                stabilize_covariance(&mut p);
                self.write_covariance(&p);
            }}

            fn attitude(&self) -> [f64; 4] {{
                [self.y[0], self.y[1], self.y[2], self.y[3]]
            }}

            fn position(&self) -> [f64; 3] {{
                [self.y[7], self.y[8], self.y[9]]
            }}

            fn velocity(&self) -> [f64; 3] {{
                [self.y[4], self.y[5], self.y[6]]
            }}

            fn finite(&self) -> bool {{
                self.y.iter().all(|value| value.is_finite())
            }}

            fn covariance(&self) -> Mat12 {{
                let mut p = [[0.0; TANGENT_LEN]; TANGENT_LEN];
                for row in 0..TANGENT_LEN {{
                    for col in 0..TANGENT_LEN {{
                        p[row][col] = self.y[cov_index(row, col)];
                    }}
                }}
                p
            }}

            fn write_covariance(&mut self, p: &Mat12) {{
                for row in 0..TANGENT_LEN {{
                    for col in 0..TANGENT_LEN {{
                        self.y[cov_index(row, col)] = p[row][col];
                    }}
                }}
            }}
        }}

        fn main() -> std::io::Result<()> {{
            let output = env::args().nth(1).expect("expected output CSV path");
            let mut file = BufWriter::new(File::create(output)?);
            writeln!(
                file,
                "time_s,measurement_valid,initialized,finite,position_error_m,attitude_error_rad,velocity_error_m_s"
            )?;
            let mut estimator = Estimator::new();
            for step in 0..=STEPS {{
                let t = step as f64 * DT_S;
                let truth = truth_state(t);
                let valid = !in_dropout(t);
                let measured_position = if valid {{ noisy_position(t, truth.position) }} else {{ truth.position }};
                let measured_attitude = if valid {{ noisy_attitude(t, truth.attitude) }} else {{ truth.attitude }};
                estimator.update(t, valid, measured_position, measured_attitude);
                let position_error = norm3(sub3(estimator.position(), truth.position));
                let attitude_error = norm3(quat_log(quat_multiply(
                    quat_conjugate(truth.attitude),
                    estimator.attitude(),
                )));
                let velocity_error = norm3(sub3(estimator.velocity(), truth.velocity));
                writeln!(
                    file,
                    "{{:.9}},{{}},{{}},{{}},{{:.9}},{{:.9}},{{:.9}}",
                    t,
                    if valid {{ 1 }} else {{ 0 }},
                    if estimator.initialized {{ 1 }} else {{ 0 }},
                    if estimator.finite() {{ 1 }} else {{ 0 }},
                    position_error,
                    attitude_error,
                    velocity_error,
                )?;
            }}
            Ok(())
        }}

        #[derive(Clone, Copy)]
        struct Truth {{
            position: [f64; 3],
            velocity: [f64; 3],
            attitude: [f64; 4],
        }}

        fn truth_state(t: f64) -> Truth {{
            let w = 2.0 * std::f64::consts::PI / 10.0;
            let x = (w * t).sin();
            let y = 0.45 * (2.0 * w * t).sin();
            let z = 1.5 + 0.12 * (0.5 * w * t).sin();
            let vx = w * (w * t).cos();
            let vy = 0.45 * 2.0 * w * (2.0 * w * t).cos();
            let vz = 0.12 * 0.5 * w * (0.5 * w * t).cos();
            let yaw = 0.35 * (0.7 * t).sin();
            Truth {{
                position: [x, y, z],
                velocity: [vx, vy, vz],
                attitude: [f64::cos(0.5 * yaw), 0.0, 0.0, f64::sin(0.5 * yaw)],
            }}
        }}

        fn noisy_position(t: f64, p: [f64; 3]) -> [f64; 3] {{
            [
                p[0] + 0.0015 * (17.0 * t).sin(),
                p[1] + 0.0015 * (19.0 * t + 0.4).sin(),
                p[2] + 0.0010 * (23.0 * t + 0.9).sin(),
            ]
        }}

        fn noisy_attitude(t: f64, q: [f64; 4]) -> [f64; 4] {{
            let noise = [0.0015 * (13.0 * t).sin(), 0.0010 * (11.0 * t + 0.2).sin(), 0.0020 * (7.0 * t).sin()];
            normalized_quat(quat_multiply(q, quat_exp(noise)))
        }}

        fn in_dropout(t: f64) -> bool {{
            DROPOUTS.iter().any(|(start, end)| t >= *start && t < *end)
        }}

        fn cov_index(row: usize, col: usize) -> usize {{
            COVARIANCE_OFFSET + col * TANGENT_LEN + row
        }}

        fn stabilize_covariance(p: &mut Mat12) {{
            for row in 0..TANGENT_LEN {{
                for col in row + 1..TANGENT_LEN {{
                    let value = 0.5 * (p[row][col] + p[col][row]);
                    p[row][col] = value;
                    p[col][row] = value;
                }}
                if !p[row][row].is_finite() || p[row][row] < 1.0e-12 {{
                    p[row][row] = 1.0e-12;
                }}
            }}
        }}

        fn inverse6(mut a: Mat6) -> Option<Mat6> {{
            let mut inv = [[0.0; MEAS_LEN]; MEAS_LEN];
            for i in 0..MEAS_LEN {{
                inv[i][i] = 1.0;
            }}
            for pivot in 0..MEAS_LEN {{
                let mut best = pivot;
                let mut best_abs = a[pivot][pivot].abs();
                for row in pivot + 1..MEAS_LEN {{
                    if a[row][pivot].abs() > best_abs {{
                        best = row;
                        best_abs = a[row][pivot].abs();
                    }}
                }}
                if best_abs < 1.0e-18 {{
                    return None;
                }}
                if best != pivot {{
                    a.swap(best, pivot);
                    inv.swap(best, pivot);
                }}
                let diag = a[pivot][pivot];
                for col in 0..MEAS_LEN {{
                    a[pivot][col] /= diag;
                    inv[pivot][col] /= diag;
                }}
                for row in 0..MEAS_LEN {{
                    if row == pivot {{
                        continue;
                    }}
                    let factor = a[row][pivot];
                    for col in 0..MEAS_LEN {{
                        a[row][col] -= factor * a[pivot][col];
                        inv[row][col] -= factor * inv[pivot][col];
                    }}
                }}
            }}
            Some(inv)
        }}

        fn normalized_quat(q: [f64; 4]) -> [f64; 4] {{
            let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
            if !norm.is_finite() || norm <= f64::EPSILON {{
                [1.0, 0.0, 0.0, 0.0]
            }} else {{
                [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
            }}
        }}

        fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {{
            [q[0], -q[1], -q[2], -q[3]]
        }}

        fn quat_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {{
            [
                a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
            ]
        }}

        fn quat_exp(phi: [f64; 3]) -> [f64; 4] {{
            let theta = norm3(phi);
            if theta <= 1.0e-12 {{
                return normalized_quat([1.0, 0.5 * phi[0], 0.5 * phi[1], 0.5 * phi[2]]);
            }}
            let half = 0.5 * theta;
            let scale = half.sin() / theta;
            [half.cos(), scale * phi[0], scale * phi[1], scale * phi[2]]
        }}

        fn quat_log(q: [f64; 4]) -> [f64; 3] {{
            let mut qn = normalized_quat(q);
            if qn[0] < 0.0 {{
                qn = [-qn[0], -qn[1], -qn[2], -qn[3]];
            }}
            let vnorm = (qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]).sqrt();
            if vnorm <= 1.0e-12 {{
                [2.0 * qn[1], 2.0 * qn[2], 2.0 * qn[3]]
            }} else {{
                let angle = 2.0 * vnorm.atan2(qn[0]);
                let scale = angle / vnorm;
                [scale * qn[1], scale * qn[2], scale * qn[3]]
            }}
        }}

        fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {{
            [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
        }}

        fn norm3(v: [f64; 3]) -> f64 {{
            (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
        }}
        """
    )
