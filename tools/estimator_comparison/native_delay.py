#!/usr/bin/env python3
"""Replay pinned native cores with the ESKF comparison's packet arrival trace."""

import argparse
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np

from compare_delay import PROFILES
from native_phase import flight_phase_writer
from native_noise import native_noise, PREDICTION_PERIOD_S
from native_aiding_noise import sensor_informed_noise
from flow_native import ardupilot_rates, px4_source, verify_dataflash
from score import POSITION, QUATERNION, VELOCITY, metrics, read, transition


def common_epochs(estimate, times):
    source_times = estimate["t_s"]
    if times[0] < source_times[0] or times[-1] > source_times[-1]:
        raise ValueError("Common scoring epochs require extrapolation")
    left = np.maximum(np.searchsorted(source_times, times, side="right") - 1, 0)
    right = np.minimum(left + 1, len(estimate) - 1)
    if np.any(source_times[right] - source_times[left] > 0.010001):
        raise ValueError("Native states have a gap larger than one reporting interval")
    result = estimate[left].copy()
    result["t_s"] = times
    for field in (*POSITION, *VELOCITY):
        result[field] = np.interp(times, source_times, estimate[field])
    quaternion = np.column_stack([estimate[field] for field in QUATERNION])
    signs = np.where(np.sum(quaternion[1:] * quaternion[:-1], axis=1) < 0, -1, 1)
    quaternion *= np.r_[1, np.cumprod(signs)][:, None]
    interpolated = np.column_stack(
        [np.interp(times, source_times, quaternion[:, index]) for index in range(4)]
    )
    norm = np.linalg.norm(interpolated, axis=1)
    if not np.isfinite(norm).all() or np.any(norm < 1e-12):
        raise ValueError("Native quaternion cannot be interpolated")
    interpolated /= norm[:, None]
    for index, field in enumerate(QUATERNION):
        result[field] = interpolated[:, index]
    return result


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError(f"External adapter boundary changed: {before}")
    return text.replace(before, after)


def prepare_px4(args):
    source = args.harness / "px4/main.cpp"
    if hashlib.sha256(source.read_bytes()).hexdigest() != (
        "b09280ae5f59b492aab5b6c573e5ebdf3ae5c4dc47752f1ade9e10757a7495ae"
    ):
        raise ValueError("Use the audited 3dbaeb9 external PX4 adapter")
    text = source.read_text()
    injection = r"""
    std::vector<std::string> arrival_header;
    const auto arrival_rows = read_csv(input + "/arrivals.csv", arrival_header);
    auto schedule_source = [&](auto &samples, unsigned source) {
        auto original = samples;
        samples.clear();
        std::vector<double> arrivals;
        size_t index = 0;
        for (const auto &row : arrival_rows) {
            if ((unsigned)row[1] != source) continue;
            while (index < original.size() && original[index].t < row[2] - 1e-8) ++index;
            if (index == original.size() || fabs(original[index].t - row[2]) > 1e-8) {
                fprintf(stderr, "Missing original sensor packet at %.9f\n", row[2]); exit(2);
            }
            samples.push_back(original[index]);
            arrivals.push_back(row[0]);
        }
        return arrivals;
    };
    const auto gps_arrivals = schedule_source(gps, 0);
    const auto flow_arrivals = schedule_source(flow, 1);
    const auto mag_arrivals = schedule_source(mag, 2);
    const auto baro_arrivals = schedule_source(baro, 3);
"""
    text = replace_once(text, "\tEkf ekf;", injection + "\n\tEkf ekf;")
    for samples, index in (
        ("gps", "ig"),
        ("flow", "ifl"),
        ("mag", "im"),
        ("baro", "ib"),
    ):
        text = replace_once(
            text,
            f"{samples}[{index}].t <= s.t",
            f"{samples}_arrivals[{index}] <= s.t + 1e-9",
        )
    text = replace_once(
        text,
        'p->ekf2_of_delay = json_number(ojs.str(), "flow_latency_ms");',
        "p->ekf2_of_delay = 0; p->ekf2_mag_delay = 0; "
        "p->ekf2_baro_delay = 0; p->ekf2_rng_delay = 0; p->ekf2_delay_max = 200;",
    )
    text = replace_once(
        text,
        "fs.gyro_rate = -gyro_frd_latest;",
        "const auto &measured_imu = imu.at((size_t)llround((f.t - t_start) / .00125));\n"
        "fs.gyro_rate = -Vector3f(measured_imu.g[0], -measured_imu.g[1], -measured_imu.g[2]);",
    )
    text = replace_once(
        text,
        "if (!g.pos_valid) continue;",
        'if (!g.pos_valid) continue; if (!g.vel_valid) { fprintf(stderr, "Partial GPS velocity is unsupported\\n"); return 2; }',
    )
    if getattr(args, "imu_noise_density", None):
        gyro_noise, accel_noise = native_noise(args.imu_noise_density)["px4"]
        text = replace_once(
            text,
            "\tif (!use_gps) p->ekf2_gps_ctrl = 0;",
            f"\tp->ekf2_gyr_noise = {gyro_noise:.12e}f;\n"
            f"\tp->ekf2_acc_noise = {accel_noise:.12e}f;\n"
            "\tif (!use_gps) p->ekf2_gps_ctrl = 0;",
        )
    if getattr(args, "exposure_flow", False):
        text = px4_source(text, replace_once)
    if getattr(args, "sensor_informed_noise", False):
        parameters = sensor_informed_noise()["px4"]
        text = replace_once(
            text,
            "\tif (!use_gps) p->ekf2_gps_ctrl = 0;",
            "".join(
                f"\tp->{name} = {value:.12e}f;\n" for name, value in parameters.items()
            )
            + "\tif (!use_gps) p->ekf2_gps_ctrl = 0;",
        )
    if getattr(args, "px4_release", None):
        from native_release import px4_release_adapter

        text = px4_release_adapter(text, args.px4_release, replace_once)
    staging = args.work / "px4-arrivals.cpp"
    staging.write_text(text)
    px4 = args.px4_source
    includes = [
        args.harness / "px4/stubs",
        px4 / "src",
        px4 / "src/lib",
        px4 / "src/lib/matrix",
        px4 / "src/modules/ekf2/EKF",
        px4 / "src/modules/ekf2/EKF/python",
    ]
    binary = args.work / "px4-arrivals"
    subprocess.run(
        [
            args.cxx,
            "-pipe",
            "-O3",
            "-DNDEBUG",
            "-std=c++17",
            *[
                f"-DCONFIG_EKF2_{feature}=1"
                for feature in (
                    "BAROMETER",
                    "GNSS",
                    "GRAVITY_FUSION",
                    "MAGNETOMETER",
                    "OPTICAL_FLOW",
                    "RANGE_FINDER",
                    "TERRAIN",
                )
            ],
            '-DMODULE_NAME="ekf2_replay"',
            "-D__PX4_POSIX=1",
            *["-I" + str(path) for path in includes],
            str(staging),
            str(args.px4_library),
            "-lm",
            "-o",
            str(binary),
        ],
        check=True,
    )
    return binary


def scheduled_capture(converter, source, arrivals):
    original_load = converter.load_csv

    def load(path):
        rows = original_load(path)
        filename = Path(path).name
        if filename not in ("gps.csv", "flow.csv", "mag.csv", "baro.csv"):
            return rows
        source_index = ("gps.csv", "flow.csv", "mag.csv", "baro.csv").index(filename)
        selected = arrivals[arrivals["source"] == source_index]
        indices = np.searchsorted(rows["t_s"], selected["measurement_t_s"] - 1e-8)
        if len(indices) and (
            np.any(indices >= len(rows["t_s"]))
            or not np.allclose(
                rows["t_s"][indices], selected["measurement_t_s"], rtol=0, atol=1e-8
            )
        ):
            raise ValueError("Native capture lacks a scheduled original measurement")
        result = {key: values[indices] for key, values in rows.items()}
        if filename == "gps.csv" and any(
            np.any(result[key] < 0.5)
            for key in ("pos_valid", "vel_valid", "vel_down_valid")
        ):
            raise ValueError("Partial GPS velocity is unsupported in this campaign")
        result["measurement_t_s"] = result["t_s"].copy()
        result["t_s"] = selected["arrival_t_s"]
        return result

    converter.load_csv = load
    return load(source / "gps.csv"), load(source / "flow.csv")


def run_ardupilot(args, capture, arrivals, delay_profile, scenario, output):
    converter = load_module(args.harness / "ardupilot/csv_to_dataflash.py", "converter")
    parser = load_module(args.harness / "ardupilot/run_ekf3.py", "parser")
    gps, flow = scheduled_capture(converter, capture, arrivals)
    gyro = arrivals[arrivals["source"] == 1]
    clear_at_s = getattr(args, "takeoff_clear_s", 18.0)
    original_writer = flight_phase_writer(converter.Writer, clear_at_s)
    writers = []

    class ScheduledWriter(original_writer):
        gps_index = 0
        flow_index = 0

        def __init__(self, path):
            super().__init__(path)
            self.sent = [0, 0, 0, 0]
            writers.append(self)

        def msg(self, name, *values):
            values = list(values)
            if name == "RGPI":
                values[3] = (
                    gps["t_s"][self.gps_index] - gps["measurement_t_s"][self.gps_index]
                    if values[5] >= 3
                    else 0.2
                )
                values[4] |= 1
            elif name == "RGPJ":
                self.gps_index += 1
            elif name == "ROFH":
                index = self.flow_index
                gx, gy = gyro["gx"][index], -gyro["gy"][index]
                distance = flow["dist_m"][index]
                values[:4] = [
                    flow["vy_flu_m_s"][index] / distance + gx,
                    flow["vx_flu_m_s"][index] / distance + gy,
                    gx,
                    gy,
                ]
                if getattr(args, "exposure_flow", False):
                    values[:4] = ardupilot_rates(flow, index).tolist()
                    values[4] = round(flow["measurement_t_s"][index] * 1000)
                self.flow_index += 1
            if name in ("RGPJ", "ROFH", "RMGI", "RBRI"):
                self.sent[("RGPJ", "ROFH", "RMGI", "RBRI").index(name)] += 1
            super().msg(name, *values)

    converter.Writer = ScheduledWriter
    gps_delay, flow_delay, _, baro_delay, jitter = PROFILES[delay_profile]
    mean_quantization = 5 if jitter else 0
    replay_work = args.work / "ardupilot-run"
    replay_work.mkdir()
    dataflash = replay_work / "replayin.bin"
    options = argparse.Namespace(
        input=str(capture),
        output=str(dataflash),
        no_gps=scenario == "denied",
        no_flow=False,
        gps_deny=[],
        nominal_dt=0.00125,
        max_dt=0.02,
        arm_after=13,
        gps_check=31,
        gps_lag=0,
        log_period=0.01,
        flow_sign=1,
        param=[
            ("EK3_SRC1_VELZ", 3),
            ("EK3_BCN_DELAY", 200),
            ("EK3_HGT_DELAY", baro_delay + jitter / 2 + mean_quantization),
            ("EK3_FLOW_DELAY", flow_delay + jitter / 2 + mean_quantization),
        ],
    )
    if getattr(args, "imu_noise_density", None):
        gyro_noise, accel_noise = native_noise(args.imu_noise_density)["ekf3"]
        options.param += [
            ("EK3_GYRO_P_NSE", gyro_noise),
            ("EK3_ACC_P_NSE", accel_noise),
        ]
    if getattr(args, "sensor_informed_noise", False):
        options.param += list(sensor_informed_noise()["ekf3"].items())
    converter.convert(options)
    flow_serialization = (
        verify_dataflash(dataflash, flow)
        if getattr(args, "exposure_flow", False)
        else None
    )
    expected = [
        int(np.count_nonzero(arrivals["source"] == source)) for source in range(4)
    ]
    if len(writers) != 1 or writers[0].sent != expected:
        raise ValueError(
            "DataFlash conversion dropped or duplicated a scheduled packet"
        )
    with (replay_work / "stdout.txt").open("w") as log:
        subprocess.run(
            [str(args.ap_replay), str(dataflash)],
            cwd=replay_work,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    logs = sorted((replay_work / "logs").glob("*.BIN"))
    if not logs:
        raise ValueError("Native ArduPilot Replay produced no output log")
    origin = json.loads((capture / "origin.json").read_text())
    count = parser.parse_output(str(logs[-1]), str(output), origin["alt_msl_m"])
    log_text = (replay_work / "stdout.txt").read_text(errors="replace")
    if not count:
        raise ValueError("Native ArduPilot Replay produced no states")
    result = dict(
        output_log_sha256=hashlib.sha256(logs[-1].read_bytes()).hexdigest(),
        replay_log=log_text,
        output_rows=count,
        adapter_sent=writers[0].sent,
        takeoff_expectation_clear_s=clear_at_s,
    )
    if flow_serialization is not None:
        result["flow_serialization"] = flow_serialization
    shutil.rmtree(replay_work)
    return result


def run(args):
    if args.output.exists():
        raise ValueError("Choose a new evidence filename")
    if getattr(args, "imu_noise_density", None):
        native_noise(args.imu_noise_density)
    args.work.mkdir(parents=True, exist_ok=True)
    pins = {}
    for name, source, expected in (
        ("px4", args.px4_source, "f1c0a1f794edf8e5e974b6ed96df3f95eda0df39"),
        ("ardupilot", args.ap_source, "1511f27194f1dcc3728270883047bdf022b3fd53"),
    ):
        pins[name] = subprocess.check_output(
            ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
        ).strip()
        if pins[name] != expected:
            raise ValueError("Native source pin changed")
    external_hashes = {
        "ardupilot/csv_to_dataflash.py": "1c165fda77af2dd39e2a9c96685441abf65893503618c230e3f436f47750e7eb",
        "ardupilot/run_ekf3.py": "46bbfaf1ae961c45c4c9f3fa6d357fe3e238d67a8cf48001c3c8caadc8ad61a0",
    }
    for name, expected in external_hashes.items():
        if hashlib.sha256((args.harness / name).read_bytes()).hexdigest() != expected:
            raise ValueError("Use the audited 3dbaeb9 external ArduPilot adapter")
    px4 = prepare_px4(args)
    references = json.loads(args.eskf_scores.read_text())["scores"]
    evidence = dict(
        native_source_pins=pins,
        external_adapter_sha256=external_hashes,
        binary_sha256={
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name, path in (
                ("px4", px4),
                ("ekf3", args.ap_replay),
                ("px4_core_library", args.px4_library),
                ("transport_trace", args.transport_trace),
            )
        },
        patched_px4_adapter_sha256=hashlib.sha256(
            (args.work / "px4-arrivals.cpp").read_bytes()
        ).hexdigest(),
        eskf_evidence_sha256=hashlib.sha256(args.eskf_scores.read_bytes()).hexdigest(),
        configuration=dict(
            px4_delay_max_ms=200,
            px4_timestamp_delays_ms=0,
            ekf3_reserved_buffer_delay_ms=200,
            ekf3_reserve_parameter="EK3_BCN_DELAY (no beacon measurements)",
            ekf3_gps_driver_lag="Exact arrival age per packet; full three-axis velocity enabled",
            ekf3_flow_and_height_delay="Declared mean transport age, including positive jitter and 100 Hz quantization",
            ekf3_magnetic_delay_ms=60,
            ekf3_takeoff_expectation_clear_s=getattr(args, "takeoff_clear_s", 18.0),
        ),
        limitations=[
            "Declared stationary phase uses native at-rest/armed interfaces; generated ESKF uses its explicit rest input",
            "Takeoff expectation clears at a declared timeout, not the full feedback-dependent Copter detector",
            "Native priors, process/measurement tuning, sensor selection and complementary output dynamics remain stack-specific",
            "EKF3 reconstructs flow/magnetic epochs using native fixed-delay rules; preserved packet timestamps do not override those rules",
            "EKF3 uses barometric height while PX4/ESKF can also fuse GPS height; all three receive three-axis GPS velocity",
            "Synthetic body-velocity/range flow reconstruction; not a real raw-image disturbance campaign",
            "Native source-active flags do not prove individual measurement acceptance",
            "Existing audited native library/binary reused; native cores were not freshly rebuilt in this restricted session",
        ],
    )
    records = []
    if getattr(args, "imu_noise_density", None):
        evidence["configuration"]["imu_noise_density"] = args.imu_noise_density
        evidence["configuration"]["native_rate_noise"] = native_noise(
            args.imu_noise_density
        )
        evidence["configuration"]["nominal_prediction_period_s"] = PREDICTION_PERIOD_S
    for lower in (False, True):
        for frequency in args.frequencies:
            for seed in args.seeds:
                tag = ("lower_" if lower else "") + f"{frequency}_{seed}"
                capture = args.captures / tag
                truth = read(capture / "truth.csv")
                gps_capture = read(capture / "gps.csv")
                if any(
                    np.any(gps_capture[field] < 0.5)
                    for field in ("pos_valid", "vel_valid", "vel_down_valid")
                ):
                    raise ValueError(
                        "This native campaign requires valid position and three-axis GPS velocity"
                    )
                case_input = args.work / "capture"
                case_input.mkdir(exist_ok=True)
                for name in (
                    "imu.csv",
                    "gps.csv",
                    "flow.csv",
                    "mag.csv",
                    "baro.csv",
                    "origin.json",
                ):
                    target = case_input / name
                    if target.is_symlink():
                        target.unlink()
                    target.symlink_to(capture / name)
                for delay_profile in args.delay_profiles:
                    for scenario in args.scenarios:
                        trace = subprocess.run(
                            [
                                str(args.transport_trace),
                                str(capture / "modelica_input.csv"),
                                scenario,
                                *map(str, PROFILES[delay_profile]),
                                str(seed),
                            ],
                            capture_output=True,
                            text=True,
                            check=True,
                        )
                        arrivals = np.atleast_1d(
                            np.genfromtxt(
                                io.StringIO(trace.stdout), names=True, delimiter=","
                            )
                        )
                        transport = json.loads(trace.stderr)
                        reference = next(
                            record
                            for record in references
                            if record["tag"] == tag
                            and record["scenario"] == scenario
                            and record["delay_profile"] == delay_profile
                        )
                        expected_transport = reference["timing"][0]
                        if any(
                            transport[key] != expected_transport[key]
                            for key in transport
                        ):
                            raise ValueError(
                                "Native arrival trace differs from the ESKF packet delivery"
                            )
                        (case_input / "arrivals.csv").write_text(trace.stdout)
                        for name in args.estimators:
                            output = args.work / (name + ".csv")
                            details = {}
                            if name == "px4":
                                completed = subprocess.run(
                                    [
                                        str(px4),
                                        "--input",
                                        str(case_input),
                                        "--output",
                                        str(output),
                                        *(["--no-gps"] if scenario == "denied" else []),
                                    ],
                                    capture_output=True,
                                    text=True,
                                    check=True,
                                )
                                details["replay_log"] = completed.stderr
                            else:
                                details = run_ardupilot(
                                    args,
                                    case_input,
                                    arrivals,
                                    delay_profile,
                                    scenario,
                                    output,
                                )
                            estimate = read(output)
                            if not len(estimate) or not all(
                                np.isfinite(estimate[f]).all()
                                for f in estimate.dtype.names
                            ):
                                raise ValueError(
                                    "Native replay contains nonfinite or missing states"
                                )
                            flight_times = estimate["t_s"][
                                (estimate["t_s"] >= 13) & (estimate["t_s"] < 60)
                            ]
                            if len(flight_times) != 4700:
                                raise ValueError(
                                    "Native replay lacks the common 100 Hz flight scoring epochs"
                                )
                            estimate = common_epochs(
                                estimate, np.arange(1300, 6000) / 100
                            )
                            record = dict(
                                name=name,
                                tag=tag,
                                frequency=frequency,
                                seed=seed,
                                profile="25/25/25 Hz" if lower else "100/50/50 Hz",
                                scenario=scenario,
                                delay_profile=delay_profile,
                                output_sha256=hashlib.sha256(
                                    output.read_bytes()
                                ).hexdigest(),
                                transport=transport,
                                arrival_trace_sha256=hashlib.sha256(
                                    trace.stdout.encode()
                                ).hexdigest(),
                                scored_state_sha256=hashlib.sha256(
                                    estimate.tobytes()
                                ).hexdigest(),
                                **details,
                            )
                            for window, start, end in (
                                ("flight", 13, 60),
                                ("before_outage", 18, 25),
                                ("outage_window", 25, 40),
                                ("after_return", 40, 60),
                            ):
                                record[window] = metrics(estimate, truth, start, end)
                            if scenario == "transition":
                                record["transition"] = transition(estimate, truth)
                                record["transition"][
                                    "first_gps_source_active_after_return_s"
                                ] = record["transition"].pop(
                                    "first_gps_use_after_return_s"
                                )
                            records.append(record)
                            output.unlink()
                            print(tag, delay_profile, scenario, name, flush=True)
                        args.output.write_text(
                            json.dumps(
                                dict(**evidence, scores=records),
                                indent=2,
                                allow_nan=False,
                            )
                            + "\n"
                        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "harness",
        "px4-source",
        "px4-library",
        "ap-replay",
        "ap-source",
        "eskf-scores",
        "transport-trace",
        "captures",
        "work",
        "output",
    ):
        parser.add_argument(
            "--" + name, type=lambda value: Path(value).resolve(), required=True
        )
    parser.add_argument("--cxx", default="c++")
    parser.add_argument(
        "--imu-noise-density", type=float, nargs=2, metavar=("GYRO", "ACCEL")
    )
    phase = parser.add_mutually_exclusive_group()
    phase.add_argument("--takeoff-clear-s", type=float, default=18.0)
    phase.add_argument(
        "--persistent-takeoff", action="store_const", const=None, dest="takeoff_clear_s"
    )
    parser.add_argument("--frequencies", type=float, nargs="+", default=[0.6])
    parser.add_argument("--seeds", type=int, nargs="+", default=[7, 19, 41])
    parser.add_argument(
        "--delay-profiles", choices=PROFILES, nargs="+", default=list(PROFILES)
    )
    parser.add_argument(
        "--scenarios",
        choices=("gps", "denied", "transition"),
        nargs="+",
        default=["gps", "denied", "transition"],
    )
    parser.add_argument(
        "--estimators", choices=("px4", "ekf3"), nargs="+", default=["px4", "ekf3"]
    )
    run(parser.parse_args())
