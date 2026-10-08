#!/usr/bin/env python3
"""Probe EKF3 replay configuration without modifying its pinned native core."""

import argparse
from collections import Counter
import hashlib
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from pymavlink import DFReader

import native_delay
from compare_delay import PROFILES
from score import metrics, read
from native_aiding_noise import sensor_informed_noise


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def diagnostics(path):
    log = DFReader.DFReader_binary(str(path))
    selections = Counter()
    statuses = Counter()
    timing = []
    flow = []
    parameters = {}
    while (message := log.recv_msg()) is not None:
        kind = message.get_type()
        if kind == "PARM":
            parameters[message.Name] = message.Value
        if kind == "XKT" and message.C in (0, 100):
            timing.append(message.to_dict())
        if getattr(message, "TimeUS", 0) < 13_000_000:
            continue
        if kind == "XKFS" and message.C in (0, 100):
            selections[(int(message.SS), int(message.MAG_FUSION))] += 1
        elif kind == "XKF4" and message.C in (0, 100):
            statuses[int(message.SS)] += 1
        elif kind == "XKF5" and message.C in (0, 100):
            flow.append((message.NI, message.FIX, message.FIY, message.HAGL))
    values = np.asarray(flow)
    return dict(
        magnetic_selection_rows=[
            dict(source_set=key[0], mag_fusion=key[1], rows=count)
            for key, count in sorted(selections.items())
        ],
        status_rows={str(key): count for key, count in sorted(statuses.items())},
        timing=timing,
        logged_flow=dict(
            rows=len(flow),
            median=np.median(values, axis=0).tolist() if len(flow) else [],
            max_abs=np.max(np.abs(values), axis=0).tolist() if len(flow) else [],
            fields=["NI", "FIX", "FIY", "HAGL"],
        ),
        parameters={
            key: value
            for key, value in parameters.items()
            if key.startswith(("EK3_", "COMPASS_"))
        },
    )


def probe(args, capture, arrivals, scenario, variant):
    original_load = native_delay.load_module
    original_remove = native_delay.shutil.rmtree
    result = {}

    def load(path, name):
        module = original_load(path, name)
        if name != "converter" or variant == "baseline":
            return module
        writer = module.Writer

        class FlightWriter(writer):
            flight = None
            released = False

            def msg(self, kind, *values):
                super().msg(kind, *values)
                if kind == "RFRN":
                    self.flight = list(values)
                if kind == "RFRH" and values[0] >= 18_000_000 and not self.released:
                    self.flight[-1] &= ~(1 << 6)
                    super().msg("RFRN", *self.flight)
                    self.released = True

        module.Writer = FlightWriter
        if variant == "finite_takeoff_mag4":
            convert = module.convert

            def configure(options):
                options.param.append(("EK3_MAG_CAL", 4))
                return convert(options)

            module.convert = configure
        return module

    def remove(path, *positional, **keywords):
        if Path(path) == args.work / "ardupilot-run":
            logs = sorted((Path(path) / "logs").glob("*.BIN"))
            result.update(diagnostics(logs[-1]))
            if getattr(args, "retain_native_run", False):
                return None
        return original_remove(path, *positional, **keywords)

    native_delay.load_module = load
    native_delay.shutil.rmtree = remove
    output = args.work / "estimate.csv"
    try:
        details = native_delay.run_ardupilot(
            args, capture, arrivals, args.delay_profile, scenario, output
        )
    finally:
        native_delay.load_module = original_load
        native_delay.shutil.rmtree = original_remove
    estimate = native_delay.common_epochs(read(output), np.arange(1300, 6000) / 100)
    truth = read(args.capture / "truth.csv")
    result.update(
        variant=variant,
        scenario=scenario,
        output_sha256=digest(output),
        scored_state_sha256=hashlib.sha256(estimate.tobytes()).hexdigest(),
        flight=metrics(estimate, truth, 13, 60),
        outage_window=metrics(estimate, truth, 25, 40),
        after_return=metrics(estimate, truth, 40, 60),
        **details,
    )
    if not getattr(args, "retain_output", False):
        output.unlink()
    return result


def prepare_px4(args):
    replace = native_delay.replace_once
    injection = r"""
            if (s.t >= 13 && s.t < 60) {
                const auto &audit_flags = ekf.control_status_flags();
                fprintf(stderr, "AUDIT %.8f %d %d %d %d %d %.9f %llu %llu %llu %llu %llu %.9e %.9e\n",
                    s.t, (int)audit_flags.mag_3D, (int)audit_flags.mag_hdg,
                    (int)audit_flags.mag_aligned_in_flight, (int)audit_flags.gps_hgt,
                    (int)audit_flags.opt_flow, ekf.get_dt_ekf_avg(),
                    (unsigned long long)ekf.aid_src_mag().time_last_fuse,
                    (unsigned long long)ekf.aid_src_optical_flow().time_last_fuse,
                    (unsigned long long)ekf.aid_src_gnss_pos().time_last_fuse,
                    (unsigned long long)ekf.aid_src_gnss_vel().time_last_fuse,
                    (unsigned long long)ekf.aid_src_baro_hgt().time_last_fuse,
                    p->ekf2_gyr_noise, p->ekf2_acc_noise);
            }
"""
    if getattr(args, "sensor_informed_noise", False):
        values = "".join(
            f'fprintf(stderr,"NATIVE_PARAM {name} %.12g\\n", (double)p->{name});\n'
            for name in sensor_informed_noise()["px4"]
        )
        injection += (
            "static bool reported_parameters = false;\n"
            "if (!reported_parameters && s.t >= 13) {\n"
            "reported_parameters = true;\n" + values + "}\n"
        )

    def instrument(text, before, after):
        result = replace(text, before, after)
        if before == "\tEkf ekf;":
            result = replace(result, "++n_updates;", "++n_updates;" + injection)
        return result

    native_delay.replace_once = instrument
    try:
        return native_delay.prepare_px4(args)
    finally:
        native_delay.replace_once = replace


def probe_px4(args, binary, capture, scenario):
    output = args.work / "px4.csv"
    completed = subprocess.run(
        [
            str(binary),
            "--input",
            str(capture),
            "--output",
            str(output),
            *(["--no-gps"] if scenario == "denied" else []),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    rows = np.array(
        [
            [float(value) for value in line.split()[1:]]
            for line in completed.stderr.splitlines()
            if line.startswith("AUDIT ")
        ]
    )
    estimate = native_delay.common_epochs(read(output), np.arange(1300, 6000) / 100)
    truth = read(args.capture / "truth.csv")
    result = dict(
        scenario=scenario,
        output_sha256=digest(output),
        rows=len(rows),
        flag_row_counts=dict(
            zip(
                (
                    "mag_3D",
                    "mag_heading",
                    "mag_aligned_in_flight",
                    "gps_height",
                    "flow",
                ),
                np.count_nonzero(rows[:, 1:6], axis=0).tolist(),
                strict=True,
            )
        ),
        prediction_dt_s=[float(rows[:, 6].min()), float(rows[:, 6].max())],
        last_fuse_timestamp_changes=dict(
            zip(
                ("mag", "flow", "gps_position", "gps_velocity", "barometer"),
                np.count_nonzero(np.diff(rows[:, 7:12], axis=0), axis=0).tolist(),
                strict=True,
            )
        ),
        rate_noise_std_range=[
            rows[:, 12:].min(axis=0).tolist(),
            rows[:, 12:].max(axis=0).tolist(),
        ],
        flight=metrics(estimate, truth, 13, 60),
        outage_window=metrics(estimate, truth, 25, 40),
        after_return=metrics(estimate, truth, 40, 60),
        replay_log="\n".join(
            line
            for line in completed.stderr.splitlines()
            if not line.startswith("AUDIT ")
        ),
    )
    if getattr(args, "sensor_informed_noise", False):
        observed = {
            line.split()[1]: float(line.split()[2])
            for line in completed.stderr.splitlines()
            if line.startswith("NATIVE_PARAM ")
        }
        expected = sensor_informed_noise()["px4"]
        if observed.keys() != expected.keys() or any(
            not np.isclose(observed[key], value, rtol=1e-6, atol=0)
            for key, value in expected.items()
        ):
            raise ValueError(
                "Actual PX4 noise parameters differ from the declared profile"
            )
        result["native_parameter_values"] = observed
    if not getattr(args, "retain_output", False):
        output.unlink()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "harness",
        "ap-replay",
        "ap-source",
        "transport-trace",
        "capture",
        "work",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--delay-profile", choices=PROFILES, default="nominal")
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=["gps", "denied", "transition"],
        choices=["gps", "denied", "transition"],
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--px4-source", type=Path)
    parser.add_argument("--px4-library", type=Path)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    args.takeoff_clear_s = None
    if args.output.exists() or args.work.exists():
        raise ValueError("Choose new output and owned work paths")
    pin = subprocess.check_output(
        ["git", "-C", str(args.ap_source), "rev-parse", "HEAD"], text=True
    ).strip()
    if pin != "1511f27194f1dcc3728270883047bdf022b3fd53":
        raise ValueError("Native source pin changed")
    adapters = {
        "ardupilot/csv_to_dataflash.py": "1c165fda77af2dd39e2a9c96685441abf65893503618c230e3f436f47750e7eb",
        "ardupilot/run_ekf3.py": "46bbfaf1ae961c45c4c9f3fa6d357fe3e238d67a8cf48001c3c8caadc8ad61a0",
    }
    if any(
        digest(args.harness / name) != expected for name, expected in adapters.items()
    ):
        raise ValueError("Use the audited 3dbaeb9 external adapters")
    args.work.mkdir(parents=True)
    capture = args.work / "capture"
    capture.mkdir()
    inputs = {}
    for name in (
        "imu.csv",
        "gps.csv",
        "flow.csv",
        "mag.csv",
        "baro.csv",
        "origin.json",
    ):
        inputs[name] = digest(args.capture / name)
        (capture / name).symlink_to((args.capture / name).resolve())
    evidence = dict(
        native_source_pin=pin,
        adapter_sha256=adapters,
        ap_replay_sha256=digest(args.ap_replay),
        transport_trace_sha256=digest(args.transport_trace),
        probe_sha256=digest(Path(__file__)),
        native_delay_sha256=digest(Path(native_delay.__file__)),
        input_sha256=inputs,
        capture_tag=args.capture.name,
        seed=args.seed,
        delay_profile=args.delay_profile,
        limitations=[
            "Configuration sensitivity probe, not a retuned estimator ranking",
            "Takeoff flag clears at declared 18 s; not the full Copter vehicle detector",
            "MAG_CAL=4 is a separate sensitivity experiment, not a required flight setting",
            "Logged innovations and selection rows do not count accepted measurement epochs",
        ],
        scores=[],
    )
    px4 = None
    if args.px4_source or args.px4_library:
        if not (args.px4_source and args.px4_library):
            raise ValueError("Supply both PX4 source and library")
        px4 = prepare_px4(args)
        evidence["px4"] = dict(
            binary_sha256=digest(px4),
            core_library_sha256=digest(args.px4_library),
            adapter_sha256=digest(args.work / "px4-arrivals.cpp"),
            scores=[],
        )
    for scenario in args.scenarios:
        trace = subprocess.run(
            [
                str(args.transport_trace),
                str(args.capture / "modelica_input.csv"),
                scenario,
                *map(str, PROFILES[args.delay_profile]),
                str(args.seed),
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        arrivals = np.atleast_1d(
            np.genfromtxt(io.StringIO(trace.stdout), names=True, delimiter=",")
        )
        (capture / "arrivals.csv").write_text(trace.stdout)
        if px4:
            print(scenario, "px4 diagnostics", flush=True)
            evidence["px4"]["scores"].append(probe_px4(args, px4, capture, scenario))
        for variant in ("baseline", "finite_takeoff", "finite_takeoff_mag4"):
            print(scenario, variant, flush=True)
            result = probe(args, capture, arrivals, scenario, variant)
            result["transport"] = json.loads(trace.stderr)
            result["arrival_trace_sha256"] = hashlib.sha256(
                trace.stdout.encode()
            ).hexdigest()
            evidence["scores"].append(result)
            args.output.write_text(json.dumps(evidence, indent=2) + "\n")


if __name__ == "__main__":
    main()
