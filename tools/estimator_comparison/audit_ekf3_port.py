#!/usr/bin/env python3
"""Run the external EKF3 kernel oracle and retain exact float32 parity evidence."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

from manifest import digest, hashes, upstream_revision
from native_delay import replace_once
from port_parity import compare


TRACE_HELPERS = r"""
#include <stdint.h>
#include <string.h>
static FILE *native_trace, *modelica_trace, *input_trace;
static void trace_pair(int operation, int number, const char *field,
                       float actual, float expected) {
  uint32_t actual_bits, expected_bits;
  memcpy(&actual_bits, &actual, sizeof(actual_bits));
  memcpy(&expected_bits, &expected, sizeof(expected_bits));
  fprintf(native_trace, "%d-%d,1,\"%s\",float32,%08x\n",
          operation, number, field, (unsigned)expected_bits);
  fprintf(modelica_trace, "%d-%d,1,\"%s\",float32,%08x\n",
          operation, number, field, (unsigned)actual_bits);
}
"""


def audit(args):
    repo = Path(__file__).resolve().parents[2]
    baseline_revision = upstream_revision(repo, "ardupilot")
    expected_revision = args.upstream_revision or baseline_revision
    if not re.fullmatch(r"[0-9a-f]{40}", expected_revision):
        raise ValueError("Declare a full immutable upstream commit")
    provenance = json.loads((args.port / "upstream.json").read_text())
    if provenance["revision"] != expected_revision:
        raise ValueError("Port and native comparison revisions differ")
    for path, expected in provenance["source_sha256"].items():
        source = subprocess.check_output(
            ["git", "-C", str(args.upstream), "show", f"{expected_revision}:{path}"]
        )
        if hashlib.sha256(source).hexdigest() != expected:
            raise ValueError(f"Unexpected upstream source bytes: {path}")
    if args.build.exists() or args.output.exists():
        raise ValueError("Choose new build and evidence paths")
    args.build.mkdir(parents=True)
    sys.path.insert(0, str(args.port / "tools"))
    spec = importlib.util.spec_from_file_location(
        "external_ekf3_validator", args.port / "tools/validate.py"
    )
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    source = validator.native_source(args.upstream)
    validator.validate_rumoca(args, args.build, source)
    driver = (args.port / "tests/rumoca_driver.c").read_text()
    driver = replace_once(
        driver,
        "int main(void) {",
        TRACE_HELPERS
        + r"""
int main(void) {
  native_trace = fopen("native.csv", "w");
  modelica_trace = fopen("modelica.csv", "w");
  input_trace = fopen("inputs.csv", "w");
  if (!native_trace || !modelica_trace || !input_trace) return EXIT_FAILURE;
  fputs("case,step,field,precision,bits\n", native_trace);
  fputs("case,step,field,precision,bits\n", modelica_trace);
""",
    )
    driver = replace_once(
        driver,
        "    DifferentialStepState state={0};",
        r"""    for (int i = 0; i < 656; ++i) {
      float value = (float)data[i];
      uint32_t bits;
      memcpy(&bits, &value, sizeof(bits));
      fprintf(input_trace, "%d-%d,%d,%08x\n", operation, number, i, (unsigned)bits);
    }
    DifferentialStepState state={0};""",
    )
    driver = replace_once(
        driver,
        "    worst=fmax(worst,current);cases++;",
        r"""
    for (int i = 0; i < 24; ++i) {
      char field[48];
      snprintf(field, sizeof(field), "state[%d]", i);
      trace_pair(operation, number, field, state.state[i], (float)data[656+i]);
      for (int j = 0; j < 24; ++j) {
        snprintf(field, sizeof(field), "P[%d,%d]", i, j);
        trace_pair(operation, number, field, state.P[i][j], (float)data[680+j*24+i]);
      }
    }
    trace_pair(operation, number, "clipCounter", state.clipCounter, (float)data[1256]);
    trace_pair(operation, number, "fused[0]", state.fused[0], (float)data[1257]);
    trace_pair(operation, number, "fused[1]", state.fused[1], (float)data[1258]);
    if (operation == 4)
      trace_pair(operation, number, "testRatio", state.testRatio, (float)data[1259]);
    worst=fmax(worst,current);cases++;""",
    )
    driver = replace_once(
        driver,
        "  return cases==160?",
        r"""
  if (fclose(native_trace) || fclose(modelica_trace) || fclose(input_trace)) return EXIT_FAILURE;
  return cases==160?""",
    )
    driver_path = args.build / "trace_driver.c"
    driver_path.write_text(driver)
    production = args.build / "rumoca/Ekf3_DifferentialStep/ProductionCode"
    command = [
        args.cc,
        "-std=c99",
        "-O2",
        "-pipe",
        "-I" + str(production),
        str(driver_path),
        str(production / "Ekf3_DifferentialStep.c"),
        str(production / "rumoca_galec_kernels.c"),
        str(args.build / "native_reference_f32.o"),
        "-lm",
        "-o",
        str(args.build / "trace_driver"),
    ]
    with (args.build / "trace-build.log").open("w") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    subprocess.run([str(args.build / "trace_driver")], cwd=args.build, check=True)
    result = dict(
        estimator="ArduPilot EKF3 Modelica port",
        upstream_revision=expected_revision,
        native_flight_baseline_revision=baseline_revision,
        cases=160,
        input_sha256=digest(args.build / "inputs.csv"),
        input_provenance="The same fixture data array supplies the generated Modelica block and extracted native C++ methods within each independent case. inputs.csv records all 656 input float32 values per case.",
        scope=["single-step prediction", "GPS horizontal position gate/correction"],
        **compare(args.build / "native.csv", args.build / "modelica.csv"),
        tolerant_validation=json.loads(
            (args.build / "rumoca-validation.json").read_text()
        ),
        port_source_sha256=hashes(
            args.port,
            [
                *args.port.glob("Ekf3/*.mo"),
                *args.port.glob("tests/*.c*"),
                *args.port.glob("tests/*.hpp"),
                *args.port.glob("tools/*.py"),
                args.port / "upstream.json",
            ],
        ),
        build_sha256=hashes(
            args.build,
            [
                driver_path,
                args.build / "trace_driver",
                args.build / "native_reference_f32.cpp",
                args.build / "native_facade_f32.hpp",
                args.build / "native_reference_f32.o",
                *production.glob("*.c"),
                *production.glob("*.h"),
            ],
        ),
        compile_command=command,
        full_flight_parity=False,
        limitations=[
            "Extracted upstream C++ methods in a test facade; not full native Replay.",
            "No complete optical-flow, magnetic, height, lifecycle or GPS-source-switching port.",
            "Independent single-step fixtures; no accumulated long-trajectory parity claim.",
            "Native oracle instantiated in float32 to match Rumoca; SITL may use double precision.",
        ],
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(
        json.dumps(
            {
                name: result[name]
                for name in (
                    "cases",
                    "components",
                    "bitwise_equal",
                    "bitwise_differences",
                    "first_difference",
                )
            }
        )
    )
    return 0 if result["bitwise_equal"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("port", "upstream", "build", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    for name in ("cc", "cxx", "rumoca"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument(
        "--upstream-revision",
        help="Explicit release commit; defaults to the historical native submodule pin",
    )
    arguments = parser.parse_args()
    for name in ("port", "upstream", "build", "output"):
        setattr(arguments, name, getattr(arguments, name).resolve())
    raise SystemExit(audit(arguments))
