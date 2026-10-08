"""Build an external GPL ArduPilot coning oracle from pinned native source."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

from native_release import SOURCE_PINS


DRIVER = """
#define AP_INLINE_VECTOR_OPS
#include <AP_Math/AP_Math.h>
#include <cstdio>
#include <initializer_list>

struct Samples {
  Vector3f _delta_angle_acc[1], _last_delta_angle[1], _last_raw_gyro[1];
  float _delta_angle_acc_dt[1]{};
} _imu;

static void integrate(const Vector3f &gyro, float dt) {
  const unsigned instance = 0;
  @COMPUTE@
  @ACCUMULATE@
}

int main() {
  const Vector3f initial_rate{0.3f, -0.2f, 0.1f};
  const Vector3f acceleration{2.f, 3.f, -1.f};
  for (float dt : {0.00125f, 0.0025f, 0.005f, 0.01f}) {
    for (int intervals : {2, 4, 8}) {
      _imu = Samples{};
      _imu._last_raw_gyro[0] = initial_rate - acceleration * dt;
      integrate(initial_rate, dt);
      _imu._delta_angle_acc[0].zero();
      _imu._delta_angle_acc_dt[0] = 0;
      for (int sample = 1; sample <= intervals; ++sample)
        integrate(initial_rate + acceleration * (sample * dt), dt);
      const Vector3f &angle = _imu._delta_angle_acc[0];
      std::printf("%.9g,%d,%.9g,%.9g,%.9g\\n", double(dt), intervals,
                  double(angle.x), double(angle.y), double(angle.z));
    }
  }
}
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def section(text, start, end):
    if text.count(start) != 1:
        raise ValueError(f"Native source anchor changed: {start}")
    first = text.index(start)
    return text[first : text.index(end, first)]


def build(args):
    repository = Path(__file__).resolve().parents[2]
    work = args.work.resolve()
    if work.is_relative_to(repository) or work.is_relative_to(args.upstream.resolve()):
        raise ValueError("Build the GPL oracle outside both source checkouts")
    if work.exists():
        raise ValueError("Choose a fresh external build directory")
    relative = "libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp"
    source = args.upstream / relative
    pinned = subprocess.check_output(
        [
            "git",
            "-C",
            str(args.upstream),
            "show",
            f"{SOURCE_PINS['ardupilot']}:{relative}",
        ]
    )
    if source.read_bytes() != pinned:
        raise ValueError("Use the pinned unmodified native inertial backend")
    text = section(
        pinned.decode(),
        "void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(",
        "void AP_InertialSensor_Backend::_notify_new_delta_angle(",
    )
    compute = section(
        text,
        "    Vector3f delta_angle = (gyro + _imu._last_raw_gyro[instance])",
        "    {\n        WITH_SEMAPHORE(_sem);",
    )
    accumulate = section(
        text,
        "        _imu._delta_angle_acc[instance] += delta_angle + delta_coning;",
        "        // apply gyro filters and sample for FFT",
    )
    database = json.loads(args.compile_commands.read_text())
    entry = next(row for row in database if row["file"].endswith(relative))
    original = entry["arguments"]
    arguments = [
        argument
        for argument in original
        if argument not in {entry["file"], "-c", "-MMD"}
        and not argument.startswith("-o")
    ]
    work.mkdir(parents=True)
    license_path = args.upstream / "COPYING.txt"
    shutil.copyfile(license_path, work / "COPYING.txt")
    driver = work / "native_coning.cpp"
    driver.write_text(
        "// SPDX-License-Identifier: GPL-3.0-or-later\n"
        "// Contains extracted ArduPilot code; see COPYING.txt and provenance.json.\n"
        + DRIVER.replace("@COMPUTE@", compute).replace("@ACCUMULATE@", accumulate)
    )
    executable = work / "native_coning"
    command = [*arguments, str(driver), "-o", str(executable)]
    provenance = dict(
        complete=False,
        license="GPL-3.0-or-later",
        upstream_revision=SOURCE_PINS["ardupilot"],
        native_source_path=relative,
        native_source_sha256=digest(source),
        driver_sha256=digest(driver),
        license_sha256=digest(work / "COPYING.txt"),
        compile_commands_sha256=digest(args.compile_commands),
        command=command,
        directory=entry["directory"],
        scope="Verbatim native coning arithmetic and native AP_Math vector types. Isolated arithmetic oracle, not the complete sensor backend or EKF3. No ESKF objects are linked.",
        excluded=["sensor filtering", "sample-gap reset", "semaphore", "logging"],
    )
    receipt = work / "provenance.json"
    receipt.write_text(json.dumps(provenance, indent=2) + "\n")
    with (work / "build.log").open("w") as log:
        subprocess.run(
            command,
            cwd=entry["directory"],
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    provenance.update(complete=True, executable_sha256=digest(executable))
    receipt.write_text(json.dumps(provenance, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, required=True)
    parser.add_argument("--compile-commands", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    build(parser.parse_args())
