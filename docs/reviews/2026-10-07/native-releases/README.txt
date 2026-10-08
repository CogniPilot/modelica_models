Stable native release comparison
================================

This follow-up rebuilds actual PX4 EKF2 and ArduPilot EKF3 cores at PX4 v1.17.0
and Copter-4.7.1. The parent review directory retains the historical native
pins and all previous results. The Modelica ports remain separate repositories
and implement only the kernels documented in ../release-port-refresh.txt.

build-manifest.json identifies the untouched upstream revisions, compiler,
native binaries, build recipe and logs. native-exposure.json records all 96
replays: 48 generated-C ESKF controls and 48 native release runs. It retains
both legacy flow reconstruction and explicit exposure packets. Its
frozen_controls verify delivery traces and compare output hashes with the
historical campaign. The ESKF controls must be byte-identical; native release
outputs may change. Failed or partial runs are retained without a complete
frozen_controls section and must not be treated as a completed comparison.

estimator-theory-comparison.txt/CSV/SVG/PNG combine the new native results with
the existing 96 final dense/root ESKF replays in the parent directory. This is
a two-seed synthetic comparison, with motion and seed coupled. Priors,
effective noise and stack source policies remain unequal. Full native NEES
and per-sensor NIS are still unavailable. The report makes no universal
superiority, complete firmware qualification or complete port-parity claim.

The parent files estimator-theory-contracts.txt, estimator-theory-lean-final.json,
eskf-hold-stationary.txt and eskf-theory-flight-replays.json provide the unchanged
formal assumptions, FOH/ZOH comparison and ESKF timing evidence. Timing in the
combined report is the earlier ESKF timing study, not a fresh native CPU test.

Reproduction
------------

Use an owned scratch directory for sources, builds, captures and logs:

  release_root="$HOME/scratch/modelica_models/native-release-comparison"
  export TMPDIR="$release_root/tmp"
  export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
  mkdir -p "$TMPDIR"

Check out PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4 and ArduPilot
dbe792162d06cab66c3475fd5556bf7a120f119e outside the main source checkout.
Initialize ArduPilot's recursive submodules. Use the separate original native
harness described in tools/estimator_comparison/README.txt. Copy that harness's
px4 directory into "$release_root/harness-px4" and append this CMake statement:

  target_sources(ekf2 PRIVATE ${EKF}/aid_sources/zero_innovation_heading_update.cpp)

This native release source belongs to the upstream core and is required by
v1.17.0. Configure the external harness with PX4_SOURCE_DIR pointing to that
release and CMAKE_BUILD_TYPE=Release. Build target ekf2, yielding libekf2.a;
the old harness's executable targets a newer development API. The comparison
tool builds release-compatible replay executables itself. Its API mapping
removes newer FusionControl and armed-status APIs absent from v1.17.0, while
retaining the release's actual parameter controls and vehicle-state setters.
It does not change native filter equations or invent an armed flag.

Build ArduPilot in its checked-out source directory:

  "$native_harness/ardupilot/apenv" ./waf configure --board sitl \
    --out="$release_root/build/ardupilot"
  "$native_harness/ardupilot/apenv" ./waf replay -j8

The CLI tools/estimator_comparison/compare_native_releases.py requires explicit
paths to the frozen captures, parent references, external harness, existing
horizon/retrodiction controls, transport executable, release sources, native
library/Replay, and new work/output paths. See --help for each option. It checks
the release revisions, tracked-source cleanliness, every frozen capture hash,
transport executable hash, packet traces and ESKF output controls. Historical
capture/build reproduction is documented in the main comparison README; the
original adapter repository is currently local and has no public remote.

To produce the combined report after a complete replay run:

  review=docs/reviews/2026-10-07
  python tools/estimator_comparison/report_estimator_theory.py \
    --native "$review/native-releases/native-exposure.json" \
    --flights "$review/eskf-theory-flight-replays.json" \
    --proofs "$review/estimator-theory-lean-final.json" \
    --output "$review/native-releases"

PX4's flight adapter passes integrated IMU samples directly to the native EKF.
The separate confirmed IntegratorConing indexing issue in the parent review
is consequently not exercised by these flight scores. EKF3 Replay uses the
default double-precision SITL build; float32 port-kernel parity is a separate
experiment. Neither exception is removed by rebuilding the latest releases.
