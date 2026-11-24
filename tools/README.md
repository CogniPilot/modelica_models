# Repository tools

Run commands from the repository root. Modelica owns the runtime mathematics;
Python handles builds, replay orchestration, independent validation and offline
scoring. Keep temporary builds, downloads and caches in a task-specific directory under
`/tmp`; preserve unique source and needed results before scratch cleanup.

| Task | Start here |
| --- | --- |
| Library structure, tests and vehicle qualification | `python -m tools.modelica_models_cli --help` |
| ESKF, PX4 EKF2 and ArduPilot EKF3 comparison | [Estimator replay tools](estimator_comparison/README.md) |
| Build the browsable Modelica reference | [Documentation publishing](../docs/publishing.md) |
| Export SLAM snapshots for a browser consumer | [SLAM export](../docs/slam-export.md) |

`ci.py` implements regression execution used by the CLI. `trajectory_compare.py`
supports mission comparisons. `modelica_models_cli/` owns command dispatch,
package checks and CI cache handling. `slam/` owns the compatibility exporter
and its source mapping.

The estimator comparison guide lists the maintained experiment entry points,
required native sources and result interpretation. Start there rather than
choosing a similarly named diagnostic script. Frozen studies require explicit
reference data; they do not depend on development notes being present.

## OpenModelica library

`nix run . -- test` supplies the checksum-pinned MSL 4.1.0 release to both
OpenModelica runners. Docker mounts it read-only at `/msl` for the current user;
no root-owned library cache is needed. The first script requires version 4.1.0
and simulates `Tests.MslRotationalSmoke` before the existing `Tests.All` suite.
With a native tool environment, set `MODELICA_MODELS_MSL` to the extracted release
root containing `Modelica 4.1.0`, `ModelicaServices 4.1.0`, and `Complex.mo`.
Missing, empty or unreadable required files fail before OpenModelica starts.
