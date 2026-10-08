# Repository tools

Run commands from the repository root. Modelica owns the runtime mathematics;
Python handles builds, replay orchestration, independent validation and offline
scoring. Keep generated outputs and caches under `$HOME/scratch`.

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
