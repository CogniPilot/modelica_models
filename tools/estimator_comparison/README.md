# Estimator replay tools

These tools build generated Modelica estimators, drive common sensor captures
through ESKF and native PX4/ArduPilot executables, and score the resulting
traces. See the Modelica HTML help in `Estimation.StrapdownINS` and `SLAM.Fusion`
for model selection and assumptions.

## Prerequisites and storage

Run from the repository root with Rumoca 0.10.2, a C compiler and Python with
NumPy. Plotting tools additionally require Matplotlib. Native validation needs
separately built upstream cores and the external replay adapters.

Keep generated code, caches, captures and binaries under `$HOME/scratch`.
Use ignored `dev/` for local development notes and reports. Neither directory
is an input bundled with the library; a comparison needing a frozen reference
must receive it explicitly.

```sh
export TMPDIR="$HOME/scratch/modelica_models/tmp"
mkdir -p "$TMPDIR"
comparison_root="$HOME/scratch/modelica_models/estimator-comparison"
```

## Build an ESKF replay

```sh
python tools/estimator_comparison/build.py --eskf-only --horizon \
  --output "$comparison_root/generated"
```

Add `--rumoca /path/to/rumoca` and `--cc /path/to/cc` if needed. This builds
`modelica_replay` for direct retrodiction and `horizon_replay` for delayed
fusion. Omitting `--eskf-only` also builds `ukf_replay`.

Use separate output directories for option combinations. `build.py --help`
lists optional magnetic, alignment, stationary-IMU, bias and square-root modes.
Those switches select experiments; they are not recommendations to enable
every option in a vehicle.

## Choose an experiment

Each script's `--help` lists its required captures, binaries, references and
output paths. Frozen comparison drivers reject mismatched source pins,
capture hashes or replay controls. Use fresh work and result paths.

| Task | Entry points |
| --- | --- |
| Generate and replay analytic captures | `run.py` |
| Evaluate an ESKF binary on captures | `evaluate.py` |
| Compare FOH, ZOH and mean-input integration | `ablate.py` |
| Compare stationary-IMU and ordinary prediction | `compare_hold.py`, `report_hold.py` |
| Compare horizon and retrodiction | `compare_delay.py` |
| Check native readiness and GPS loss/return | `compare_readiness.py`, `diagnose_native_readiness.py` |
| Observe native covariance with unchanged published states | `compare_readiness_covariance.py` |
| Join a declared campaign, retaining failures | `report_readiness_campaign.py` |
| Replay the stable native source pins | `compare_native_releases.py` |
| Check loose/tight visual corrections and the camera | `qualify_visual.py` |
| Compare sequential loose/tight visual navigation | `qualify_visual_navigation.py` |
| Recheck GNC Lean theorems and audit their axioms | `check_estimator_theory.py`, `check_bounded_error.py` |

The visual runners are self-contained synthetic experiments; see the
[validation guide](../../docs/validation.md#visual-aiding). Native campaign
drivers require external adapters and explicitly supplied reference data.
Some older drivers guard historical native revisions, so check their required
pins before building a native executable.

## Native sources

Stable-release replays use PX4 **v1.17.0** and ArduPilot **Copter-4.7.1**.
Their commit pins are in [`native_release.py`](native_release.py) and match
the validation submodules. Rumoca is pinned to **0.10.2** in `flake.nix`.

The validation submodules pin upstream source without making it a production
Modelica dependency. Initialize them in an owned scratch clone:

```sh
validation_sources="$HOME/scratch/modelica_models/validation-sources"
git clone --no-hardlinks . "$validation_sources"
git -C "$validation_sources" submodule update --init -- \
  tools/estimator_comparison/upstream/px4 \
  tools/estimator_comparison/upstream/ardupilot
```

Build with adapters matching the selected replay API and source revision.
Do not update submodules to a moving remote branch for a reproducible run.
Native code, generated native probes and their licenses stay outside ESKF
artifacts; see the [licensing boundary](../../docs/licensing.md).

## Interpret results

Compare identical physical captures and packet arrivals, with declared noise,
initialization, aiding and timing policies. Verify that each native filter
established GPS fusion before an outage; startup acquisition is a different
experiment from losing an established fix.

Score position, velocity, attitude and yaw RMS alongside covariance validity,
NEES and NIS. NEES requires a common tangent convention and matching state,
truth and covariance epochs. Native scalar NIS and ESKF joint NIS have different
dimensions and cannot be ranked by their raw values alone.

Keep failed and invalid cases in the declared comparison denominator. Report
missing or indefinite covariance explicitly. Time-correlated trajectory samples
are not independent Monte Carlo trials, and host CPU timings are not target
WCET. These synthetic experiments do not establish an all-scenario ranking.
