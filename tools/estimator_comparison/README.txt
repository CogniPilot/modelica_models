Estimator replay comparison
===========================

These tools generate independent analytic truth and noisy common captures,
replay actual generated Modelica plus external native estimator cores, and
score GPS-aided, GPS-denied and GPS-loss/recovery cases. This is a controlled
kinematic benchmark, not a flight qualification or a universal filter ranking.
The dated review in docs/reviews/2026-10-07 includes findings and all scores.

PX4 and ArduPilot are pinned validation submodules under upstream/; their
source files are not vendored into this repository. Native replay adapters
remain in the separately licensed estimator-comparison repository. Its
owned worktree is $HOME/scratch/estimator-comparison/matched-sensor-replay,
branch workspace/matched-sensor-replay, commit 3dbaeb9. That repository has no
remote. Modelica ports are maintained separately and do not replace native cores.
The native harness commit is durable in $HOME/git/estimator-comparison's Git
object database. Do not substitute the incomplete port for native EKF3.

Reproduction
------------

Run from the modelica_models root, using its Nix development environment
(Rumoca 0.10.2, Python with numpy/matplotlib, and a C compiler):

  export TMPDIR="$HOME/scratch/modelica_models/tmp"
  export XDG_CACHE_HOME="$HOME/scratch/modelica_models/cache"
  mkdir -p "$TMPDIR" "$XDG_CACHE_HOME"
  comparison_root="$HOME/scratch/modelica_models/estimator-comparison-reproduced"
  native_harness="$HOME/scratch/estimator-comparison/matched-sensor-replay/harness"
  python tools/estimator_comparison/build.py --output "$comparison_root/generated"

Initialize the validation submodules in a disposable clone on scratch, keeping
their Git object databases there too. The main checkout can leave them
uninitialized. Clone once; update the scratch checkout deliberately when
testing a different Modelica revision.

  validation_sources="$HOME/scratch/modelica_models/validation-sources"
  git clone --no-hardlinks . "$validation_sources"
  git -C "$validation_sources" submodule update --init -- \
    tools/estimator_comparison/upstream/px4 \
    tools/estimator_comparison/upstream/ardupilot
  px4_source="$validation_sources/tools/estimator_comparison/upstream/px4"
  ap_source="$validation_sources/tools/estimator_comparison/upstream/ardupilot"

The gitlinks pin PX4-Autopilot f1c0a1f794edf8e5e974b6ed96df3f95eda0df39 and
ArduPilot Copter 4.7.0 1511f27194f1dcc3728270883047bdf022b3fd53. Do not use
submodule update --remote for a reproducible comparison. Native source and
adapter licenses remain with their respective repositories; production
Modelica builds do not depend on these validation submodules.

Build the external cores using the separate adapters and the submodule sources:

  cmake -S "$native_harness/px4" -B "$comparison_root/native-px4" \
    -DPX4_SOURCE_DIR="$px4_source" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$comparison_root/native-px4" --parallel
  git -C "$ap_source" submodule update --init --recursive
  (
    cd "$ap_source"
    "$native_harness/ardupilot/apenv" ./waf configure --board sitl \
      --out="$comparison_root/native-ardupilot"
    "$native_harness/ardupilot/apenv" ./waf replay
  )

AP_REPLAY is supplied explicitly by run.py:

  px4_binary="$comparison_root/native-px4/ekf2_replay"
  ap_binary="$comparison_root/native-ardupilot/sitl/tool/Replay"
  python tools/estimator_comparison/run.py --output "$comparison_root" \
    --native-harness "$native_harness" --px4-replay "$px4_binary" \
    --ap-replay "$ap_binary" \
    --modelica-replay "$comparison_root/generated/modelica_replay" \
    --ukf-replay "$comparison_root/generated/ukf_replay"

Repeat with --lower-rates. Defaults are seeds 7 19 41 and horizontal trajectory
frequencies .25 .6 rad/s (the --speeds option names frequencies, not m/s).
All aiding streams remain available except the requested GPS outage. Captures
are deterministic; the lower profile downsampled the same sensor realizations.

  python tools/estimator_comparison/score.py "$comparison_root/results" \
    "$comparison_root/data" --output "$comparison_root/scores.json"
  python tools/estimator_comparison/score.py "$comparison_root/results-lower-rates" \
    "$comparison_root/data" --lower-rates --output "$comparison_root/scores-low.json"
  python tools/estimator_comparison/report.py \
    --scores "$comparison_root/scores.json" "$comparison_root/scores-low.json" \
    --data-root "$comparison_root/data" --results "$comparison_root/results" \
    --lower-results "$comparison_root/results-lower-rates" \
    --output "$comparison_root/report"

The report links to the dated source scores and paper review; copy reproduced
scores to estimator-scores.json / estimator-scores-lower-rates.json if making
a standalone report bundle. Raw CSVs, generated sources, native builds and
replay DataFlash logs are large and stay in scratch. The committed capture
manifest records SHA-256 hashes for the reviewed inputs, outputs and sources.
Different compiler/platform arithmetic can change output bytes; recompute
scores and compare metric tolerances rather than expecting byte identity.

Validation
----------

  python tools/estimator_comparison/test_score.py
  python tools/estimator_comparison/check_paper_series.py
  python tools/ci.py omc
  python tools/ci.py rumoca

The Rumoca checks now compile and execute a C99 UKF hover regression. Native
use requires cc (or MODELICA_MODELS_CC); the Nix application supplies its pinned
C compiler. Both raw and preintegrated prediction paths are exercised.

The scoring checks use known vector norms, wrapped heading, quaternion sign
invariance, invalid finite outputs, nonfinite failures and no extrapolation.
The exact rational series check is independent of Modelica and checks twelve
matrix examples; it is a sanity check, not a proof of the paper's theorem.
Tests.StrapdownPreintegrationJacobianTests checks all five physical bias blocks
through three composed FOH/ZOH steps. The previous implementation fails it.

Important interpretation
------------------------

The GPS-denied reference frame is the known startup origin. Flow + range +
magnetic heading can constrain local motion; no sensor in this suite supplies
an independent absolute horizontal position during denial. Native defaults
and Modelica RDD2 tuning differ. Only sensor captures and declared datums are
matched; this is an as-configured stack comparison, not matched Kalman gains.

The Modelica ESKF can accept at most one aiding source each tick. Continuous
100 Hz flow starves magnetic corrections; lowering aiding rates exposes that
scheduling effect. This comparison does not change the estimator dispatcher.

The released compiler hoisted one indexed UKF error-vector call out of the
mean reduction and repeated sigma point 2. Materializing the error before
accumulation fixes both predictors without patching Rumoca. The native C hover
regression in tools/ci.py catches this: the previous source translates 3.87 m
on the first hover prediction, while the corrected source preserves hover.
The corrected UKF was replayed across the full matrix. GPS, transition and
lower-rate denied cases sustain prediction; high-rate denied cases still reject
some predictions as covariance evolves. Those outputs remain scored.

The UKF's default initial bias variance is retained. A preliminary common-
covariance run with RDD2's 1e-6 gyro-bias variance failed immediately because
the float32 Cholesky threshold is about 1.79e-6. Do not rank unscented-filter
theory from a configuration that rejects prediction. prediction_accepted is
exported independently of the valid flag, which stays true during rejection.

The released Rumoca Python package needs a corrected Cargo dependency-fetch
hash in flake.nix. The compiler and binding source remain the exact v0.10.2
release. Whole-array coefficient assignments avoid the release's GALEC
indexed-storage replay recursion; explicit row/column matrix products avoid a
separate GALEC outerProduct rank limitation. The implementation uses shared
array/matrix expressions.
