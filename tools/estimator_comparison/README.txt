Estimator replay comparison
===========================

These tools generate independent analytic truth and noisy common captures,
replay actual generated Modelica plus external native estimator cores, and
score GPS-aided, GPS-denied and GPS-loss/recovery cases. This is a controlled
kinematic benchmark, not a flight qualification or a universal filter ranking.
The dated review in docs/reviews/2026-10-07 includes findings and all scores.

Read docs/reviews/2026-10-08/native-innovations before using the stable-release
GPS/transition rankings. Actual scalar correction observers reproduce all 24
native state outputs byte for byte, but EKF3 first fuses GPS at 35.6-43.912 s
in its GPS cases. None of its four transition cases fuses GPS before the
25 s outage. Those cases measure startup followed by acquisition, not an
established-GPS loss/recovery transition. The 13 s preflight does not satisfy
EKF3's gyro-bias covariance readiness condition. A common readiness-qualified
warmup and revised outage timing are required before a final ranking. The new
report includes actual scalar NIS and measurement variances with explicit
selection and sensor-coverage limitations; it does not claim joint vector NIS.

The common-warmup pilot in docs/reviews/2026-10-08/readiness-benchmark uses
120 s of stationary data, takeoff at 120 s, GPS loss at 132 s and return at
147 s. Both native filters establish GPS before takeoff, stop GPS corrections
during the outage, and resume after return. compare_readiness.py shifts capture,
transport, ESKF rest declaration, native arm/takeoff hints and scoring together.
compare_readiness_covariance.py requires matching native published-state bytes
before joining full-covariance NEES. These drivers use the externally instrumented
native validation cores; no native source is added here.

The completed eight-capture campaign is in
docs/reviews/2026-10-08/matched-campaign. report_readiness_campaign.py binds its
declaration, capture manifest, completed results and frozen source snapshot;
it retains invalid covariance/readiness conditions and failed captures in the
paired denominator. Its CSVs cover all RMS components, common 15D NEES,
native per-sensor scalar NIS, GPS recovery and instrumented ESKF timing.
plot_readiness_campaign.py exports all horizontal-position pairs as PNG/PDF/SVG.
replay_px4_magnetics.py observes public magnetic-state getters through a separate
adapter and requires frozen published-state and innovation byte parity.
The per-sensor ESKF NIS audit is in docs/reviews/2026-10-08/eskf-innovations.
instrument_eskf_innovations.py observes an owned generated-C copy;
replay_eskf_innovations.py requires frozen state and complete covariance byte
parity; eskf_innovations.py reconstructs joint NIS from the actual residual/S.
report_eskf_innovations.py requires all 96 ESKF cases of the matched campaign.
Repeated identical pure-function evaluations are audited and counted once.
A complete match of native effective Q/R remains open.

PX4 innovation observer times include a 1 s clock epoch offset, as already
accounted for by native_consistency.py. native_innovations.py and
diagnose_native_readiness.py now remove it too. The older frozen innovation
report used native-clock windows and reported first GPS at 2.12625 s; physical
capture time is 1.12625 s. The new pilot uses physical time throughout.

Generate a common warmup with generate.py CAPTURE --warmup-s 120, followed by
flow_exposure.py --source CAPTURE --output EXPOSURE. The unchanged default is
13 s. transport_trace accepts an optional final mission-offset argument;
replay.c accepts --mission-offset SECONDS. For 120 s warmup the offset is 107 s.
All large captures, native observer builds and replay outputs belong under
$HOME/scratch; durable summaries belong in the dated review directory.

The common-sensor-noise follow-up adds generate.py --common-native-floors.
It uses 1 microtesla magnetic noise, 0.05 rad/s compensated flow noise and
0.05/0.05/0.075 m/s GPS velocity noise. The camera simulation derives image
motion from named body-velocity/attitude fields; its independent test oracle
uses world velocity and quaternion geometry. The capture profile selects
the declared native noise settings and ESKF --measurement-noise inputs;
ordinary captures retain the original byte-identical behavior.

compare_readiness.py records readiness failures explicitly and finishes the
declared scenarios rather than presenting a failed case as qualified. Its
--scenarios option supports bounded diagnostics. The covariance companion
records non-positive-definite matrices without repair; --allow-incomplete
permits diagnosis of available native rows in an interrupted campaign but
does not declare that comparison complete. Empty, duplicated or unknown
conditions are rejected. diagnose_native_covariance.py accepts physical
--start-s/--end-s windows. Read the dated common-sensor-noise review before
using the experimental configuration; two EKF3 covariance cases fail.

The follow-up in docs/reviews/2026-10-08/ekf3-covariance-stages isolates their
first invalid covariance to a native constrained-gain GPS velocity correction.
instrument_ekf3_covariance_stages.py adds read-only operation snapshots to an
owned clean pinned source copy. replay_ekf3_covariance_stages.py requires frozen
capture, noise/arrival, published-state and full-covariance logging parity;
diagnose_ekf3_covariance_stages.py reconstructs scalar updates and labels offline
Joseph counterfactuals separately. The durable raw witness can be checked
without external native sources. The native bad-IMU trigger still needs tracing.

The subsequent ekf3-imu-integrity review records pre-override residuals and
isolates the early one-sigma trigger in stationary preflight. The read-only
instrument_ekf3_imu_integrity.py hooks share the verified replay driver through
--imu-integrity, without stage time bounds. diagnose_ekf3_imu_integrity.py checks
every timestamp, threshold, flag transition and native velocity override.
generate.py --gps-fix-after-s 21 changes only initial GPS availability for all
filters; all physical draws and default capture bytes are preserved. The
controlled pilot passes readiness and all six native covariance checks; a
declared eight-capture campaign now tests independent seeds, motions and heights.

Read docs/reviews/2026-10-07/native-configuration-audit.txt before interpreting
the native rankings. A later 24-replay audit reproduced the previous native
outputs but found a persistent ArduPilot takeoff flag, different active magnetic
fusion modes, unequal internal sensor rates and unresolved sensor averaging/time
semantics. Earlier scores remain historical configured-stack measurements;
they do not establish PX4 superiority over ArduPilot or ESKF superiority over
equivalently calibrated native filters. The audit includes reproducible probes.

The subsequent eskf-altitude.html / eskf-altitude.txt comparison retains the
2 m climb and adds a 4 m climb across all six seeds and GPS scenarios. EKF3
switches naturally into three-axis magnetic fusion at 4 m. Its takeoff hint now
clears at 18 s in native_delay.py; --persistent-takeoff reproduces the historical
adapter lifecycle, and --takeoff-clear-s controls the declared timeout. This
is a bounded vehicle-phase approximation, not Copter's complete detector.

PX4 and ArduPilot are pinned validation submodules under upstream/; their
source files are not vendored into this repository. Native replay adapters
remain in the separately licensed estimator-comparison repository. Its
owned worktree is $HOME/scratch/estimator-comparison/matched-sensor-replay,
branch workspace/matched-sensor-replay, commit 3dbaeb9. That repository has no
remote. Modelica ports are maintained separately and do not replace native cores.
The native harness commit is durable in $HOME/git/estimator-comparison's Git
object database. Do not substitute the incomplete port for native EKF3.

Native versus Modelica port parity
---------------------------------

The flight comparisons use generated Modelica ESKF and the actual native PX4
and ArduPilot cores. They do not run the partial PX4/ArduPilot Modelica ports.
Prediction/GPS kernel agreement cannot qualify a missing optical-flow, magnetic,
height, delayed-fusion or source-switching implementation. Keep port fidelity
and flight-performance results separate until those paths are implemented.

See docs/reviews/2026-10-07/estimator-port-fidelity.txt for the initial strict parity
audit. At that audit, the local PX4 port targeted bd62df5; the native flight baseline
targets f1c0a1f. Align the revisions before claiming exact parity. ArduPilot's
port and native source target 1511f27, but the float32 Modelica export differs
in precision from the default double-precision SITL build.

audit_ekf3_port.py consumes the separately licensed external port and pinned
upstream Git objects; no native filter algorithm or port is copied here. Run
with --port, --upstream, --build, --output, --cc, --cxx and --rumoca. Build and
output paths must be new. --upstream-revision FULL_SHA selects an explicit
release instead of the historical native submodule pin. Both revisions are
recorded separately; selecting a release does not update the flight baseline.
It freshly exports with Rumoca 0.10.2, runs the
external 160-case float32 validator, and then records every state/covariance
component, clipping counter, fusion decision and applicable gate ratio as raw
hexadecimal bits. An exact mismatch writes evidence and exits with status 1.

The first run found 25 mismatches in the external test facade: vector division
multiplied by a reciprocal, unlike upstream's componentwise division. Correcting
that facade in an owned external copy, with all Modelica files unchanged,
produced 96,560/96,560 exact matches. The canonical external repository has not
been modified. Its corrected copy is disposable; the dated evidence records
source hashes and the change needed for reproduction. These remain independent
kernel cases, not a native firmware or complete flight parity claim.

The follow-up release-port-refresh.txt/.json in the same review directory
records updates in separate owned port repositories to PX4 v1.17.0 (d6f12ad1)
and ArduPilot Copter-4.7.1 (dbe79216), verified as latest stable on October 7,
2026. PX4 covariance/quaternion kernels pass 191,040 exact float32 comparisons;
the refreshed ArduPilot prediction/GPS kernels pass 96,560. These release port
targets do not change the older native flight binaries or historical scores.
The report identifies the signed-off port commits, source hashes and replay
limitations. No PX4 or ArduPilot port source is stored in this repository.

port_parity.py also checks arbitrary externally produced paired traces:

  python tools/estimator_comparison/port_parity.py \
    --native native.csv --modelica modelica.csv --output parity.json \
    --native-input-sha256 "$capture_hash" \
    --modelica-input-sha256 "$capture_hash" \
    --native-revision "$upstream_revision" \
    --port-upstream-revision "$upstream_revision" \
    --scope 'prediction and GPS kernels'

Both traces require columns case,step,field,precision,bits in identical order.
Use float32/eight hex digits or float64/sixteen hex digits. Hex values describe
the IEEE bit pattern, independent of host byte order; quote fields containing
commas. The checker rejects different declared input hashes/revisions, missing,
duplicate or reordered components, precision changes, malformed values and
nonfinite outputs. Signed zeros count as different bits. It reports the first
difference, affected field groups, maximum absolute/scaled error and ULP
distance. It cannot independently attest an external producer's declarations;
retain adapter, toolchain and input provenance alongside the traces.

The requested common-noise RMS/NEES/NIS flight comparison remains a separate
requirement. Existing scores have unmatched effective measurement noise and
native source policies. Full NEES requires the full covariance, a common error
definition, and state/truth at the covariance's fusion epoch. ArduPilot's
quantized innovation logs and diagonal covariance logs cannot supply exact NIS
or joint NEES. Do not substitute gate-test ratios or diagonal approximations.

Formal contracts and covariance safety
-------------------------------------

See docs/reviews/2026-10-07/estimator-theory-comparison.txt for the detailed
GPS, GPS-denied and loss/return comparison, and estimator-theory-contracts.txt
for the implementation comparison and precise Lean theorem assumptions.
estimator-theory-comparison.csv retains every method, mapping and scoring window.
The SVG shows medians and observed ranges, not confidence intervals.

The new dense correction guard rejects an impossible joint state/measurement
covariance even if its innovation is positive definite. It uses a triangular
factor solve to avoid rejecting valid mixed-unit GPS covariance. The existing
square-root path already checks conditional measurement noise. CI now challenges
both generated correction libraries with invalid, valid and ill-scaled cases:

  python tools/estimator_comparison/check_correction_covariance.py \
    --dense "$comparison_build/eskf-correction-kernel.so" \
    --root "$comparison_build/square-root-correction-kernel.so" \
    --output "$comparison_build/correction-joint-covariance.json"

The raw-IMU covariance path also now computes its transition before propagation.
The earlier representation refactor left that local matrix unassigned; packet
flight comparisons did not exercise the fallback. The retained packet/sequential
regression exposed a 0.04 covariance discrepancy. CI's new RawPredictionReplay
probe checks zero-noise inertial sensitivities, zero-time prior preservation and
correlated priors in dense/root modes using the actual float32 deployment export:

  python tools/estimator_comparison/check_raw_prediction.py \
    --library "$comparison_build/raw-prediction-kernel.so" \
    --output "$comparison_build/raw-prediction.json"

Tests.All passes locally with OpenModelica and cflags=-O0. The default -Os
aggregate build was stopped during a long compilation; this is not a completed
default-CI pass. Production C probes and the flight drivers use -O2.

GNC's new correlated Joseph, optimal-gain gap, PSD factor and horizontal
translation observability proofs are prepared in a separate GNC commit.
gnc-estimation-covariance.patch preserves it for review/application to an owned
checkout based on the recorded GNC revision. The canonical GNC checkout and
its uncommitted work were preserved. The commit has Gmail authorship and DCO.

With that proof checkout and its pinned Lean 4.29.1 dependency cache available:

  python tools/estimator_comparison/check_estimator_theory.py \
    --gnc "$gnc_proof_checkout" --cache "$gnc_dependency_cache" \
    --lean "$HOME/.elan/toolchains/leanprover--lean4---v4.29.1/bin/lean" \
    --build "$comparison_root/proofs" \
    --output "$comparison_root/proof-check.json"

Build and evidence paths must be new. This recompiles ten selected modules,
then runs GNC's checked-environment axiom audit; it reuses checked transitive
dependencies. It is not the full GNC release verification or a formal proof
of Modelica/C refinement. Equally modeled optimal Kalman updates can tie;
Lie-group coordinates alone cannot establish universal ESKF superiority.

Reproduce the report from the complete saved evidence:

  review=docs/reviews/2026-10-07
  python tools/estimator_comparison/report_estimator_theory.py \
    --native "$review/native-exposure.json" \
    --flights "$review/eskf-theory-flight-replays.json" \
    --proofs "$review/estimator-theory-lean-final.json" \
    --output "$comparison_root/report"

Stable native releases
----------------------

The follow-up review in docs/reviews/2026-10-07/native-releases rebuilds actual
native cores at PX4 v1.17.0 and ArduPilot Copter-4.7.1, using frozen physical
captures and packet delivery traces. compare_native_releases.py checks source
pins, capture hashes and ESKF byte-identical controls. The report records these
release results separately from the historical scores in the parent directory.
Its README describes the release-specific external build/API adaptation.
The public port and formal-proof pull requests are recorded in publication.json
there. The partial ports are not used as native flight baselines.

Historical reproduction
-----------------------

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

For the historical commands below, deliberately select the old native pins
after initialization. Current gitlinks track the separately validated stable
release campaign; historical scripts retain their original revision guards.

  git -C "$px4_source" checkout f1c0a1f794edf8e5e974b6ed96df3f95eda0df39
  git -C "$ap_source" checkout 1511f27194f1dcc3728270883047bdf022b3fd53

Those are the historical PX4 revision and ArduPilot Copter 4.7.0. Do not use
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

ESKF improvement experiments
---------------------------

The geometric candidates and their regressions are documented in
docs/reviews/2026-10-07/eskf-geometry.txt and eskf-geometry.html. Both
useEquivariantMagnetometer and useGeometricAlignment remain false by default.
They are observations/initialization changes inside the current ESKF, not a
complete bias-aware equivariant filter. Vector fusion requires a calibrated
local field model and regresses some held-out denied cases.

Use separate build directories for each option combination:

  python tools/estimator_comparison/build.py \
    --output "$comparison_root/geometric-alignment" --geometric-alignment
  python tools/estimator_comparison/build.py \
    --output "$comparison_root/geometric-vector" \
    --geometric-alignment --equivariant-magnetometer

The CSV replay accepts - for its output filename to stream estimates to stdout
and - for its optional diagnostic filename to stream covariance to stderr.
evaluate.py uses these streams to keep raw replay outputs in memory:

  python tools/estimator_comparison/evaluate.py \
    --replay "$comparison_root/geometric-alignment/modelica_replay" \
    --captures "$comparison_root/data" --output "$comparison_root/alignment.json"

Repeat with --frequencies .45 .85 --seeds 101 307 911 on held-out captures.
Add --diagnostics --gyro-bias .0008 -.0005 .0004 --accel-bias .02 -.01 .015
for full covariance scoring against these captures' declared synthetic biases.
The scorer checks finite outputs, status, exact row delivery and timestamps.
It reports correlated-sample NEES descriptively, not as independent Monte Carlo.
Output JSON filenames must be new to preserve previous evidence.

The replay accepts an optional integration mode after the GPS scenario:
foh (default), zoh (800 Hz endpoint samples), or mean (one exact held-input
preintegral from the mean of eight samples). Sensor packets, timestamps,
noise settings and filter rate remain identical. An optional final filename
records IMU bias estimates and the full local error covariance.

  python tools/estimator_comparison/ablate.py \
    --replay "$comparison_root/generated/modelica_replay" \
    --captures "$comparison_root/data" --output "$comparison_root/ablation"

Use --integrations foh --diagnostics for covariance traces. The output directory
must be new. Default single-worker timing is elapsed replay time; parallel
workers are useful for accuracy checks but their elapsed times are not CPU
benchmarks. The dated eskf-ablation-scores.json instead records child CPU time
from serial runs of the final implementation.

For held-out runs, use run.py with --speeds 0.45 0.85 --seeds 101 307 911 and a
new --output directory, then repeat with --lower-rates. Do not regenerate the
original captures in place. The dated eskf-held-out-scores.json includes all
three native/generated estimators; eskf-held-out-before-scores.json replays
the previous ESKF on those same held-out captures.

Build the optional Monte Carlo bridge from the actual generated preintegrator
and the covariance function used by prediction:

  noise_export="$comparison_root/noise"
  rumoca compile Tests/PreintegrationNoiseReplay.mo \
    --model Tests.PreintegrationNoiseReplay --source-root "$PWD" \
    --target galec-production --output "$noise_export"
  preintegrator="$comparison_root/generated/Tests_PreintegrationReplay/Tests_PreintegrationReplay/ProductionCode"
  noise_code="$noise_export/Tests_PreintegrationNoiseReplay/ProductionCode"
  cc -O2 -shared -fPIC -I"$preintegrator" -I"$noise_code" \
    tools/estimator_comparison/preintegration_noise.c \
    "$preintegrator/Tests_PreintegrationReplay.c" \
    "$noise_code/Tests_PreintegrationNoiseReplay.c" \
    "$noise_code/rumoca_galec_kernels.c" -lm \
    -o "$comparison_root/preintegration-noise.so"
  python tools/estimator_comparison/check_preintegration_noise.py \
    --library "$comparison_root/preintegration-noise.so" \
    --output "$comparison_root/preintegration-noise.json"

This checks hover, constant turning and linearly varying angular rate with
20,000 trials per motion and integration mode. It tests packet-level covariance
and shared-endpoint correlation; it does not establish full-filter consistency
under arbitrary motion, bias random walks or delayed sensor fusion.

For descriptive full-filter NEES, run consistency.py with --estimate,
--covariance, --truth, --output, and explicit --gyro-bias / --accel-bias vectors.
The current analytic capture uses .0008 -.0005 .0004 rad/s and .02 -.01 .015
m/s2. These are truth for scoring only, never initialization inputs. Flight
samples are correlated and cannot be treated as independent Monte Carlo draws.

The dated eskf-improvements.txt reports intermediate changes, final metrics,
held-out regressions and remaining native-adapter limitations. Preserve the
original estimator-scores*.json and capture-manifest.json as historical evidence.

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

The historical native-comparison campaign used the earlier single-source
ESKF dispatcher. The 2026-10-07 review adds independent flow, height and
magnetic corrections alongside the primary GPS/mocap correction. Use the dated
improvement and geometric reports when assessing the current implementation.

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


Buffered horizon versus retrodiction
-----------------------------------

Build both drivers from the same current ESKF with Rumoca 0.10.2:

  comparison_root="$HOME/scratch/modelica_models/estimator-comparison"
  python tools/estimator_comparison/build.py \
    --output "$comparison_root/delayed-build" --eskf-only --horizon
  python tools/estimator_comparison/compare_delay.py \
    --retrodiction "$comparison_root/delayed-build/modelica_replay" \
    --horizon "$comparison_root/delayed-build/horizon_replay" \
    --captures "$comparison_root/data" \
    --frequencies 0.6 --seeds 7 19 41 --repetitions 3 \
    --output "$comparison_root/delayed-original.json"

Repeat with the held-out capture directory, frequency .85 and seeds
101/307/911. report_delay.py accepts --original, --held-out and --output to
write an interactive HTML report and a sibling per-seed CSV.

Both drivers receive identical timestamped packets and seeded transport
jitter. Delay profiles are zero, nominal and stress; see compare_delay.py.
GPS denial is applied at measurement time, before transport. The horizon
filters approximately 200 ms behind the present and predicts forward with
buffered SE_2(3) increments. Current-state and fusion-state scores use their
respective timestamps. The generated queue is called on aiding arrival or
fusion release; the composed Modelica block still calls it every IMU tick.

The replay schedules generated components explicitly because the composed
HorizonEstimator export reaches Rumoca's recorded sampled-read diagnostic.
Host-only fold and correction budgets permit the complete numerical campaign;
they do not qualify flight scheduling. Reported CPU covers generated numerical
components, excluding CSV, simulator and adapter copies. Instance sizes include
generated scratch; the transport simulator is separate. Horizon covariance
scoring is not implemented. This first delay stage compares the two ESKF modes;
the subsequent native-delay review adds native EKF2/EKF3 on the same arrival
schedules.
See docs/reviews/2026-10-07/eskf-delay.html for evidence and limitations.


Declared stationary startup
---------------------------

The ESKF now accepts vehicleAtRest, default false. In Modelica, supply it as
an instance modifier, for example Estimator(vehicleAtRest=landedAndStationary).
This is a declaration of physical rest, not an inference from a quiet IMU or
from being disarmed. The independent zero-velocity constraint uses
stationaryVelocityVariance_m2_s2=0.01 and runs alongside live sensor aiding.
Non-positive variance disables it. Its acceptance contributes once per fusion
instant to the output-predictor correction notification and preserves primary
anchor rejection health. Existing unaided pseudo measurements are separate.

Pass --stationary-until 13 to compare_delay.py or directly to either replay
binary to supply the declared stationary phase used by the native adapters.
The horizon applies this phase at its IMU/fusion timestamp, rather than the
present output time. This input is supported by the ESKF component and the
explicit component scheduler; the composed HorizonEstimator does not expose a
stationary-phase queue and remains blocked by Rumoca 0.10.2.

The fixed-data campaign covers all three scenarios, two sensor rates, three
arrival profiles, and both three-seed motion sets. Builds with
--geometric-alignment and with both --geometric-alignment and
--equivariant-magnetometer retain the experimental geometry ablations.
The options remain disabled by default because their gains are mixed.

Covariance checks can use evaluate.py --scenarios denied --diagnostics
--delay-profile nominal --stationary-until 13 with the capture's declared
--gyro-bias and --accel-bias for scoring only. Horizon covariance remains
unavailable; its delayed filter covariance must not be attributed to the
present predicted output. See docs/reviews/2026-10-07/eskf-stationary.txt and
the linked interactive reports for results, provenance and limitations.


Experimental semi-direct bias corrections
-----------------------------------------

The ESKF parameter useSemiDirectBias defaults false. A build with
--semi-direct-bias enables the SE_2(3) semidirect se(3) local retraction and
full 15-state covariance reset for all correction paths. Physical IMU
propagation and process noise retain their existing first-order local maps.
This is a correction-geometry experiment, not the 18-state tangent-group
filter or a claim of exact nonlinear bias-error propagation.

Build in a separate directory and repeat the declared-rest comparison:

  python tools/estimator_comparison/build.py \
    --output "$comparison_root/semidirect" --eskf-only --horizon \
    --semi-direct-bias

Optional --geometric-alignment and --equivariant-magnetometer retain the
same ablations as the stationary comparison. The fixed-data results are
small and mixed, so none of these geometry options is enabled by default.
See docs/reviews/2026-10-07/eskf-semidirect.txt and its interactive reports
for the complete comparison with the unchanged native EKF2/EKF3 results.

The Rumoca CI check exports Tests.SemiDirectBiasCorrectionReplay and compares
actual float32 C with an independent float64 Gaussian correction and reset.
Both geometries, heading/axis constraints, correlated measurement noise,
attitude trust limits and exact rejected-state preservation are exercised.
Run check_semidirect_correction.py --library LIBRARY --output NEW_JSON to
repeat the independent check on a separately compiled probe. The earlier
eskf-semidirect-bias-design.txt describes the helper-only stage before this
integration; its source snapshot and claims remain historical.


White IMU noise calibration experiment
--------------------------------------

RDD2's flight-log process noise is not a noise model for every sensor capture.
The ESKF ProcessNoise fields are continuous spectral-density matrices. Native
EKF2/EKF3 prediction parameters are angular-rate/acceleration standard deviations
and their source multiplies their variances by prediction interval squared.
Copying the same numbers between these interfaces does not match uncertainty.

calibrate_imu_noise.py estimates isotropic white-noise densities from an
explicitly declared stationary window, using adjacent differences to remove
constant sensor bias. It does not infer rest, estimate bias drift, characterize
colored noise or recommend flight tuning:

  python tools/estimator_comparison/calibrate_imu_noise.py \
    --imu "$comparison_root/data/0.6_7/imu.csv" --stationary-window 1 5 \
    --output "$comparison_root/imu-noise.json"

Its printed --imu-noise-density GYRO ACCEL arguments work with the replay
binary, compare_delay.py and evaluate.py. The units are rad/sqrt(s) and
m/(s sqrt(s)); the replay squares them into the ESKF PSD diagonal. No option
retains the exported deployment's tuning. Bias random walks and initial priors
are unchanged. Score files record the supplied densities. Freeze a calibration
from original data before evaluating held-out captures.

The eskf-noise review applies one original seed's 1--5 s calibration to the
entire original/held-out matrix and retains heading, alignment and magnetic-
vector ablations. The native results remain the earlier configured native
stacks, not filters retuned to these densities. See
docs/reviews/2026-10-07/eskf-noise.txt and eskf-noise-native-units.json for
findings, the executed native covariance-helper unit check and limitations.

native_noise_units.cpp is an independent test wrapper that includes the pinned
PX4 generated covariance header through its validation checkout. It contains
no copied native filter implementation. Build it with C++17 and the external
harness's px4/stubs, PX4 src/lib/matrix, and PX4 src/modules/ekf2/EKF/python
include directories. It verifies angle/velocity noise scaling at 5/10/12.5/20 ms
and prints dt, angle variance and velocity variance as CSV. With two optional
gyro/accelerometer density arguments, it converts to native rate noise and
checks density squared times dt instead of the default rate-noise scaling.
EKF3's formula
is audited separately in its submodule source, not exercised by this wrapper.

Altitude and magnetic activation experiment
-------------------------------------------

generate.py accepts --climb-height-m (default 2). compare_altitude.py reads the
original captures, generates 4 m captures with matching noise draws, reuses the
frozen calibration and checks unchanged low-altitude outputs against earlier
scores. It runs both ESKF modes plus native cores, with magnetic mode unforced.
Use separate new --work/--output paths for each motion set. For the original set:

  python tools/estimator_comparison/build.py \
    --output "$comparison_root/vector" --eskf-only --horizon \
    --geometric-alignment --equivariant-magnetometer
  python tools/estimator_comparison/compare_altitude.py \
    --captures "$HOME/scratch/modelica_models/estimator-comparison/data" \
    --frequency .6 --seeds 7 19 41 \
    --harness "$native_harness" --ap-source "$ap_source" \
    --ap-replay "$ap_binary" --px4-source "$px4_source" \
    --px4-library "$comparison_root/native-px4/libekf2.a" \
    --transport-trace "$comparison_root/transport_trace" \
    --retrodiction "$comparison_root/vector/modelica_replay" \
    --horizon "$comparison_root/vector/horizon_replay" \
    --calibration docs/reviews/2026-10-07/eskf-noise-calibration.json \
    --native-reference docs/reviews/2026-10-07/native-delay-original.json \
    --eskf-reference docs/reviews/2026-10-07/eskf-noise-vector-original.json \
    --audit-reference docs/reviews/2026-10-07/native-configuration-audit-original.json \
    --work "$comparison_root/altitude-original" \
    --output "$comparison_root/altitude-original.json"

Repeat with held-out captures in
$HOME/scratch/modelica_models/eskf-improvements/held-out/data, frequency .85,
seeds 101 307 911, and the three held-out reference files. The strict output-hash
controls require the recorded compiler/build arithmetic; a rebuild that changes
bytes needs a separately reviewed numeric compatibility baseline, not a bypass.
report_altitude.py --original FILE --held-out FILE --output PREFIX renders the
HTML/CSV/text; --covariance FILE includes the separate retrodiction diagnostics.
The altitude review also retains a large seed-41 covariance overconfidence case.

Matched white-IMU-noise experiment
----------------------------------

compare_native_noise.py applies the existing frozen stationary calibration to
both native cores, retaining their default configurations as controls. Use the
altitude experiment's original captures, higher captures and reference JSON,
the same native binaries/adapters, and new owned work/output paths. For example:

  python tools/estimator_comparison/compare_native_noise.py \
    --captures "$HOME/scratch/modelica_models/estimator-comparison/data" \
    --higher-captures "$comparison_root/altitude-original" \
    --reference docs/reviews/2026-10-07/eskf-altitude-original.json \
    --calibration docs/reviews/2026-10-07/eskf-noise-calibration.json \
    --native-reference docs/reviews/2026-10-07/native-delay-original.json \
    --harness "$native_harness" --ap-source "$ap_source" \
    --ap-replay "$ap_binary" --px4-source "$px4_source" \
    --px4-library "$comparison_root/native-px4/libekf2.a" \
    --transport-trace "$comparison_root/transport_trace" \
    --work "$comparison_root/native-noise-original" \
    --output "$comparison_root/native-noise-original.json"

Repeat with held-out paths and references. report_native_noise.py takes
--original-reference, --held-out-reference, --original, --held-out and --output
to produce the combined text/CSV report. It refuses missing/duplicate cases,
changed default controls, reference/core/transport mismatches, inconsistent
noise mappings and nonfinite metrics. Native rate noise equals the density
divided by sqrt(prediction period), using the observed settled 10 ms PX4 and
12.5 ms EKF3 periods. Runtime parameters and flight timing are checked.

This matches the leading white-noise variance only. Bias random walks, priors,
measurement noise floors, sensor averaging/delay semantics and ingestion rates
remain different. EKF3's mapped accelerometer value is below its documented
recommended minimum but inside the actual runtime clamp; this is a synthetic
ablation, not flight tuning. See docs/reviews/2026-10-07/native-noise.txt.

The current relationship to the ACC manuscript, all five composed physical
bias Jacobians, first-order bias correction and covariance/certificate limits
is recorded in docs/reviews/2026-10-07/acc-preintegration-match.txt.

Fusion-horizon covariance diagnostics
-------------------------------------

The horizon replay now accepts a diagnostic CSV after the integration mode:

  horizon_replay IMU.csv MODEL_INPUT.csv OUTPUT.csv denied foh COVARIANCE.csv

Its t_s is publication time; fusion_t_s is the epoch of the recorded filter
covariance and fusion_ready marks a real released horizon. The published
predictor's pose does not carry this covariance. Score it explicitly with:

  python tools/estimator_comparison/consistency.py \
    --estimate OUTPUT.csv --covariance COVARIANCE.csv --truth TRUTH.csv \
    --gyro-bias .0008 -.0005 .0004 --accel-bias .02 -.01 .015 \
    --fusion-horizon --output CONSISTENCY.json

Those bias vectors belong to the synthetic capture generator and are scoring
truth only. consistency.py refuses delayed diagnostics without the explicit
option and checks matching publication/fusion epochs and readiness. Existing
retrodiction diagnostics and their default scoring retain their earlier format.

check_horizon_consistency.py --help replays both heights and all GPS scenarios
from an altitude reference, checks unchanged primary-output hashes and packet
delivery, and records windowed delayed-state consistency. Use new owned work
paths and matching original/higher capture directories. Its flight window ends
at 59.7 s in fusion time. report_horizon_consistency.py combines both motion
sets with --magnetic-ablation, preserving the rejected diagnostic experiment.
See docs/reviews/2026-10-07/horizon-consistency.txt. Flight covariance results
are encouraging in this controlled set, while stationary startup remains
overconfident. No native covariance ranking or formal Monte Carlo claim is made.

FOH/ZOH and stationary IMU ablation
----------------------------------

compare_hold.py separates hold order from the optional stationary IMU model.
Every run retains the existing declared-rest velocity correction until 13 s.
Only stationary-enabled binaries replace inertial prediction during that interval
with stationary prediction and a six-axis gyro/bias and gravity/bias observation.
The estimator option useStationaryImu defaults to false; a trusted vehicleAtRest
signal is also required. This experiment does not validate a rest detector.

Build both variants from the same source and compiler:

  hold_root="$HOME/scratch/modelica_models/hold-comparison"
  python tools/estimator_comparison/build.py --eskf-only --horizon \
    --geometric-alignment --equivariant-magnetometer \
    --output "$hold_root/normal"
  python tools/estimator_comparison/build.py --eskf-only --horizon \
    --geometric-alignment --equivariant-magnetometer --stationary-imu \
    --output "$hold_root/stationary"

Pass their horizon_replay/modelica_replay executables as --horizon,
--retrodiction, --stationary-horizon and --stationary-retrodiction to
compare_hold.py. Supply the original and held-out altitude reference JSONs
with --references, their two original capture roots with --captures, and the
matching four-metre capture roots with --higher-captures. Choose new --work
and --output paths. The script verifies capture hashes, exact frozen FOH
controls, sensor delivery, output epochs and finite states before scoring.
Per-run score checkpoints remain in --work; raw covariance files are removed
after scoring. report_hold.py --input SCORES.json --output REPORT writes a
text summary and CSV with all paired cases and windows.

The stationary_imu.c bridge checks the actual Rumoca 0.10.2 generated functions
against an independent stationary process and correlated Gaussian correction.
Compile it as a shared library with the estimator ProductionCode include path
and generated rumoca_galec_kernels.c; it includes the estimator C itself.
check_stationary_imu.py --library LIBRARY.so --output CHECK.json exercises
held-input packets at several durations and nonzero bias anchors. Its use of
generated scratch names is compiler-version specific. This numerical check
does not establish exact FOH stochastic covariance or flight performance.

The 288-run results, eight no-rest controls and numerical kernel check are in
docs/reviews/2026-10-07/eskf-hold-stationary.txt and the accompanying JSON/CSV.
Host thread CPU measurements exclude CSV I/O and are not target WCET results.

Explicit optical-flow exposure probe
------------------------------------

flow_exposure.py --source CAPTURE --output NEW_CAPTURE creates 100 ms image
exposures at 10 Hz from a frozen high-rate capture. It preserves the navigation
IMU, GPS and truth, and samples the original magnetic/barometric captures at
10 Hz. The image integrates physical scene rotation plus the original image
translation noise. An independent unbiased calibrated camera gyro supplies
the rotation integral; navigation-IMU noise is not inserted into the image and
then cancelled. No sensor packet reaches before an actual exposure exists.

The extended flow.csv explicitly carries exposure end, midpoint, duration,
FLU image/gyro angle integrals and marginal variances. Native adapters divide
the angles by the duration and convert axes. The ESKF driver accepts this
file through --flow-packets FLOW.csv and uses its midpoint epoch and covariance.
Without that option, the historical mapping and fixed configuration are retained.
The fusion-horizon adapter explicitly enables variable exposure configuration;
its regression checks both that path and preservation of fixed legacy settings.

The exposure transport profile uses GPS/flow/mag/baro delays 110/30/60/20 ms
and no jitter. The 60 ms magnetic delay represents EKF3's fixed magnetic timing
assumption. Native half-filter-step conventions remain unchanged. Range needs
further work: EKF3 subtracts 25 ms from range read time and applies a median
filter, whereas this common packet carries midpoint range. Explicit flow
exposure does not resolve that separate range timing/preprocessing difference.

To reproduce the current probe, create new captures named 0.6_7_2m,
0.6_7_4m, 0.85_101_2m and 0.85_101_4m from their frozen altitude-study inputs.
Build the normal ESKF horizon/retrodiction executables with geometric alignment
and vector magnetometer enabled; stationary IMU stays disabled. Build two
PX4 adapters through audit_native_configuration.prepare_px4(args), passing
exposure_flow=False/True and the same frozen imu_noise_density to each. Source
and library arguments follow the earlier native audit workflow. These functions
stage only adapters; native cores and their source pins remain unchanged.

compare_exposure.py --help lists the required capture root, four executables,
pinned native paths, references, transport trace and new work/output paths.
It runs legacy and explicit mappings on identical captures and arrival traces,
checks actual EKF3 DataFlash serialization and logged PX4 API values, and scores
current outputs plus ESKF covariance at its correct epoch. Native audit output
retains magnetic selection and PX4 last-fuse diagnostics. Eight separate frozen
controls establish compatibility with historical outputs. report_exposure.py
accepts --input SCORES.json --controls CONTROLS.json --output REPORT.

The dated native-exposure.txt/JSON/CSV retain all 96 probe scores, eight controls
and remaining limits. This two-seed probe does not replace the full held-out,
sensor-disturbance or flight campaign. Marginal exposure variances do not model
shared-endpoint correlations exactly, and native tuning/source policies still
differ. Raw native code remains exclusively in the validation submodules.

The legacy retrodiction mapping failed positive-definiteness checks in both
held-out GPS-denied height cases. These runs retain their accuracy results and
explicitly report unavailable NEES, failed sample counts and the first failed
covariance matrix. No diagonal jitter or dropped samples conceal the failures.
All explicit-mapping covariance samples passed this probe; that does not prove
general numerical robustness. native-exposure-validation.json and the manifest
record exact controls, capture/report reproduction, rejected corrupt evidence
and the unchanged generated Modelica objects and native core hashes.

Covariance-reset numerical regression
-------------------------------------

The first legacy retrodiction failure is captured in
fixtures/ill_conditioned_reset.json: the prior covariance, SE_2(3) reset
Jacobian and generated result at 37.62 s. Instrumenting the generated estimator
preserved the original replay output byte for byte. The same congruence in
float64 remains positive definite. Double accumulation or compensated summation
in that reset alone does not fix either complete failing replay; these generated
C experiments are diagnostic candidates, not promoted Modelica changes.

LinearAlgebra.covarianceRoot computes a lower triangular factor from rectangular
noise-factor columns using scaled Givens rotations. It preserves zero and
rank-deficient inputs without adding noise. Tests.CovarianceRootTests checks
rectangular dimensions, triangular structure, sign and degenerate inputs.
Tests.CovarianceRootReplay supplies dynamic 15-by-30 inputs to actual Rumoca
generated float32 code. covariance_root.c exposes that block to
check_covariance_root.py, which compares it with independent NumPy QR on two
seeded sets of anisotropic inputs and the captured reset. tools/ci.py includes
the export, shared-library build and both numerical seeds.

To reproduce separately, export Tests.CovarianceRootReplay to a new owned build
directory with Rumoca 0.10.2, then compile covariance_root.c together with that
export's Tests_CovarianceRootReplay.c and rumoca_galec_kernels.c into a shared
library. Pass --library LIBRARY and --output CHECK.json to
check_covariance_root.py; --case defaults to the committed numerical fixture.
The dated eskf-covariance-reset.txt/JSON/manifest retain the root-cause evidence,
rejected candidate replays and primitive checks. This review preceded the
end-to-end prototype described below.

Opt-in square-root covariance prototype
---------------------------------------

ESKF.Estimator.useSquareRootCovariance selects a lower covariance factor for
prediction, correlated observations, constrained gains, injection resets and
covariance bounds/recovery. It defaults to false. Dense covariance remains
available for diagnostics and auxiliary terrain/barometer calculations.
Direct ESKF.State constructors now require useSquareRootCovariance and
covarianceRoot; initialize supplies both automatically.

build.py --square-root-covariance enables the representation in generated-code
replays. Build control and candidate directories with the same --horizon,
--eskf-only, --geometric-alignment and --equivariant-magnetometer options, adding
--square-root-covariance only for the candidate. Use owned directories under
$HOME/scratch/modelica_models, with Rumoca 0.10.2 and the same compiler settings.

compare_square_root.py takes --captures from the frozen exposure study,
--reference docs/reviews/2026-10-07/native-exposure.json, four binaries via
--horizon-control, --retrodiction-control, --horizon-root and --retrodiction-root,
a fresh --work directory and --output JSON. The retrodiction executable is
modelica_replay; the horizon executable is horizon_replay. It requires all 48
disabled-mode outputs to match the frozen reference exactly and identical
sensor transport for all 96 executions. It refuses invalid factors, nonfinite
states or lost packets. report_square_root.py takes --input JSON, --reference,
--text and --csv to produce every paired result, including regressions.

Root diagnostics append l0_0 through l14_14 after the existing dense P columns.
consistency.py checks lower-triangular structure, positive finite diagonals and
reconstruction before computing NEES with L. It counts dense rounding failures
separately, preserving every sample. The disabled diagnostic format is unchanged.

Tests.SquareRootCorrectionReplay and square_root_correction.c expose the actual
correction to the existing independent Gaussian/geometry reference checker.
tools/ci.py exports, builds and checks it for two seeds alongside the legacy
correction and QR primitive. The dated review includes four seeds: 2,880
corrections covering correlated noise, projected/heading gains, trust limits,
both bias geometries and exact preservation on rejection.

eskf-square-root.json/.txt/.csv and the manifest retain the 96-run comparison.
All 238,560 scored factors passed; the two legacy full-P failures disappeared
in these replays. Large legacy flow drift remains. Typical explicit-exposure
accuracy changes are small, with regressions retained. Filter CPU costs about
23–26 times the paired control and generated state storage grows 55,192 bytes
even with the feature disabled. This prototype needs optimization and dedicated
recovery/dropout qualification before production use. It preserves the existing
approximate process-noise model and does not prove superiority over either
native estimator; their measurement floors and source policies still differ.

QR loop optimization
----------------------

The follow-up eskf-root-optimized.json/.txt/.csv preserves all 48 root outputs
and covariance diagnostics byte for byte, alongside 48 unchanged full-P
controls. To reproduce this additional preservation check, pass the previous
eskf-square-root.json as --root-reference to both compare_square_root.py and
report_square_root.py.

The QR implementation updates the two rotated rows in separate, short loops
over trailing columns. Rumoca 0.10.2 then emits direct indexed stores, avoiding
the former full-work-matrix selection sweep for each rotation. The scaled
Givens arithmetic and noise model are unchanged. Independent numerical checks
cover the captured covariance reset and arbitrary anisotropic inputs; one of
1,000 extreme primitive cases differs only in the sign of zero, with every
nonzero factor entry bit-identical.

Filter CPU drops about 86% relative to the first root prototype. It still costs
about 3.4–3.6 times the full-P filter, or 1.9–2.6 times total replay-component
CPU. These are host thread timings, not target WCET. Whole-state storage is
unchanged: 40,076 bytes of the 55,192-byte growth is in generated step scratch,
which dominates the small QR local-buffer reduction. The representation
remains opt-in; memory, recovery/dropout qualification and native measurement
policy matching remain open.
