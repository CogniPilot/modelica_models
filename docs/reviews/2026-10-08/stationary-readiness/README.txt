Stationary IMU model with readiness-qualified preflight
8 October 2026

All 96 development replays finish with valid published state coverage and
positive-definite common-15 covariance in every declared window. The stationary
joint-horizon ESKF has lower flight horizontal and velocity RMS than both
native filters in all 24 capture/scenario conditions. This is development
evidence for an existing optional algorithm, not a general superiority claim.
The eight captures have only two independent noise seeds. These captures were
already inspected before this stationary study and are not untouched validation.

Only -DSTATIONARY_IMU_MODEL is added to each of the four frozen builds.
Generated Rumoca 0.10.2 objects, physical noise, sensors, packet delivery,
measurement gates, delay settings and geometric/vector magnetic options are
unchanged. The public reporter checks build definitions and object/binary
hashes, original native/reference manifests, physical input hashes and actual
transport counters. No Modelica source, vehicle defaults or native core changed.
Native PX4 v1.17.0 and Copter-4.7.1 use their recorded release pins.

The existing stationary model propagates a declared resting state with bias
random walks, observes angular rate as gyro bias, and observes coupled gravity
and accelerometer bias. It uses the common declared-rest interval before 120 s;
ordinary FOH inertial propagation resumes at takeoff. It does not mean noiseless
IMU measurements, and the experiment does not qualify a practical rest detector.
All filters receive the same rest/in-air schedule. Stationary correction is
separate from FOH versus ZOH integration; this ablation keeps FOH throughout.

All three scenarios share 120 s noisy preflight, first GPS at 21 s, loss at
132 s and return at 147 s. Flight accuracy uses 120-166.7 s. Two seeds crossed
with 0.45/0.85 rad/s motion and 2/4 m heights produce eight captures. Baseline
native state/covariance results are reused from ../matched-campaign with frozen
SHA bindings. comparison.csv includes all 720 rows for ten estimator variants,
eight captures and three scenarios/windows. pairs.csv retains every metric,
window, missing/invalid denominator, median difference and worst loss.

Median capture-level flight RMS and mean NEES:

scenario       estimator                    horiz m   vert m    vel m/s  att deg  yaw deg  NEES15
gps            stationary_horizon            0.04386  0.03285  0.01718  0.11436  0.10261  7.86895
gps            stationary_retrodiction       0.04468  0.03615  0.01748  0.10551  0.09353  8.21158
gps            stationary_horizon_joint      0.04389  0.02849  0.01564  0.11397  0.10212  8.11227
gps            stationary_retrodiction_joint  0.04465  0.02556  0.01655  0.10528  0.09328  8.45177
gps            px4                           0.17219  0.04755  0.13308  2.70993  2.69741 1421.65137
gps            ekf3                          0.05460  0.05644  0.04824  0.24610  0.18578 42.94359
denied         stationary_horizon            0.33707  0.03831  0.04166  0.26117  0.24094  6.90753
denied         stationary_retrodiction       0.33946  0.04161  0.04081  0.24217  0.22381  6.68310
denied         stationary_horizon_joint      0.30780  0.03361  0.04080  0.26847  0.24893  7.07350
denied         stationary_retrodiction_joint  0.31416  0.03767  0.03955  0.26126  0.24252  6.82498
denied         px4                           0.40394  0.03157  0.05817  1.00698  0.99127 26.40346
denied         ekf3                          0.41904  0.07278  0.12588  0.22086  0.15613 41.38962
transition     stationary_horizon            0.11085  0.03321  0.02393  0.12639  0.11141  7.23708
transition     stationary_retrodiction       0.11477  0.03636  0.02341  0.10494  0.08955  7.52780
transition     stationary_horizon_joint      0.11094  0.03077  0.02310  0.12936  0.11453  7.50376
transition     stationary_retrodiction_joint  0.11349  0.02872  0.02201  0.10599  0.09082  7.79510
transition     px4                           0.27465  0.04956  0.13694  2.78785  2.77369 886.21102
transition     ekf3                          0.15687  0.06141  0.07380  0.26918  0.21138 42.56999

Remaining gaps and tradeoffs

ArduPilot has lower denied-flight yaw RMS than stationary joint horizon in
6/8 captures. PX4 has lower denied-flight vertical RMS in 5/8. Stationary joint
horizon also loses vertical RMS to PX4 in 3/8 GPS and 2/8 transition captures;
it loses yaw RMS to ArduPilot in 1/8 GPS and 3/8 transition captures.
No universal or componentwise superiority claim follows.

Against corresponding original joint horizon, stationary joint horizon improves
denied horizontal RMS in 8/8 cases (median -14.87%) and denied yaw in 8/8
(median -24.93%). It worsens GPS vertical RMS in 8/8 (median +3.22%, maximum
increase 0.00156 m), and GPS yaw improves in only 3/8 (median change +5.64%).
Transition horizontal RMS improves in 4/8 with a median change of +0.03% and
worst increase 0.00898 m. All losses remain in the paired report. The original
seed-911 pilot had much larger attitude/yaw gains than these eight conditions;
pilot.json retains all twelve cases without treating that pilot as validation.

All 48 joint-pressure replays also retain full-16 covariance checks over
238,560 actual float32 matrices from 117-166.7 s at each filter's own state
or fusion epoch. Every full matrix is symmetric and positive definite, every
pressure Schur complement is positive, and no repair/jitter is used. These
raw diagnostic hashes and every minimum Schur value are in summary.json.
No new per-sensor NIS observation is made for the stationary candidate here;
the prior nonstationary sensor audit cannot establish stationary NIS parity.
All common-15 consistency windows, CPU measurements and recovery statistics
are retained in ancillary.json. CPU includes preflight and host effects,
and is not target worst-case timing or a fair native CPU comparison.

Native effective Q/R, priors, terrain processing and magnetic-state policies
remain stack-specific despite identical physical captures. ESKF uses a fixed
calibrated field vector; native filters estimate or select magnetic states.
NEES below 15 is not automatically superior. Correlated samples, fixed initial
bias truths and two noise seeds do not support independent Monte Carlo
confidence or theoretical dominance. Real sensor disturbances, exact shared-
endpoint FOH covariance and full nonlinear estimator theory remain open.

Fresh independent validation was declared before generation/replay: four new
noise seeds, two new motions and two new heights, yielding sixteen captures.
independent-validation-declaration.json preserves that plan. It calls for
480 state replays plus 96 native covariance replays and retains the original
ESKF controls. No results from that live campaign are included or claimed here.
No default adoption is made from the present development result.

Reproduction and provenance

build.json retains the exact compiler commands and binary/object hashes.
declaration.json precedes the known-seed pilot; campaign-declaration.json
precedes the 96 development replays and binds the frozen tool snapshot.
campaign.py and pilot.py preserve the exact development drivers. Large raw
captures, generated code, binaries and covariance traces stay in the owned
$HOME/scratch/modelica_models/stationary-readiness directory. The original
captures stay in $HOME/scratch/modelica_models/ekf3-imu-integrity/held-out.
Source/compiler paths in build manifests describe the measured local build;
they are evidence, not portable project configuration. summary.json binds the
reporter/imports and all candidate/reference manifests; evidence-sha256.json
binds every copied artifact in this directory.

Run tools/estimator_comparison/report_eskf_ablation.py with:
  --candidate "$HOME/scratch/modelica_models/stationary-readiness"
  --references "$HOME/scratch/modelica_models/ekf3-imu-integrity/held-out"
  --reference-summary docs/reviews/2026-10-08/matched-campaign/summary.json
  --reference-declaration "$HOME/scratch/modelica_models/ekf3-imu-integrity/fix-study/declaration.json"
  --baseline-build docs/reviews/2026-10-08/common-sensor-noise/replay-build.json
  --output FRESH_REVIEW_DIRECTORY

The first reporting attempt failed on single-key tuple indexing before writing
an output directory. report-initial-failure.txt preserves that failure. After
fixing the reporter, report-v2 passed without repeating any estimator replay.
Seven reporter negative controls check input/delivery binding, missing scenarios,
full-pressure qualification, covariance failures, missing denominators and
preventing candidate rows from inheriting baseline accuracy or coverage.

Hosted CI run 37759625735 at e011989 passed Modelica regression and CUBS2.
RDD2 qualification was cancelled and the overall run failed. The local Rumoca
memory issue and RDD2 mission qualification remain unresolved. This report
changes comparison tools/evidence and does not claim a CI or RDD2 fix.
