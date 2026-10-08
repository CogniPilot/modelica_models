EKF3 bad-IMU trigger and matched GPS first-fix control
=====================================================

The common-noise EKF3 covariance failure traced in the preceding review starts
with an earlier noise-triggered integrity event during stationary preflight.
This review records the original residuals before the native velocity override,
then tests one common input change: GPS is unavailable until physical 21 s.
The native algorithm, detector thresholds, sensor noise draws and all ESKF
parameters/binaries remain unchanged. This is a causal diagnostic on the known
seed-911 capture, not a held-out result or a general estimator ranking.

Actual immediate-GPS trigger
----------------------------
The pinned Copter-4.7.1 native core first evaluates the observed vertical
integrity check at publication 11.2 s. badIMUdata and its event timer are zero.
Because the time since that zero timer is less than 15 s, the native threshold
multiplier is 1, rather than 9. No preceding bad-IMU event is required for this
initial one-sigma threshold. The first observation is below both thresholds.

At publication 12.4 s, fusion epoch 12.2 s, the stationary vehicle has:
  height residual                 -0.226754481893 m
  vertical velocity residual      -0.110431378297 m/s
  height observation sigma         0.100000001490 m
  vertical velocity sigma          0.075000002980 m/s
  normalized residuals            -2.267544785 and -1.472418319
  threshold multiplier             1

The residuals have the same sign and exceed the native one-sigma bounds, so
the native last-bad timestamp becomes 12.4 s. The hold decision uses elapsed
time computed before that timestamp update; the flag first becomes true at
12.5 s, even though that next pair does not itself satisfy the trigger.

The 10 s hold keeps the detector at its one-sigma recovery threshold. Further
ordinary noisy observations repeatedly update the timestamp. The complete
11.2-167 s trace contains 1559 evaluated before/after pairs, 174 trigger
conditions, and 1546 checks ending with badIMUdata true. The flag never clears
after its first activation. Every branch, timestamp update and velocity override
reconstructs exactly from the recorded pre-override values. The simulator's
preflight truth is stationary, with IID IMU noise plus constant biases and no
injected aliasing fault. physical-controls.json records the actual draws.

The detector normalizes by observation variances, not by total innovation
covariance. These normalized residuals are not Kalman NIS. A native GPS velocity
innovation observed later, after replacing the estimate with GPS velocity,
cannot reveal this original trigger. No takeoff hint is active at 12.4 s.

Matched first-fix diagnostic
----------------------------
The --gps-fix-after-s 21 capture option changes only GPS fix/validity flags,
the merged GPS-fresh flag, and metadata before 21 s. All physical sample values,
noise draws, and non-GPS files are identical. All samples after 21 s are
identical. The same revised capture/arrival schedule is used by all six filter
configurations. Default captures still reproduce the old bytes for identical
API arguments. The CLI's float 120.0 versus an older API integer 120 changes
JSON serialization only; this distinction is recorded rather than hidden.

EKF3 first evaluates the delayed-fix vertical check at publication 31.4 s,
with threshold multiplier 9. Its full GPS case has 1357 integrity checks and
its transition case has 1207; neither has a trigger or an active bad-IMU flag.
The integrity observers reproduce the independently logged native state AND
full covariance outputs byte for byte. All six native covariance replays now
have valid common 15D covariances, with matching published states. All native
GPS readiness/loss/return checks pass. The six GPS-denied state CSVs are also
byte-identical to the original immediate-GPS comparison, as expected.

Selected flight RMS, metres; full 18-case metrics are in flight-statistics.csv:
Scenario       Filter                 Horizontal    Vertical
GPS            ESKF horizon             0.041356     0.032443
GPS            ESKF retrodiction         0.040586     0.037153
GPS            ESKF joint retrodiction   0.040629     0.023981
GPS            PX4 EKF2                 0.161597     0.052900
GPS            ArduPilot EKF3           0.046190     0.050428
GPS-denied     ESKF horizon             0.181459     0.034151
GPS-denied     ESKF joint retrodiction   0.176086     0.033536
GPS-denied     PX4 EKF2                 0.241778     0.058679
GPS-denied     ArduPilot EKF3           0.262783     0.068535
Loss/return    ESKF horizon             0.123639     0.034044
Loss/return    ESKF joint horizon        0.120980     0.031766
Loss/return    PX4 EKF2                 0.257305     0.056355
Loss/return    ArduPilot EKF3           0.138218     0.054523

EKF3 GPS horizontal RMS changes from the failed immediate-GPS case's 1.6577 m
to 0.04619 m. ESKF retains a position RMS lead in all six valid native conditions
of this capture. This is not a lead on every metric: EKF3 GPS yaw RMS is
0.16382 deg, compared with 0.17579 deg for ESKF horizon. All metrics and failures
must remain visible when choosing improvements.

Common-state flight mean NEES spans 3.54-7.51 for the ESKF variants, 41.66-910.68
for PX4, and 38.11-44.30 for EKF3. These are descriptive values on correlated
flight samples, not independent Monte Carlo confidence tests. NEES below 15 is
not automatically superior; effective Q/R, priors, terrain, magnetic/height
policies and native observation acceptance still differ. Native scalar NIS
retains its candidate-selection and sensor-coverage limitations. The optional
joint pressure update and fusion-horizon/retrodiction comparison are unchanged.

Evidence and reproduction
--------------------------
gps-immediate-integrity.csv contains the complete raw 1559 pre/post pairs;
gps-immediate-integrity.json binds native source/binary, physical capture,
state/covariance parity and every trigger. first-trigger.csv is a small raw
witness including the initial check, the first trigger and the next activation.
gps-late-integrity.json and transition-late-integrity.json retain the complete
late-fix branch summaries and parity evidence. late-fix-pilot.json contains all
18 state replays, native scalar NIS, ESKF NEES and timing. Its covariance
companion contains all six native consistency audits. default-parity.json,
physical-controls.json and denied-controls.json record the unchanged controls.

instrument_ekf3_imu_integrity.py adds read-only hooks to a clean owned pinned
native source copy, including the ordinary covariance logging observer. Build
native Replay in that owned scratch directory. The existing verified replay
driver accepts --imu-integrity, omitting stage time bounds, and still requires
the frozen pilot, covariance manifest, capture/noise/arrival hashes and exact
published-state/full-covariance parity. Pure diagnostics need no firmware:

  python tools/estimator_comparison/diagnose_ekf3_imu_integrity.py \
    docs/reviews/2026-10-08/ekf3-imu-integrity/gps-immediate-integrity.csv \
    --output "$HOME/scratch/modelica_models/integrity-check.json"

Generate the controlled variant with the existing seed 911, speed 0.6,
--warmup-s 120 --common-native-floors --gps-fix-after-s 21, then use the ordinary
exposure, readiness and covariance drivers. Keep all large artifacts under
$HOME/scratch. No ArduPilot/PX4 implementation source is committed here.

Independent campaign
---------------------
declaration.json fixes an eight-capture factorial before any held-out data was
generated: seeds 20271009/20271010, frequencies 0.45/0.85 rad/s, heights 2/4 m,
120 s warmup and first GPS fix at 21 s. The four ESKF variants and both native
cores run all GPS/denied/loss-return scenarios, for 144 state replays and 48
native covariance replays. held-out-captures.json records independent camera
geometry/noise checks and input hashes for every capture. A frozen tool snapshot
is bound by held-out-source-snapshot.json. Every result, readiness failure and
invalid covariance will be retained. The campaign is running; it is not scored
or claimed complete in this review. It must finish before selecting a new ESKF
algorithm/default based on these conditions. Broader noise/outage/terrain cases,
complete per-sensor NIS and theoretical implementation coverage remain open.

Validation
-----------
61 comparison tests pass in the hosted Python environment, with no skipped
tests. New independent checks distinguish the first trigger from the subsequent
hold activation, reject missing pairs or incorrect overrides, and verify that
GPS first-fix configuration preserves every physical sample. Ruff and Modelica
structure checks pass. RDD2 mission qualification remains unresolved; this
numerical comparison does not fix its Rumoca runtime memory issue.

Pinned native detector and suppression implementation:
https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
