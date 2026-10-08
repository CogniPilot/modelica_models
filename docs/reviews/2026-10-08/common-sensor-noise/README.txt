Common sensor-noise comparison
==============================

The new capture and comparison profile align several previously unequal sensor
assumptions. Physical GPS velocity noise is now 0.05/0.05/0.075 m/s, magnetic
noise is 1 microtesla per axis, and compensated camera flow noise is 0.05 rad/s
per axis. ESKF receives the corresponding covariance. Native settings use these
GPS/flow/magnetic values, pressure sigma 0.1 m, terrain range variance 0.0004 m2,
and the previously derived continuous IMU/bias process-noise mapping. The profile
was derived from pinned native source floors, not selected for a favorable RMS.

PX4's native vertical GPS velocity variance is 2.25 times horizontal variance.
The new profile accounts for this in the physical capture, ESKF covariance and
EKF3 vertical-noise setting. Effective-noise.json verifies actual native GPS
position variance 0.04 m2, horizontal velocity 0.0025 m2/s2, vertical velocity
0.005625 m2/s2, flow 0.0025 rad2/s2, and observed magnetic variances on every
applicable correction row. This is stronger than assigning equal parameter
numbers to fields with different units or internal multipliers.

The full effective models remain unequal. PX4 height observation variances
include its estimated height-bias uncertainty; EKF3 inflates pressure variance
fourfold during the declared takeoff interval. Range/terrain propagation,
source selection, magnetic states and priors differ. The ESKF and PX4 use GPS
altitude as well as pressure; this EKF3 configuration uses barometric height.
At 2 m EKF3 uses magnetic heading; ESKF uses vector magnetic fusion. Identical
available packets do not imply identical accepted corrections. These are
configured-stack measurements, not an intrinsic algorithm ranking or fully
matched R/Q experiment. The terrain-only EKF3 range-noise mapping retains the
documented source-unit discrepancy from the earlier native aiding-noise audit;
it is not a deployment parameter recommendation.

The physical schedule is unchanged from the common-warmup pilot: stationary
until takeoff at 120 s, loss 132 s, return 147 s, mission end 167 s. Seed 911,
trajectory parameter 0.6, climb 2 m. All cores use the same actual capture and
sensor arrival trace. Position RMS uses common 100 Hz epochs 120-166.7 s;
covariance uses each filter's physical fusion epoch. The native cores remain
the pinned PX4 v1.17.0 and ArduPilot Copter-4.7.1 releases. No native or Modelica
estimator equation, vehicle default, readiness threshold or gate was changed.

Results
-------
Flight horizontal/vertical position RMS (m), followed by mean common 15D NEES:

scenario     configuration          horizontal   vertical     NEES
GPS          horizon                  0.04136     0.03244      6.73
GPS          retrodiction              0.04059     0.03715      7.51
GPS          horizon joint             0.04142     0.03103      6.48
GPS          retrodiction joint        0.04063     0.02398      7.23
GPS          native PX4                0.16162     0.05852    917.96
GPS          native EKF3               1.65770     0.05837   undefined
denied       horizon                  0.18146     0.03415      3.76
denied       retrodiction              0.18325     0.03924      3.75
denied       horizon joint             0.17624     0.02801      3.55
denied       retrodiction joint        0.17609     0.03354      3.54
denied       native PX4                0.24178     0.05868     41.66
denied       native EKF3               0.26278     0.06853     44.30
loss/return  horizon                  0.12364     0.03405      6.89
loss/return  retrodiction              0.12722     0.03849      7.42
loss/return  horizon joint             0.12100     0.03177      6.57
loss/return  retrodiction joint        0.12448     0.02582      7.05
loss/return  native PX4                0.25697     0.06164    637.29
loss/return  native EKF3               0.14880     0.07704   undefined

Each ESKF configuration beats all four valid native reference conditions on
both horizontal and vertical RMS: the three PX4 cases and EKF3 denied case.
The two failed EKF3 covariance cases are retained above but excluded from win
counts. This is one motion/seed; temporal NEES samples are correlated, and a
lower NEES is not automatically better. Large native NEES shows that changing
nominal noise alone does not produce a well-calibrated reference configuration.

The joint pressure-bias option improves vertical RMS in all six paired ESKF
cases. It also improves denied/transition horizontal RMS, with approximately
0.1 percent horizontal regressions in the GPS cases. All 29,820 scored augmented
16D navigation/pressure covariance matrices are positive definite without
symmetrization, jitter or repair; those six audit replays reproduce the original
pilot state CSVs byte for byte. ESKF benchmark entries retain FOH, geometric
alignment and declared startup-rest pressure calibration; these optional features
have not been enabled as new vehicle defaults based on this pilot.

EKF3 failure under the experimental configuration
-------------------------------------------------
GPS fusion starts at 11 s, before takeoff; the original 13 s readiness mismatch
does not explain this failure. Later the GPS case loses nearly all accepted
GPS corrections. Its first covariance failure occurs at fusion time 123.5 s,
published at 123.7 s. The native 24D covariance itself has a negative eigenvalue,
-1.61154e-8, and the mapped 15D matrix has a negative eigenvalue -1.71510e-6.
Its standardized marginal minimum eigenvalue is -0.0013523. There are 4312 failed
Cholesky rows in the complete 10-166.7 s diagnostic interval. NEES is undefined;
no rows or matrices are repaired or replaced. The transition covariance fails
too. Scalar innovation variances can remain positive despite an invalid full
covariance; scalar NIS alone is not a PSD check.

The source operation responsible has not been isolated. This is evidence of a
failed validation configuration, not proof of a general ArduPilot defect or an
ESKF theoretical advantage. All six covariance-observer state CSVs match their
innovation-observer counterparts byte for byte. The read-only observer did not
change the published trajectories.

Rejected setup and independent oracle
-------------------------------------
Before this corrected comparison, the new camera path accidentally used tuple
element 5 (specific force) instead of element 6 (body velocity). A first test
repeated that indexing mistake and missed it. A replacement oracle derives body
velocity and slant range from independent world velocity, position and quaternion
geometry. It rejects that setup: flow residual standard deviations are 0.3292
and 0.5841 rad/s against the declared 0.05 rad/s. The negative-control failure
is retained. All initial camera-setup replays and covariance diagnostics are
excluded from this comparison and cannot support a noise-only diagnosis.

Trajectory now returns named fields with units, preserving tuple compatibility;
the camera path explicitly selects velocity_body_m_s. The independent oracle
passes on the corrected actual 167 s capture: residual standard deviations are
0.05143 and 0.05019 rad/s, with near-zero means. All default raw/exposure capture
files still reproduce byte for byte after this refactor. The previous published
common-warmup pilot uses the original camera path and is unaffected by this
rejected experimental setup.

Evidence and reproduction
--------------------------
pilot.json contains all 18 corrected state replays, native scalar NIS, ESKF NEES,
readiness failures, binary/input/source hashes and configuration. Its complete
flag means all declared replays executed; readiness_qualified is false because
one native reference loses GPS aiding. native-covariance.json records six native
parity checks and each valid or failed covariance result. joint-covariance.json
records the six full 16D ESKF audits. ekf3-gps-covariance-failure.json retains
the independent native/marginal negative-eigenvalue witness.

effective-noise.json checks the actual native scalar observation variances;
flight-statistics.csv retains all errors, covariance failures and validity flags.
rejected-setup.json binds the excluded setup artifacts still in owned scratch.
default-parity.json records unchanged default captures and eight state CSVs,
plus invalid-noise-input controls. validation.json binds the current tools and
durable evidence. Native candidate coverage is still incomplete, scalar NIS is
selection-conditioned, and this pilot does not export ESKF per-sensor NIS.

Use generate.py CAPTURE --seed 911 --speed 0.6 --warmup-s 120
--common-native-floors, then flow_exposure.py --source CAPTURE --output EXPOSURE.
The origin profile automatically selects matching configurable noise inputs in
compare_readiness.py and its covariance companion. Relink replay.c to include
--measurement-noise support; generated Modelica objects remain unchanged. Keep
large artifacts below $HOME/scratch/modelica_models/common-sensor-noise.

52 Python tests, Ruff, default capture/replay byte parity and invalid-input
controls pass. Main CI 37743028247 finished with Modelica regression and CUBS2
qualification passing, and RDD2 qualification failing: its execution step was
cancelled at 07:45:17 UTC. ci-status.json preserves the terminal job/step status.
The hosted log reports cancellation, without an explicit out-of-memory error.
The outstanding RDD2 Rumoca memory/qualification issue is not claimed fixed by
this benchmark change.

Pinned source references for the noise conventions:
https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp
https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_fusion.cpp
https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp

Next: isolate the native covariance failure operation, complete matched-policy
and effective-noise audits, then broaden independent seeds, motions and heights.
The goal of improving ESKF across those conditions remains active.
