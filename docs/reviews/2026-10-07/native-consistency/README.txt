Native covariance consistency follow-up
======================================

native-covariance.json records 24 completed replays of native PX4 v1.17.0
EKF2 and ArduPilot Copter-4.7.1 EKF3: two frozen captures, two heights, and GPS,
GPS-denied, and GPS loss/return. All use explicit camera exposure packets.
Every published native output CSV is byte-identical to the corresponding
uninstrumented stable-release replay in ../native-releases/native-exposure.json.
Observers read the full native 24x24 covariance without changing filter updates.
Only original observer/test tooling is included here; upstream sources remain
outside this repository and in the validation submodules.

The covariance is mapped into a common 15D local right-error navigation/bias
marginal, preserving cross-covariances. This is not full 24D native NEES, a
nonlinear distributional equivalence proof, or per-sensor NIS. Position and
velocity errors are body-local; attitude is a right rotation error; biases
are rates in FLU. PX4's native attitude perturbation is left/NED. EKF3's
additive quaternion covariance uses the normalized quaternion differential,
and integrated bias covariance is divided by dtEkfAvg. Four tests cover the
full coordinate differential, NEES invariance, quaternion radial direction,
and retention of changed posteriors at repeated timestamps.

EKF3 moves its internal geographic origin. Its exported delayed position must
be translated into the public output origin before comparison with truth.
The final observer performs the same origin translation as the public position
accessors. Treating raw internal position as public position produced invalid
NEES in native-covariance-rejected-origin.json. That interrupted run is retained
solely as rejected diagnostic evidence and must never enter a ranking.

native-covariance-reader-setup.json, native-covariance-epoch-setup.json and
native-covariance-posterior-setup.json retain earlier incomplete setup attempts.
The reader initially expected a t_s column, then rejected repeated fusion
epochs. Native EKF3 can change state/covariance through mode changes without a
new IMU epoch. The final reader retains every observed posterior, including
repeated epochs, and reports their counts. No failed scientific samples are
silently removed or repaired with covariance jitter. Startup is outside the
predeclared scoring windows; raw startup rows remain in scratch traces.

Scoring uses native fusion time, with windows 10-13 s (before takeoff),
13-59.7 s (flight), 25-40 s (outage), and 40-59.7 s (after return).
The latter two windows also apply to GPS and permanently denied controls;
they describe time intervals rather than actual sensor outages in those cases.
These correlated time samples are descriptive, not independent Monte Carlo
trials. GPS-denied global position is not made observable by this statistic.

Per-case scores are in native-covariance.csv. Flight mean 15D NEES ranges:

  Native stack       GPS             Denied        Loss/return
  PX4 EKF2           31.78-63.89      3.53-4.18     26.64-39.62
  ArduPilot EKF3      1.60-2.14        0.97-1.65      1.58-2.17

A well-calibrated independent ensemble would have mean NEES near dimension 15;
smaller is not inherently better. These results suggest overconfidence in
several PX4 GPS cases and conservative EKF3 covariances under this replay
configuration. Effective R/Q, priors, magnetic and height policies are not yet
matched, and the two seeds are coupled to motion. This does not establish
algorithmic superiority. Existing ESKF NEES and RMS remain in the parent
review; this follow-up does not change the ESKF algorithm or establish a new
three-way matched consistency ranking.

Reproduction
------------

Use new owned clean source copies at the revisions in the observer manifests.
Keep large sources, builds and raw covariance traces below
$HOME/scratch/modelica_models/native-consistency. Use
instrument_native_covariance.py once per clean source copy, then rebuild the
native cores with the external harness/build recipes in ../native-releases.
The observer avoids copying the covariance onto the native stack. Raw CSV
traces, approximately a gigabyte for the complete campaign, remain in scratch;
this directory retains hashes and compact durable evidence.

compare_native_consistency.py --help lists explicit source, observer manifest,
binary, harness, reference, frozen case, transport and new work/output paths.
The completed work directory for this run is runs-v6. It verifies source pins,
observer file hashes, frozen physical captures, regenerated arrival trace hashes,
transport metadata and published output parity before computing NEES. Its
per-case work directories retain arrivals and raw covariance separately.
The symlink fix in compare_exposure.py prevents a pre-existing arrivals.csv
from making these case traces share a mutable physical-source file.

Validation
----------

All 32 estimator Python tests and Ruff checks/format checks pass.
validation.json records source, evidence, build and replay log hashes.
This follow-up does not fix the separately reproduced main CI regression:
Rumoca 0.10.2 rejects the newer ESKF pure-call interface, while the merged
ea5c475 baseline passes Tests.StrapdownEstimatorInterfaceTests to 0.02 s.
Do not interpret successful native replays as a passing Modelica CI run.
