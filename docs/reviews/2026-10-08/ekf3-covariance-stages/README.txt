EKF3 constrained-gain covariance failure: operation trace
========================================================

This follow-up isolates the first invalid covariance in the common-sensor-noise
GPS and GPS-loss/return cases. It does not repair either replay or count its
failure as an ESKF accuracy win. The full estimator comparison remains open.

The actual pinned Copter-4.7.1 core is dbe792162d06cab66c3475fd5556bf7a120f119e.
Only read-only covariance/gain/state observers were added to a separate owned
source copy. Both complete 167 s replays reproduce the prior published state
CSV AND every full native logging covariance snapshot byte for byte. A second
execution with the portable verification driver reproduces the same results.
No ArduPilot or PX4 source is included in this repository.

First failure and its cause
--------------------------
At publication 123.7 s, fusion horizon 123.5 s, the first scalar GPS north
velocity update has a positive-definite common 15D prior. Native aiding mode is
AID_ABSOLUTE (0). The badIMUdata flag is true, while global and per-axis bias
inhibition flags are false. The native bad-IMU branch zeros all accelerometer
bias gain entries, indices 13, 14 and 15. Those are the only gain entries that
differ from the unconstrained covariance-derived gain at the failure.

The update then applies P <- P - K H P, with the constrained K, and averages
the two triangles in ForceSymmetry. ConstrainVariances does not remove the
resulting negative mode. Reconstructing these exact operations agrees with
the recorded matrices to 1.31e-18 maximum absolute entry error.

Stage                          Common 15D minimum eigenvalue
Before scalar correction        +2.8968623740e-9
After left update, symmetric part -1.0062194513e-6
After ForceSymmetry              -1.0062194513e-6
After ConstrainVariances          -1.0062194513e-6
Offline Joseph, same prior/gain   +2.8968711559e-9

After ForceSymmetry, the native 24D matrix itself has a negative eigenvalue
(-1.0022012814e-8). Thus this failure is present before the common-coordinate
transformation. Eigenvalues mix physical units; the common Cholesky failure,
the native negative mode, the explicit stage reconstruction and the same-gain
counterfactual together establish this particular numerical/algebraic failure.
Asymmetric snapshots are labeled and never accepted as valid covariances.

The failure is identical in GPS and transition replays because it precedes
the GPS outage at 132 s. There are 18 scalar corrections in each recorded
123.6-123.8 s publication window. A second north-velocity correction at 123.8 s
also turns a valid prior indefinite. Full-run logging failures are 4312 GPS
and 2342 transition rows in the physical 10-166.7 s diagnostic window.

For arbitrary K, the covariance of the corrected error is
  (I-KH) P (I-KH)' + K R K'.
Replacing it with (I-KH)P relies on the optimal unconstrained gain identity.
Averaging an asymmetric result does not restore that identity or ensure PSD.
The 2D regression counterexample independently demonstrates this distinction;
the optimal-gain control makes the simplified and Joseph updates agree.

This trace establishes which branch suppresses the gains, not why badIMUdata
was initially activated. That flag is already true at the beginning of the
bounded trace. Its earlier vertical-velocity/independent-height trigger,
startup history, packet semantics and phase hints still need investigation.
Do not infer a general ArduPilot flight failure or intrinsic estimator ranking.
In particular, scalar innovations logged after the bad-IMU velocity override
cannot substitute for tracing its original pre-override trigger residual.

ESKF implication
----------------
Our full-P ESKF correction already calls LinearAlgebra.josephUpdate with its
effective constrained gain, then applies the Lie reset congruence. No ESKF
algorithm/default changes were made for this audit. The recorded native witness
also passes the existing generated production Joseph function in float32,
after the explicit symmetry averaging used by correctLinear. See
generated-joseph.json, its thin generated-code wrapper and reproduction script.
This exercises the covariance algebra with the observed gain; it does not
turn that gain policy into a full ESKF replay or validate overall superiority.
Existing joint-pressure/generated-correction and conditional Lean audits cover
additional gain constraints and resets under their documented assumptions.

Evidence
---------
gps.json and transition.json retain input/source/binary hashes, independent
state/full-covariance parity and complete bounded-stage diagnostic summaries.
first-failure.csv contains all five raw snapshots of the first failing scalar
update, including complete native 24D P, state, actual gain, R and inhibition
flags. Both scenarios have the same bytes for this witness. It can be checked
without the external firmware or large scratch replay artifacts:

  python tools/estimator_comparison/diagnose_ekf3_covariance_stages.py \
    docs/reviews/2026-10-08/ekf3-covariance-stages/first-failure.csv \
    --output "$HOME/scratch/modelica_models/ekf3-witness.json"

Use instrument_ekf3_covariance_stages.py CLEAN_OWNED_SOURCE --output MANIFEST
on a clean pinned ArduPilot source copy, then configure/build native Replay.
The instrumenter also adds the ordinary full-covariance logging observer.
replay_ekf3_covariance_stages.py verifies the source transformation, original
pilot/covariance manifests, capture/noise/arrival hashes, and post-replay parity.
Its required arguments are shown by --help. Use --start-us 123600000
--end-us 123800000, --scenario gps or transition, the corresponding frozen
capture, common-sensor-noise/pilot.json and native-covariance.json, and the
native-releases/native-exposure.json noise reference. All source copies,
builds and full replays belong below $HOME/scratch.

The generated Joseph check uses the existing Rumoca 0.10.2
Tests_BarometerConsiderReplay/ProductionCode cache, whose C/header hashes are
recorded in generated-joseph.json. Compile joseph_probe.c with that directory
on the include path and its rumoca_galec_kernels.c, using -O2 -fPIC -shared -lm.
Place the resulting joseph_probe.so and a copy of its wrapper in the owned
artifact directory alongside the gps-verified/transition-verified replay
directories and their JSON evidence. check-joseph.py accepts --artifact-root,
--generated-code and a required fresh --output path. Its default directories
derive from $HOME and match this audit's scratch layout. The recorded build
used GCC 15.2.0; the check covers four cases, two unique failing updates.

The C++ observer tests require a compiler, exercise one shared CSV stream across
two translation units, enforce the time bounds and reject overwriting an
existing trace. Unit tests also reject incomplete or incorrect scalar traces.

CI follow-up
-------------
The comparison commit 366a85a exposed two unit-test import errors in hosted CI
37747170785: optional pymavlink was imported while loading pure covariance and
campaign-validation helpers. The error was reproduced with the exact hosted
Python environment locally. Commit 1addf80 defers the DataFlash import until
native diagnostics actually run and uses the standalone hash helper. All 56
comparison tests now pass in that hosted environment without pymavlink; no
test is skipped in the recorded run. Ruff also passes. Native replay parsing
still requires pymavlink, as before. RDD2 qualification remains unresolved.

Pinned native implementation references
---------------------------------------
https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp

Next: isolate the earlier bad-IMU trigger, then broaden independent matched
seeds, motions and heights with explicit readiness, covariance, NIS and noise-
policy checks. A native failed covariance case remains excluded from rankings.
