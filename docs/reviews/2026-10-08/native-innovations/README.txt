Native scalar innovations expose an initialization mismatch
==========================================================

The new observers reproduce all 24 frozen native state outputs byte for byte.
They capture actual scalar correction innovations, innovation variances and
measurement variances in PX4 v1.17.0 and ArduPilot Copter-4.7.1, using the
unchanged inputs and published trajectories of the stable-release comparison.
The firmware source remains in external validation copies and submodules.
Only the observer utility and orchestration are committed here.

The key finding changes how the previous rankings must be interpreted. EKF3
does not fuse horizontal GPS until 35.6-43.912 seconds in the four GPS cases.
In all four loss/return cases it has zero horizontal GPS corrections in the
13-25 second pre-outage flight interval. Its first GPS correction occurs at
40.1-43.912 seconds, after the nominal GPS return. PX4 begins at 2.12625 seconds
and has 120 GPS-position corrections in the 13-25 second interval in each case.

Consequently, this frozen campaign does not implement the requested established
GPS -> GPS-denied -> GPS-restored experiment for EKF3. Its historical transition
scores describe startup followed by GPS acquisition. They remain valid records
of that configured stack, but do not establish an apples-to-apples GPS loss
ranking, or superiority of PX4/ESKF over an initialized EKF3.

After GPS begins, EKF3's native GPS sample spacing is 100 ms. The initial
apparent average correction rate near 5 Hz was caused by late startup, not
steady downsampling. No firmware admission rule has been changed.

The covariance observer provides independent context because its published
trajectory is byte-identical to this observer's. With compass aiding, native
readyToUseGPS requires delAngBiasLearned. checkGyroCalStatus requires all three
delta-angle-bias variances to be no larger than (radians(0.15)*dtEkfAvg)^2.
That necessary condition is not met at the 13 second takeoff. For the first
2 m GPS case the largest variance is 23.7 times the threshold at takeoff and
reaches it around 35.5 seconds, just before GPS fusion at 35.6 seconds. This is
a necessary-condition audit from post-update snapshots, not observation of
every admission flag or an intervention proving the sole cause.

Required follow-up
------------------
Give all estimators the same physical stationary warmup, and verify actual
native GPS fusion before flight and before the outage. Shift flight, outage,
return, vehicle-phase hints and scoring windows consistently. Do not initialize
one native filter with truth, bypass its readiness checks, change its firmware
admission logic, or silently relabel the existing captures. Retain cold-start
results separately from readiness-qualified flights. Use new independent
seed/motion captures as well as matched validation captures, then re-evaluate
ESKF improvements with actual sensor variances and priors recorded.

Scalar NIS scope
----------------
The observer records the values used by the sequential scalar correction,
rather than reconstructing innovation variance from a gate ratio. EKF3 GPS
and height checks can use different variances from the actual fusion. Its
sequential GPS residuals and variances are recalculated after earlier axis
corrections. PX4's direct GPS correction uses its own stored inputs. Both
behaviors are retained. Header sample_us means the native internal sample
timestamp, which may include native delay/inter-sampling adjustments; it is
not asserted to equal the physical capture timestamp. Zero denotes an
unrecorded timestamp for other auxiliary sources.

Gate candidates and fusion inputs have separate stages. Current gate coverage
is EKF3 optical flow and heading only. Selection-conditioned accepted-update
NIS is not an unconditional chi-square consistency experiment. Per-axis NIS
is not full joint vector NIS, and scalar sums are not promoted to one. The
report retains invalid rows in counts and marks those groups invalid; finite
statistics explicitly exclude unusable rows. All scored groups in this
campaign have finite valid inputs. Temporal samples are correlated.

Coverage includes GPS velocity/position, main height corrections, optical flow,
PX4 magnetic-axis corrections and EKF3 heading corrections. EKF3 three-axis
magnetic corrections, auxiliary terrain/range corrections and complete rejected
GPS/height/PX4 gate candidates are not yet instrumented. In particular, a small
number of heading rows in a 4 m case does not represent its later 3D magnetic
fusion. These gaps are disclosed, rather than treated as zero innovation.

Native effective R/Q, priors, magnetic modes and height policies still differ
from ESKF. Very low observed scalar NIS means neither that a filter is better
nor that its noise should be tuned on these training captures alone. The
measurement-variance columns now expose those differences directly.

Evidence and reproduction
-------------------------
native-innovations.json binds the 24 parity checks and native build/observer
hashes. scalar-statistics.csv contains every scored sensor/axis/stage/window.
readiness.json binds actual GPS events and the independently retained full
native covariance traces. comparison.txt provides the condition-level results.
Five new negative controls check invalid-row retention, stage accounting,
future fusion epochs and the difference between a gate ratio and scalar NIS.
All 39 estimator-comparison Python tests and Ruff pass.

Use new owned sources/builds/traces under
$HOME/scratch/modelica_models/native-innovations. Run
instrument_native_innovations.py once on each clean pinned source copy, then
build with the existing external recipes in
docs/reviews/2026-10-07/native-releases. No native algorithm change is applied.
The release API was rechecked: PX4 latest is v1.17.0; the Copter release is
Copter-4.7.1. Full pinned revisions remain in native_release.py and observer
manifests; the GitHub latest ArduPilot release endpoint itself reports Plane.

Run compare_native_consistency.py --observation innovations with the original
frozen native reference/captures/transport, native sources/binaries, both new
observer manifests, external harness and fresh work/output paths. The default
covariance mode remains supported; its existing manifests pass the strengthened
source verifier unchanged. diagnose_native_readiness.py joins completed
innovation and covariance studies with their retained trace directories. It
checks reference, trace and trajectory hashes before evaluating readiness.
Raw CSVs and generated/build artifacts stay on scratch; compact evidence and
source hashes remain here. No new ESKF tuning is accepted on these findings.
