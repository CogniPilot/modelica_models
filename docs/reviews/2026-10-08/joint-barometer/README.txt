Joint pressure-bias estimation: optional ESKF checkpoint
=======================================================

useJointBarometerBias defaults to false. It estimates the pressure datum mean
and variance alongside the existing 15-element navigation cross covariance.
Every accepted aiding correction can update the datum through that correlation;
pressure fusion uses the shared uncertainty rather than treating it as fresh
independent noise. Both navigation Lie reset conventions transport the cross
covariance. The scalar datum is additive in world coordinates. The code uses
block vector expressions and reuses the innovation factorization; it does not
add a dense 16-by-16 state propagation or change the default correction policy.

Results and limits
------------------
Two completed 48-replay campaigns use the existing frozen physical captures,
GPS/denied/loss-return cases, two coupled seed/motion choices, two heights,
fusion-horizon and retrodiction timing. There are 48 candidate replays and 48
fresh controls. Every control state CSV reproduces its frozen reference byte
for byte. Native scores and covariance observers are frozen references, not
fresh native replays. These are not additional independent or held-out flights.

Against default startup calibration, vertical flight RMS improves in all
twelve conditions: 37.1-61.6 percent with the horizon and 37.3-63.1 percent with
retrodiction. Against declared-rest calibration alone, the improvements are
13.3-39.9 percent and 7.1-41.5 percent respectively. With declared rest and
joint estimation, both timing methods have lower vertical RMS than both frozen
native cores in all twelve cases, but lower horizontal RMS in only ten.

Horizontal error regresses in several paired comparisons. The largest change
against default calibration is +37.74 percent for the horizon and +85.85 percent
for retrodiction; against rest calibration it is +5.16 and +84.08 percent.
Velocity improves in all twelve paired cases for each configuration, while
attitude and yaw changes are mixed. comparison.csv and paired-flight.csv retain
all cases and outage/return windows, including full 15D NEES. summary.json
includes win counts and undefined-window disclosures without dropping failures.
All reported NEES windows are defined.

versus-consider.txt compares these candidate scores with the prior frozen
consider-state candidates. Input, arrival and reference hashes match. Joint
estimation improves vertical RMS in all twelve default-startup comparisons,
but only ten horizon and nine retrodiction rest-startup comparisons. This is
reuse of completed studies, not another replay campaign or a uniform win.

The joint correction costs more CPU time. Median horizon pipeline time is
13.7-14.1 microseconds per IMU packet versus 11.2-11.3 for paired controls;
retrodiction is 7.4-7.5 versus 5.6-5.7. These thread-CPU measurements include
startup and are not embedded worst-case timing. The option remains disabled
pending held-out validation and review of the accuracy/cost tradeoffs.

Effective native R/Q, priors, height-source and magnetic policies remain
unequal. Native per-sensor NIS, richer independent motion captures, complete
Modelica ports and stronger implementation proofs remain unfinished. This
checkpoint does not establish universal superiority over EKF2 or EKF3, nor
does it demonstrate a new Lie-group pre-integration advantage.

Validation
----------
The generated-C oracle checks full augmented Joseph updates with correlated
noise, constrained navigation gains, finite Lie resets, both covariance modes,
accepted/rejected updates and randomized nonzero datum means. Each seed covers
384 randomized configurations plus 1200 repeated pressure and 86 GPS updates.
The repeated batch-information oracle has zero residual so its tangent frame
stays fixed; it is not a nonlinear batch-estimation equivalence claim.

The actual generated RDD2 estimator passes aiding/rejection, scalar delayed
calibration, pressure withholding and joint-datum persistence probes. The old
consider-state oracle also passes both seeds. All 477120 scored full 16D
navigation/datum covariance matrices are positive definite without repair.
OpenModelica Tests.All checks and simulates successfully (5650 equations and
variables). Rumoca 0.10.2 DAE lowering, 34 Python tests, package structure and
Ruff pass. The existing consider report reproduces byte for byte after the
report tool gains an explicit joint-policy option.

JointPressureAudit.lean instantiates existing gnc_lean covariance theorems:
the optimal full augmented gain has a positive-semidefinite covariance
advantage over a zero-datum-row gain, including a common reset congruence.
Its explicit hypotheses require a valid linearized joint model and the
unconstrained optimal full gain. Heading/trust constrained navigation gains
need not satisfy that optimum, so this does not prove that this entire ESKF
implementation dominates the consider implementation or native estimators.
The arbitrary-gain Joseph PSD theorem is audited separately. The selected
imports use the previously audited artifacts; source and artifact hashes match
estimator-theory-lean-final.json. This is not a full gnc_lean CI success claim.

The full composed Modelica horizon still reaches Rumoca ED020 at the existing
HorizonEstimator.mo:259:5 read. Its reviewed clock identity changes from 589
to 590. The benchmark composes the generated filter, predictor and queue in
the C adapter. The RDD2 mission CI memory/cancellation issue remains open;
passing the selected checks does not mean all qualification jobs have passed.

Reproduction
------------
Keep generated C, objects, raw covariance CSVs and disposable outputs under
$HOME/scratch/modelica_models/joint-barometer. Build with Rumoca 0.10.2 and
GCC 15.2.0 -O2 using tools/estimator_comparison/build.py --eskf-only --horizon
--equivariant-magnetometer --geometric-alignment. Add --joint-barometer-bias
for candidates; add --declared-rest-barometer for both rest variants. Runtime
flags in replay.c select these parameters using the same generated objects.

Use compare_barometer_datum.py with the stable-release native reference,
captures and transport executable documented in
docs/reviews/2026-10-07/native-releases. The rest controls additionally
use --control-datum-reference ../barometer-datum/frozen-replays.json. Supply
--scope describing joint estimation and fresh owned work/output paths.

Run report_barometer_consider.py with --candidate-policy joint, the two frozen
studies, native-exposure.json and native-covariance.json. The covariance audit
check_barometer_joint_covariance.py additionally takes both retained work
directories. Kernel JSONs, generated-C logs, proof log, compact metrics and
source/evidence hashes are retained here; large build and trace artifacts
remain on scratch. tools/ci.py runs both independent datum oracles for seeds
20261008 and 911 and the joint actual-estimator persistence probe.
