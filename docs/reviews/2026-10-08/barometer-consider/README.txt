Pressure datum correlation: optional ESKF improvement and frozen comparison
==========================================================================

The new useBarometerBiasConsider option defaults to false. It carries the
15-element navigation/datum cross covariance through prediction, every aiding
correction, both Lie reset conventions, covariance limits and reseeding.
Pressure corrections use the shared datum covariance rather than assimilating
its uncertainty independently on every packet and then imposing a height
variance floor. The datum mean remains fixed during navigation corrections.
This is a Schmidt filter, not a fully estimated pressure-bias state.

The default calibration policy still learns its original minimum startup
window. The separate declared-rest option learns through the declared initial
rest. In both consider variants the scalar calibration evaluates uncertainty
at the pressure timestamp and propagates the remaining random-walk variance
to the current epoch. No noise parameter was retuned for this experiment.

All 96 ESKF replays completed: two timing methods, two startup policies,
consider disabled/enabled, and twelve frozen conditions. The conditions cover
GPS, GPS-denied and GPS loss/return, two coupled seed/motion choices and two
heights. All 48 control state CSVs reproduce the published references byte for
byte. Physical captures and packet arrivals retain their frozen hashes.
Native scores come from the already published PX4 v1.17.0 and ArduPilot Copter
4.7.1 full-core replays; they were not rerun or retuned in this experiment.

With declared-rest calibration, consider reduces vertical flight RMS another
10.6--26.5% for the horizon and 5.4--34.3% for retrodiction. Both candidates
beat both frozen native baselines for vertical RMS in 12/12 conditions. Their
horizontal counts are only 10/12. Compared with their corresponding rest
controls, horizontal RMS worsens in 9/12 horizon and 10/12 retrodiction cases;
the largest increases are 9.0% and 28.5%. Without declared-rest calibration,
the largest horizontal regressions are 76.6% and 64.2%. These tradeoffs keep
the option disabled. comparison.csv and paired-flight.csv retain every case,
including GPS-return windows, attitude, yaw, velocity and descriptive NEES.

Every one of the 477,120 scored full 16D navigation/datum covariance matrices
passed Cholesky without repair. Joint covariance validity does not establish
empirical consistency or accuracy: the actual pressure drift is sinusoidal,
and the filter uses a random-walk model. The repeated-pressure unit example
also shows that recursive consideration need not attain full estimation's
optimum. derivation.txt explains the assumptions and the conditional proof.

Median filter CPU time increases approximately 2.5--5.0%, relative to fresh
controls from the same build. The replay pipeline includes prediction,
preintegration and queues. These thread-CPU measurements include startup;
they are not embedded worst-case timing. The horizon pipeline remains more
expensive than retrodiction in these captures. summary.json records costs,
state sizes, paired changes and all native win counts.

Effective native R/Q, priors, height sources and magnetic policies remain
unequal. Native per-sensor NIS, richer held-out motion, complete Modelica ports
and stronger implementation proofs remain unfinished. This experiment is not
a new Lie-group superiority result, a complete native-equivalence result, or
deployment qualification. The next useful comparison is joint pressure-bias
estimation against consideration, followed by held-out fault/transition data;
changing weights until these twelve cases pass would not prove superiority.

Validation
----------
OpenModelica Tests.All checked and simulated successfully. Rumoca 0.10.2
exported the real RDD2 estimator, FOH block, predictor and queue; the composed
adapter tests passed. Independent float64 augmented Joseph calculations check
the actual float32 generated C in dense/root modes, both Lie geometries,
constrained gains, correlated and independent noise, rejections, propagation,
variance limits, reseeding, repeated pressure and delayed datum diffusion.
An additional actual-estimator scalar oracle verifies delayed startup datum
learning, pressure withholding and correlation after release. All 34 existing
Python tests, structure checks and Ruff pass. The new generated-C probes are
included in tools/ci.py.

Full Modelica horizon composition still stops at the same Rumoca ED020 read,
HorizonEstimator.mo:259:5; its reviewed identity moves from 585 to 589 after
the new fields. This is not a working composed-block export. Prior main CI
run 37732868914 passed regression and CUBS2 but failed RDD2 during the manual
mission: the job reports cancellation and leaked semaphores. It does not
identify a host OOM cause. The broader CI memory issue remains unresolved.

Reproduction
------------
Use the physical captures, transport executable and stable-release runs
documented in docs/reviews/2026-10-07/native-releases. Put build outputs and
retained traces under $HOME/scratch/modelica_models/barometer-consider.

Build tools/estimator_comparison/build.py with --eskf-only --horizon
--equivariant-magnetometer --geometric-alignment for controls. Add
--barometer-bias-consider for candidates, and --declared-rest-barometer for
both variants of the rest comparison. All variants may reuse generated
objects; the C bridge flags set the actual public runtime parameters.

Run compare_barometer_datum.py on each pair. The rest controls additionally
use --control-datum-reference pointing to the previous barometer-datum
frozen-replays.json. That argument verifies all 24 rest control hashes rather
than silently accepting the default policy as their reference. Supply --scope
to describe the consider experiment. Use fresh owned work/output paths.

Run report_barometer_consider.py with the two completed studies and the native
state/covariance references. check_barometer_joint_covariance.py additionally
takes the retained work directories and checks all 96 traces. Raw traces and
generated code stay on scratch; their hashes and compact results are retained
here. Report reproduction is byte-identical. Negative controls reject missing
completion, duplicate cases, changed controls and changed references; an
undefined NEES row remains disclosed rather than being dropped.

PressureDatumAudit.lean imports the selected gnc_lean KalmanCorrection module
from PR #1 at commit 14e07e7, using the same cached proof artifacts audited in
estimator-theory-lean-final.json. It checks a 16D gain with a zero datum row
and instantiates the existing covariance theorem. This selected proof passed
locally. The separate full gnc_lean CI build was still running at validation;
no full-repository proof-build success is claimed.
