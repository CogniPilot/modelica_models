ESKF per-sensor innovation audit and magnetic development ablations
8 October 2026

The missing ESKF per-sensor NIS evidence is now available for all 96 ESKF
cases of the completed eight-capture matched campaign. Every observed replay
reproduces its frozen published states and its complete raw covariance CSV
byte for byte, including all pressure cross-covariances. This required 96
observer replays and 96 baseline covariance replays. No estimator defaults or
Modelica algorithm changed in this audit.

Independent float64 Cholesky reconstruction of r' S^-1 r passes for all
520,304 generated function evaluations. These represent 453,584 distinct
sensor/state/sample-epoch candidates. There are no invalid observations.
Rumoca repeats some pure GPS/barometer evaluations while assembling record
arguments in step; all 66,720 repeated evaluations match every recorded
field exactly. The raw hashes retain these calls, and summaries count each
identical candidate once. Conflicting evaluations invalidate the audit.
These repeats do not establish that the navigation state fused a packet twice.

The observer copies the actual residual, effective noise diagonal and complete
innovation covariance immediately before the dense SPD solve, then observes
NIS and outcome after the sensor correction. It rejects square-root mode.
State epoch means sample timestamp plus measurement age: fusion epoch for
horizon filtering, current filter epoch for retrodiction. It is not the
publication epoch. Prelinear rejections never become zero-NIS observations.
Accepted, rejected and all-candidate statistics remain separate.

Flight NIS

Values are medians of eight capture-level means of joint NIS divided by
measurement dimension. The eight conditions use only two independent sensor
noise seeds; varying height and motion does not create eight independent
noise realizations. Full results for both delay methods, joint/nonjoint
pressure and all four time windows are in summary.csv and records.json.

scenario    variant        GPS(6)  flow(2)  magnetic(3)  pressure(1)
GPS         horizon        0.981   1.006    0.991        0.757
GPS         horizon_joint  0.983   1.006    0.991        1.022
denied      horizon          --    0.999    0.991        0.756
denied      horizon_joint    --    0.999    0.991        1.015
loss/return horizon        0.969   1.004    0.991        0.757
loss/return horizon_joint  0.971   1.004    0.991        1.022

Flow and magnetic candidate NIS are close to their expected dimension.
This does not establish unbiased attitude, correct observability, optimal
state covariance or matching effective native Q/R. The joint pressure option
also brings pressure NIS closer to one; its previously measured vertical RMS
benefit remains in ../matched-campaign. A smaller NIS alone is not better.
Native sequential scalar NIS and ESKF joint vector NIS are different
statistics. Temporal correlation and gate selection preclude treating these
samples as independent chi-square trials.

Magnetic development ablations

Two separate, declared-before-replay pilots use the known seed-911 capture
with the same physical noise and arrival trace. Each includes all four ESKF
variants and all three GPS scenarios, for twelve fresh replays. They use
precisely recorded generated-C substitutions equivalent to the specified
Modelica expressions. They were not exported from modified Modelica and are
not production artifacts or untouched validation. Every common-15 covariance
window passes. Full 16D pressure covariance was not separately qualified for
these rejected candidates. Each directory retains the declaration, exact
replacement, build commands/hashes, replay driver, complete scores and paired
changes for flight, outage and after-return windows.

magnetic-jacobian:
Use wedge(predictedField) instead of wedge((predictedField+measuredField)/2).
The motivation was to remove measurement-noise dependence from H. Flight yaw
and attitude worsen in all twelve cases. Yaw RMS rises 2.12-5.56 percent;
denied horizontal and velocity RMS also worsen in all four variants.
Vertical RMS improves slightly. Reject as a general replacement.

magnetic-consider:
Keep the full symmetric vector residual, full S and navigation/bias gains;
project only the attitude gain onto world vertical, with headingOnly=false.
The arbitrary-gain Joseph covariance is retained. This isolates tilt gain
from the earlier scalar-heading ablation, which also suppressed other gains.
Flight yaw improves in all twelve cases. Denied yaw RMS falls 31.88-36.51
percent, but denied horizontal RMS rises 9.76-11.07 percent and velocity RMS
rises 11.01-12.37 percent. Velocity worsens in every GPS scenario/variant.
Loss/return horizontal RMS improves 4.74-4.89 percent, while GPS horizontal
RMS worsens 0.74-1.10 percent. This is a visible tradeoff, not general
superiority; retain the existing vector magnetic algorithm.

The evidence supports investigating the joint attitude/velocity/bias geometry
and exact shared-endpoint IMU noise propagation. Any resulting candidate must
retain these development losses, pass matched covariance and timing checks,
and then be tested on fresh independent captures. The GPS-denied position/yaw
gaps versus native EKF2/EKF3 remain in the matched comparison. Full native
effective Q/R equivalence, realistic sensor disturbances, complete tangent-
group EqF theory and target WCET remain open. No theoretical or practical
dominance claim follows from this audit.

Reproduction

Use the frozen source/binary/capture bindings in ../matched-campaign and
../common-sensor-noise/replay-build.json. All large exports and replay files
belong under $HOME/scratch/modelica_models. The public tools are:

  instrument_eskf_innovations.py SOURCE.c --output-source OWNED.c --output JSON
  replay_eskf_innovations.py --reference PILOT.json --noise-reference NOISE.json
    --captures CAPTURES --baseline-dir BASELINE --observed-dir OBSERVED
    --work FRESH_WORK --output FRESH_JSON
  eskf_innovations.py --innovations TRACE.csv --reference PILOT.json --output JSON
  report_eskf_innovations.py --qualification QUALIFICATION_ROOT
    --references FROZEN_CAMPAIGN_ROOT --output FRESH_REVIEW_DIRECTORY

NOISE.json is docs/reviews/2026-10-07/native-releases/native-exposure.json.
observer.json and build.json identify the exact instrumented generated source,
header, objects, compilation commands and baseline generated objects. Cases
contain complete qualification and reconstruction manifests. A complete raw
denied joint-horizon example is retained here and hash-bound to its case.
The remaining raw observations, states and covariance files are retained in
the owned scratch campaign; their hashes are in the qualification manifests.

Validation: 75 Python tests pass with no skips, including ten joint-NIS and
observer negative controls. Ruff passes. Hosted run 37755741761 at 64ca11e
passed Modelica regression. Its RDD2 job failed during manual-flight simulation
with a runner shutdown signal and operation cancellation. The log does not
establish the shutdown cause. CUBS2 mission qualification passed; the run
finished with failure because of RDD2.
The previously reproduced local Rumoca memory failure remains unresolved;
this audit does not claim a successful RDD2 mission qualification.
The copied CI log has trailing whitespace removed for repository formatting;
ci-rdd2-original-sha256.txt identifies the unmodified scratch log.
