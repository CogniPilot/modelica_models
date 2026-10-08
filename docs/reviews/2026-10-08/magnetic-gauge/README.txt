Magnetic nullspace projection: formal identities and rejected development test
8 October 2026

Reject this Jacobian projection as a general replacement. All 24 known-seed
replays have higher attitude and yaw RMS than their matched controls. Velocity
RMS worsens in 20/24. The projection satisfies the exact-real geometric
properties checked below, but they do not establish better filter performance.
No Modelica algorithm, native core or vehicle default changed.

Hypothesis and experiment

For predicted body field p and measured field y, the current symmetric output
linearization is H = skew((p+y)/2). The candidate uses

  Hg = H * (I - p*p'/(p'*p)).

This annihilates p while retaining the action of H on vectors perpendicular
to p. Rotation about the predicted field cannot change the physical magnetic
vector. However, H is an endpoint-averaged finite-residual construction;
requiring its nullspace to equal that of the local physical derivative is
not, by itself, a valid optimality argument or proof that the existing H is
wrong. The experiment measures the resulting filter behavior separately.

The declared factorial crosses stationary IMU off/on, horizon/retrodiction,
joint/nonjoint pressure and all three GPS scenarios on the known seed-911
capture: 24 fresh candidate replays. Original controls come from the common
120 s readiness pilot; stationary controls come from ../stationary-readiness.
Physical sensors/noise, gates, rest information, FOH, delays and actual packet
delivery are unchanged within every pair. This is a development test using
an already inspected capture, not independent validation or a native benchmark.
The ongoing sixteen-capture stationary validation does not select this candidate.

generated.patch is the exact generated-C change. It captures p before Rumoca
reuses temporary storage and projects the final Jacobian. All other generated
source is byte-identical; source-qualification.json records that check.
The object is compiled with the recorded GCC 15.2 command and replaces only
the navigation object. build.json retains all eight commands/binary hashes.
The declared real Modelica counterpart is in declaration.json, but no modified
Modelica export or floating-point refinement proof was made for this prototype.

Measured results

Flight percent changes relative to the matching unprojected control:
metric                   lower/24  higher/24    minimum %   maximum %
horizontal position          11        13         -0.2094      +0.9608
vertical position            17         7         -0.1714      +0.0188
3D velocity                   4        20         -0.1645      +0.4941
attitude                      0        24         +1.8456      +5.5575
yaw                           0        24         +1.7664      +5.4837

Without stationary correction, denied horizontal RMS worsens in 4/4 variants
and yaw rises 3.26-3.59%. With stationary correction, denied horizontal RMS
improves slightly in 4/4, but velocity worsens in 4/4 and yaw rises 3.37-4.80%.
These losses rule out advancing this candidate as the proposed general
improvement. All 432 metric pairs across flight/outage/after-return windows
remain in paired-metrics.csv, including 3D position. pilot.json retains all
state coverage, common-15 consistency, timing and transition recovery results.
No new per-sensor NIS trace was collected for this rejected candidate.

All replay outputs pass finite-state, timestamp, step-status and actual packet
delivery checks. Every common-15 covariance window is valid. All twelve joint
cases also pass full-16 covariance checks on 59,640 actual float32 matrices
over 117-166.7 s, at each filter's state/fusion epoch. These matrices are exactly
symmetric and positive definite, with positive pressure Schur complements
and no symmetrization, jitter or repair. Raw covariance hashes are in summary.json.
Passing these checks does not reverse the measured accuracy regression.

Formal scope

MagneticGauge.lean imports GNC.Lie.Euclidean from cognipilot gnc_lean and proves
nine exact-real identities with Lean 4.29.1:

  projected_action: the complete row action of the projected Jacobian;
  projected_nullspace: Hg*p=0 for nonzero p'*p;
  projected_preserves_transverse_action: Hg*v=H*v when p'*v=0;
  projected_preserves_secant: a transverse exact residual identity survives;
  projected_zero_direction_information: (Hg*p)'*W*(Hg*p)=0 for any W;
  stationary_attitude_bias_ambiguity: the joint resting linear observation
    of a magnetic vector and gravity/bias has a cancelling attitude/bias mode;
  shortest_cayley_transverse: the shortest-rotation Cayley vector is transverse;
  symmetric_cayley_secant: for unit, non-antipodal p,y the symmetric Jacobian
    maps that Cayley vector exactly to y-p;
  projected_cayley_secant: the projected Jacobian retains that same identity.

The resting ambiguity uses angle=a*p and accelerometer-bias change
  -skew(gravity)*angle.
Both instantaneous linearized magnetic and gravity/bias residuals vanish.
This is not a full flight observability theorem: motion and other sensors can
add information, and gyro-bias observations are a separate part of the rest
model. It does not prove that this ambiguity causes every measured yaw error.

Every audited theorem uses only propext, Classical.choice and Quot.sound.
No sorryAx or added physical axiom is permitted. theory.json binds the source,
checker, compiled artifact and imported GNC geometry cache. theory.log is the
successful audit. Earlier secant-proof failures and the intermediate successful
build with a style warning are retained; final proofs compile without warnings.
geometry.json separately retains 1,000 numerical Rodrigues/Cayley checks.
Its noisy symmetric-Jacobian sensitivity includes finite angular error and
must not be interpreted as measurement-noise sensitivity alone.

These are algebraic identities, not proofs of nonlinear convergence, minimum
MSE, floating-point implementation correctness or superiority to EKF2/EKF3.
The rejected replay provides direct evidence against using those identities
alone to justify this algorithm change. Joint initialization/bias geometry
and shared-endpoint FOH covariance remain separate open investigations.

Reproduction and evidence

Keep large exports, objects and replay output under $HOME/scratch. pilot.py
preserves the measured development driver; runner.json binds its frozen source
snapshot and both reference pilots. To reproduce, copy this evidence into a
fresh owned scratch directory, restore the tools snapshot at afca041, apply
generated.patch to the SHA-bound original generated navigation source, and
reuse/rebuild the objects recorded in ../common-sensor-noise/replay-build.json.
The pilot expects the corresponding SHA-bound control captures and shared
generated objects in their recorded owned scratch locations. Do not execute
the replay driver in the source checkout or overwrite the completed experiment.
All large candidate/raw files remain in
  $HOME/scratch/modelica_models/magnetic-gauge.
No native PX4 or ArduPilot source is included in this review.

Recheck the proof using a read-only GNC/mathlib cache:
  python check_theory.py --cache GNC_CHECKOUT/.lake --lean PINNED_LEAN
    --build OWNED_FRESH_BUILD --output OWNED_FRESH_JSON
The checker derives library paths from that cache and writes only to the fresh
owned build/evidence paths. It rejects incomplete or nonstandard axiom audits.
evidence-sha256.json binds every copied review artifact.

The review patch uses zero context lines to preserve normal repository whitespace;
apply it with git apply --unidiff-zero or GNU patch. The original contextual
patch hash remains in source-qualification.json. Earlier failure-log trailing
whitespace is normalized; its unmodified scratch SHA is retained separately.
