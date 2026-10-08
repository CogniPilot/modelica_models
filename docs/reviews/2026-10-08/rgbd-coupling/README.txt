Canonical source migration and Modelica visual coupling gates
8 October 2026

179 classes from 74 pinned slam_web source files were relocated into normal
Modelica packages. migration-audit.json checks every source hash, class token
stream and import target; arithmetic is preserved. Added Fusion primitives
are independent CogniPilot implementations, not translated native firmware.

qualification.json identifies the actual source and generated artifact hashes
for fresh Rumoca 0.10.2 exports, C builds and independent numerical checks.
All three component qualifications passed:
  - 2160 actual ESKF injections at 36 orientations validate the tangent bridge,
    bias permutation and full21 covariance/NEES basis invariance.
  - 2363 generated camera observations validate calibrated RGB-D prediction,
    navigation/landmark Jacobians, lever arms and invalid-geometry refusal.
  - Both Modelica loose and tight updates accept all 64 matched fixed-map
    cases. Maximum state difference is 1.20e-7; full covariance difference is
    1.252e-9. The independent SE_2(3) reset/correction oracle agrees to 6.14e-7
    for physical state/rotation entries and 1.20e-9 for covariance entries.

Behind-camera and indefinite-noise cases are refused by both paths and retain
the prior. Repeated identical landmarks make the full six-dimensional pose
compression singular: loose refuses it, while the raw update can retain partial
information. This is a limitation of this full-pose compression, not a proof
that every loosely coupled method must discard partial constraints.

An additional double-precision linearized comparison uses 20 features and
1024 independent prior/noise draws. With the full sufficient pose statistic,
raw and compressed means/covariances agree numerically. Mean state NEES is
15.0403, raw 60D NIS is 59.7032 and compressed 6D NIS is 6.03674. The raw NIS
also includes the orthogonal 54D residual energy; these dimensions must not be
compared as if interchangeable. Dropping pose covariance cross terms breaks
the correction equivalence.

Six additional GNC Lean theorems freshly check the exact linearized information
and score identities, common posterior correction, sufficient pose statistic
and preservation of unobserved directions. The source here is byte-identical
to GNC.Estimation.VisualCompression in GNC commit b134d3a. Every theorem's
axioms were audited; only propext, Classical.choice and Quot.sound occur.
This proves algebraic information preservation under the stated common model;
it does not prove nonlinear/global equivalence, map correlation correctness,
floating-point refinement or estimator superiority. It is included in
https://github.com/CogniPilot/gnc_lean/pull/2.

These are fixed KNOWN map, single-update gates. They do not qualify persistent
SLAM, shared IMU/GPS information, uncertain retained maps, fusion delay, loop
closure, flight RMS/recovery, runtime efficiency or superiority over EKF2/3.
Imported full-graph source availability does not establish its execution on
the pinned Rumoca 0.10.2 runtime. See docs/slam-models.txt for reproduction.
