within Estimation.StrapdownINS;

package ESKF
  "15-state aided strapdown inertial-navigation error-state Kalman filter"
  constant Integer TangentLength = 15;
  constant Integer ProcessNoiseLength = 12;

  // The algorithm-independent Correction*, Source*, and Recovery* telemetry
  // codes live in StrapdownINS and are shared through PartialEstimator.
  // DECLARED RECOVERY ENVELOPE: 100 m of position displacement.
  //
  // Within 100 m, against a truthful aiding stream, the ladder is claimed to
  // reacquire and converge. Beyond it nothing is claimed except fail-closed
  // behaviour: the estimate stays finite, stays bounded, stays usable, and
  // never teleports onto a wrong position.
  //
  // 100 m is what BOTH independent harnesses now demonstrate, after the
  // additive-inflation repair in inflateCovariance. Before that repair the
  // two disagreed above 50 m -- one reacquired at 75 m and 100 m, the other
  // never reacquired at 75 m -- and the envelope would have been set at the
  // weaker figure, because an envelope is a promise to the vehicle and is
  // set by the worst credible evidence rather than the best. Fixing the
  // mechanism removed the disagreement instead of averaging over it.
  //
  // Measured after the repair: 75 m reacquires at 1.9 s at 10 Hz and 2.4 s
  // at 1.67 Hz, and 100 m at 3.0 s at 1 Hz, each converging to within 1 mm.
  // The declared 100 m still sits below the 1e4 m2 position envelope the
  // variance limits encode, so the claim stays inside the covariance the
  // filter is allowed to reach.
  //
  // Both sides are pinned by tools/estimator-witness/recovery_envelope.c:
  // inside asserts reacquisition and a convergence bound, outside asserts
  // only the fail-closed contract. Raising this number requires evidence
  // from BOTH harnesses, not one.
  constant Real RecoveryEnvelope_m = 100.0;

  // Magnitude above which a Real is treated as not finite. Exactly
  // representable in binary32 (max ~3.4e38) and binary64, so the same
  // source means the same thing in simulation and in generated flight
  // code, and 1e30 is ~20 orders of magnitude beyond any residual,
  // position, or velocity this filter can legitimately see. The test
  // `abs(x) < FiniteMagnitudeLimit` is false for NaN and for +/-Inf
  // alike, so one comparison per element covers both.
  constant Real FiniteMagnitudeLimit = 1.0e30;

  // Smallest quaternion norm accepted as an ATTITUDE SEED.
  //
  // Finiteness is not enough on the seeding path. A quaternion of all
  // zeros, or one scaled to near zero, is perfectly finite and passes
  // every magnitude check, but initialize() normalizes whatever it is
  // given: dividing by a vanishing norm turns representation noise into a
  // full-scale rotation, so the filter would start from a garbage attitude
  // that no aiding source can later contradict. Below 1e-3 the
  // normalization amplifies error by more than a thousandfold, which is
  // the point at which the value carries no attitude information at all.
  // A seed that cannot prove it is normalizable is treated exactly as an
  // absent one.
  constant Real MinimumSeedQuaternionNorm = 1.0e-3;

  // TRUST REGION on the attitude part of a single correction, in radians.
  //
  // The whole error-state formulation is a LINEARIZATION about the current
  // nominal attitude: the measurement Jacobians, the Joseph update and the
  // reset Jacobian are all first-order in the attitude error. That is
  // sound while the error is small and meaningless once it is not. A
  // correction is therefore capped at a rotation this filter can still
  // represent honestly, and anything larger is applied in installments
  // over successive ticks, each one re-linearized about the state the
  // previous one produced.
  //
  // This is not a tuning knob and is deliberately not in Tuning: it is a
  // property of the approximation, not of the mission. 0.1 rad is 5.7
  // degrees, an order of magnitude inside where the small-angle terms
  // start to matter, and three to four orders of magnitude above any
  // correction seen on a healthy tick -- so it never binds in nominal
  // flight and the nominal trace is bit-identical with it in place.
  //
  // Measured need: with a 1 kHz truthful GPS stream and a filter holding
  // v = 0 against an 8 m/s truth, the first correction the gate admitted
  // carried an attitude correction of about 1.9 rad. Applied in one step
  // it rotated the published quaternion by ~107 degrees, permanently --
  // after which gravity was resolved into the wrong axes, and vertical
  // velocity ran away to -53 m/s while position error reached +52 m. The
  // filter never recovered, because nothing observes attitude directly.
  //
  // 0.15 rad, and the exact value matters for two independent reasons.
  //
  // Branch boundary: the SE_2(3) and SO(3) exponential maps switch between
  // a small-angle series and the closed form at theta^2 = 1e-2, i.e. at
  // theta = 0.1 rad exactly. A clamp of 0.1 would park every limited
  // correction precisely on that boundary, where rounding decides which
  // formula runs and consecutive ticks can take different branches for the
  // same rotation. 0.15 rad gives theta^2 = 2.25e-2, comfortably on the
  // closed-form side.
  //
  // Installment size: the limit must not split corrections that one update
  // can legitimately apply whole. A directly observed 0.1 rad attitude
  // error -- an ordinary mocap disagreement -- is well inside the
  // linearization and should be taken in a single step; clamping it merely
  // delays convergence and makes a single-correction result depend on the
  // limit rather than on the measurement. 0.15 rad leaves such corrections
  // untouched while still bounding the ~1.9 rad request that destroys
  // attitude outright.
  //
  // At 8.6 degrees the neglected small-angle terms are of order
  // theta^2/6 ~ 0.4%, so the linearization the whole error-state
  // formulation rests on is still sound, and it remains three to four
  // orders of magnitude above any correction seen on a healthy tick.
  constant Real MaxAttitudeCorrection_rad = 0.15;

  type TangentVector = Real[15]
    "{position,velocity,attitude,gyro bias,accelerometer bias} error";
  type Covariance = Real[15, 15];
  annotation(Documentation(info = "<html>
    <p>The nominal extended pose is propagated by
    <code>LieGroups.SE23.Quat.exp_mixed</code>. Covariance lives in a
    body-local right-perturbation error ordered as position, velocity,
    attitude, gyroscope bias, and accelerometer bias.</p>
    <p>Over one IMU interval the corrected IMU input is held constant. The
    locally linearized error transition is discretized with a third-order matrix
    exponential polynomial; the process-noise integral is evaluated by
    Simpson quadrature. Both operations retain full cross-axis covariance and
    use matrix expressions rather than three scalar filters.</p>
    <p>This is an aided strapdown INS ESKF, not a strict invariant Kalman
    filter. The additive bias states are not part of a group-affine, exactly
    log-linear augmented system. The local pose error and manifold retraction
    are geometric implementation choices inside an otherwise ordinary
    error-state EKF.</p>
    <p>The convention and inertial error dynamics follow T. D. Barfoot,
    <a href=\"https://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser24.pdf\">
    <i>State Estimation for Robotics</i>, 2nd-edition draft</a>, Section 9.4:
    the nominal state is right-perturbed, so the local error is
    <code>T_hat^-1 T</code>. In the tangent ordering used here, Barfoot's
    position, velocity, attitude, gyro-bias, and accelerometer-bias blocks map
    directly to <code>continuousTransition</code> and
    <code>noiseInputMatrix</code>. Prediction implements
    <code>P+ = Phi P Phi' + integral(Phi G Q G' Phi')</code>. Correction uses
    an arbitrary-gain Joseph update followed by the right-Jacobian reset that
    transports covariance to the newly injected nominal-state tangent.</p>
  </html>"));
end ESKF;
