within Estimation.MultiSensorInvariant;

function inflateCovariance
  "Ramp position and velocity covariance toward the mission envelope"
  input Estimation.MultiSensorInvariant.Covariance covariance;
  input Estimation.MultiSensorInvariant.VarianceLimits limits;
  input Real dt(unit = "s");
  input Real timeConstant_s
    "e-folding time of the variance ramp; must be strictly positive";
  output Estimation.MultiSensorInvariant.Covariance inflated;
protected
  Real bound[TangentLength];
  Real diagonalAddend[TangentLength];
  Real stepFactor;
algorithm
  bound := cat(1,
    limits.position_m2,
    limits.velocity_m2_s2,
    limits.attitude_rad2,
    limits.gyroscopeBias_rad2_s2,
    limits.accelerometerBias_m2_s4);

  // A BOUNDED RAMP, not a jump to the envelope.
  //
  // The first version of this set the diagonal straight to the mission
  // envelope the moment the window elapsed. That converts gating into
  // adoption: with P_vv at the envelope (400 m2/s2) against a reported
  // R_vv of 0.09, the Kalman gain is 0.9998, so the very next sample --
  // whatever it says -- becomes the state. Measured consequences, both
  // strictly worse than taking no action at all: a cold-start filter at
  // v = 0 against a truthful 8-12 m/s stream acquired roughly
  // (+40, 0, -70) m/s of spurious velocity and +52 m of position error
  // where the unmodified filter reached -0.18 m; and the discrepancy at
  // which a lying sole optical-flow sensor gets adopted rather than
  // rejected fell from 3.63 m/s to 0.72 m/s.
  //
  // Ramping instead means the gate widens continuously, so the first
  // sample that passes does so at the SMALLEST covariance that admits it.
  // The gain at that moment is whatever the evidence actually justifies
  // rather than a saturated 1.0, the state moves by a correspondingly
  // bounded amount, and a measurement that is merely somewhat
  // inconsistent never sees a wide-open gate at all. The per-tick growth
  // is exp(2*dt/tau) on the variance, so at dt = 1 ms and tau = 0.5 s a
  // single tick can change any variance by at most a factor of 1.004:
  // the immediate post-inflation gain cannot swing the state outside the
  // pre-inflation gate, because the gate barely moved.
  //
  // stepFactor is the per-tick bound on the two-sided congruence factor,
  // so the variance bound is its square. Rebuilding it from dt each call
  // keeps the ramp rate a real time constant at any sample rate, exactly
  // as the windows that trigger it are real times.
  stepFactor := exp(dt / timeConstant_s);

  // Position and velocity only -- tangent indices 1:6. This is
  // load-bearing rather than a shortcut.
  //
  // Inflating attitude declares orientation unknown, and a filter that
  // believes that explains the next position or velocity residual as an
  // attitude error, because the measurement Jacobians couple them. With
  // no attitude-bearing source present -- GPS and optical flow observe
  // neither orientation nor anything that pins it down -- that spurious
  // rotation is never corrected and then corrupts every subsequent
  // body-frame residual. Measured: inflating all fifteen states sent the
  // velocity estimate to -35 m/s and +67 m/s against a 6 m/s truth during
  // re-acquisition. Wiping attitude through the covariance is the same
  // failure as wiping it through the state; only the mechanism differs.
  //
  // Position and velocity are also exactly the states the aiding sources
  // observe, so they are the only ones whose over-confidence can lock a
  // good fix out. Widening anything else buys no admissions.
  for i in 1:TangentLength loop
    // Affirmative guard: grow only a diagonal entry that is positive AND
    // still below the bound. A zero, negative or NaN variance yields a
    // zero addend rather than an infinite or NaN one, leaving the
    // degenerate matrix for the Cholesky guard in solveSPD to report.
    //
    // The ADDEND for this entry: what one ramp step would have added
    // multiplicatively, capped so the entry never passes the envelope.
    diagonalAddend[i] := if i <= 6 and covariance[i, i] > 0.0
        and covariance[i, i] < bound[i]
      then min((stepFactor * stepFactor - 1.0) * covariance[i, i],
               bound[i] - covariance[i, i])
      else 0.0;
  end for;

  // ADDITIVE diagonal inflation, not a two-sided congruence.
  //
  // D*P*D scales every off-diagonal by f_i*f_j, so it preserves every
  // correlation coefficient EXACTLY -- including near-degenerate ones. The
  // diagonal then saturates at the envelope while the innovation block
  // stays as ill-conditioned as it started, and a residual lying along the
  // small-eigenvalue direction is rejected no matter how large the
  // variances get. Measured at a 75 m displacement against truthful GPS:
  // S diagonal 10000.25 and 400.09 with cross term 1992.9, eigenvalues
  // 0.685 to 10399.7, condition 15180, Cholesky succeeding, and NIS 76.8
  // against a joint threshold of 36. Inflating harder could never fix that,
  // because the ill-conditioning was being inflated too.
  //
  // Adding to the diagonal instead leaves the off-diagonals untouched, so
  // correlation coefficients FALL as the variances rise and the smallest
  // eigenvalue grows with the ramp. It is also strictly cheaper: no 15x15
  // products at all.
  //
  // The amount added is exactly what the multiplicative ramp would have
  // added to each diagonal entry, so the growth RATE and therefore the
  // ramp timing are unchanged -- only the off-diagonal amplification is
  // dropped. No new constant is introduced.
  inflated := covariance;
  for i in 1:TangentLength loop
    inflated[i, i] := covariance[i, i] + diagonalAddend[i];
  end for;
end inflateCovariance;
