within Tests;

model HorizonPredictorTests
  "Algebraic identities the delayed fusion horizon and its predictor rest on"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Integer horizonEntries = 160 "200 ms of buffer at 800 Hz";
  constant Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};

  // (a) Composing the buffer reproduces one accumulating integration pass.
  // FOH paper Lemma 5, Sec. IV-C: adjacent right factors multiply.
  final parameter Real compositionFirstOrderHold[4] =
    Tests.HorizonChecks.compositionResidual(
      horizonEntries, samplePeriod, true);
  final parameter Real compositionZeroOrderHold[4] =
    Tests.HorizonChecks.compositionResidual(
      horizonEntries, samplePeriod, false);
  // Half the sample period over the same span, to show the ONE quantity that
  // does not agree exactly disagrees at second order rather than by a bug.
  final parameter Real compositionRefined[4] =
    Tests.HorizonChecks.compositionResidual(
      2 * horizonEntries, 0.5 * samplePeriod, true);

  // (b) A shifted horizon pose reapplies the same buffered factors.
  // FOH paper Theorem 6, Sec. VI.
  final parameter Real rebase[3] = Tests.HorizonChecks.rebaseResidual(
    horizonEntries, samplePeriod, gravityWorldEnu_m_s2,
    {0.30, -0.20, 0.15, 0.05, 0.09, -0.04, 0.02, -0.03, 0.06});

  // (c) With no correction the incremental predictor equals a full recompute.
  final parameter Real incremental[3] =
    Tests.HorizonChecks.incrementalResidual(
      horizonEntries, samplePeriod, gravityWorldEnu_m_s2);

  // (d) The Jacobian bias move stands in for re-integration at the new bias.
  // FOH paper Proposition 8, Sec. VI-A, whose bound is (T_D * ||db_g||)^2.
  constant Real gyroscopeBiasMove_rad_s[3] = {1.0e-3, -2.0e-3, 1.5e-3};
  constant Real accelerometerBiasMove_m_s2[3] = {2.0e-2, -1.0e-2, 3.0e-2};
  final parameter Real biasMove[3] = Tests.HorizonChecks.biasMoveResidual(
    horizonEntries, samplePeriod,
    gyroscopeBiasMove_rad_s, accelerometerBiasMove_m_s2);
  final parameter Real windowSpan_s = horizonEntries * samplePeriod;
  final parameter Real biasMoveBound_rad = (windowSpan_s
    * sqrt(gyroscopeBiasMove_rad_s * gyroscopeBiasMove_rad_s)) ^ 2;

equation
  // ---- (a) the composition theorem, to floating point ---------------------
  // Measured on this stream: 1.3e-18 m, 1.3e-17 m/s, 5.6e-17 rad under the
  // first-order hold, and identically zero under the zero-order hold. The
  // limits below carry roughly three decades of margin over the measurement
  // and are still far under any quantity the estimator can resolve.
  assert(compositionFirstOrderHold[1] < 1.0e-15,
    "FIFO composition disagrees with direct integration in position");
  assert(compositionFirstOrderHold[2] < 1.0e-14,
    "FIFO composition disagrees with direct integration in velocity");
  assert(compositionFirstOrderHold[3] < 1.0e-14,
    "FIFO composition disagrees with direct integration in attitude");
  assert(compositionZeroOrderHold[1] < 1.0e-15,
    "Zero-order-hold composition disagrees in position");
  assert(compositionZeroOrderHold[2] < 1.0e-14,
    "Zero-order-hold composition disagrees in velocity");
  assert(compositionZeroOrderHold[3] < 1.0e-14,
    "Zero-order-hold composition disagrees in attitude");

  // The bias Jacobians are the one output that is NOT claimed to agree
  // exactly, and saying so is the point of this pair of assertions. The
  // accumulating pass linearizes each interval about the midpoint of the
  // running preintegral; the per-tick pass linearizes about the midpoint of
  // its own interval. Halving the sample period over the same 200 ms span cut
  // the disagreement from 1.73e-7 to 4.42e-8, a factor of 3.9, so it is second
  // order in the step and not a defect in the composition. A first-order
  // error, or a wrong chain rule, would fail the ratio test even while passing
  // the magnitude test.
  assert(compositionFirstOrderHold[4] < 1.0e-6,
    "Composed bias Jacobians are further from the accumulating pass than the
     second-order linearization difference explains");
  assert(compositionRefined[4] < 0.4 * compositionFirstOrderHold[4],
    "Bias Jacobian disagreement does not fall quadratically with the sample
     period, so it is not the midpoint linearization difference it is
     documented to be");

  // ---- (b) re-base after a horizon correction -----------------------------
  // Measured: 1.2e-13 m, 3.9e-15 m/s, 2.6e-15 rad over 160 compositions from a
  // pose the injection moved by 0.4 m and 0.07 rad. This is floating-point
  // accumulation over the fold, not a modeling error: the identity itself is
  // exact.
  assert(rebase[1] < 1.0e-11,
    "Re-base by one fold differs from stepwise composition in position");
  assert(rebase[2] < 1.0e-12,
    "Re-base by one fold differs from stepwise composition in velocity");
  assert(rebase[3] < 1.0e-12,
    "Re-base by one fold differs from stepwise composition in attitude");

  // ---- (c) incremental predictor equals a full recompute ------------------
  assert(incremental[1] < 1.0e-11,
    "Incremental prediction differs from a full recomposition in position");
  assert(incremental[2] < 1.0e-12,
    "Incremental prediction differs from a full recomposition in velocity");
  assert(incremental[3] < 1.0e-12,
    "Incremental prediction differs from a full recomposition in attitude");

  // ---- (d) the bias move is inside its own theorem's bound ----------------
  // Measured: 1.4e-9 rad of attitude against a Proposition 8 bound of 2.9e-7
  // for this bias offset and window. Asserting against the BOUND rather than a
  // hand-picked number is what makes this a check of the theorem the design
  // cites and not a regression lock on today's arithmetic.
  assert(biasMove[3] < biasMoveBound_rad,
    "Jacobian bias move exceeds the Proposition 8 second-order remainder bound");
  assert(biasMove[1] < 1.0e-6,
    "Jacobian bias move differs from re-integration in position");
  assert(biasMove[2] < 1.0e-5,
    "Jacobian bias move differs from re-integration in velocity");

  annotation(experiment(StartTime=0.0, StopTime=0.0,
    Tolerance=1.0e-10, Interval=1.0),
    Documentation(info="<html>
    <p>Simulated AS A TOP-LEVEL MODEL, not through <code>Tests.All</code>. Every
    variable here is constant-foldable, and when the model is instantiated as a
    component of <code>Tests.All</code> OpenModelica evaluates the whole thing
    away at translation, so none of these messages reach the generated code.
    <code>Tests/run-horizon.mos</code> is the entry point that actually gates
    the properties; this is the same argument recorded in
    <code>Tests/run-position-loop.mos</code>.</p>
    </html>"));
end HorizonPredictorTests;
