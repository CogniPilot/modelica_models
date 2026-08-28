within Tests;

model HorizonPredictorTests
  "Algebraic identities the delayed fusion horizon and its predictor rest on"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Integer horizonEntries = 160 "200 ms of buffer at 800 Hz";
  constant Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  // The lattice the step-level arms below are run on. Deliberately the short
  // horizon of Tests.HorizonInterfaceTests rather than the flight one: the
  // ring has to FILL and RELEASE inside the run for the re-base path to be
  // reached at all, and at twenty windows that would take 169 ticks before the
  // first release.
  constant Integer stepTicks = 400 "Half a second at 800 Hz";
  constant Integer deltasPerFusion = 8;
  constant Integer horizonWindows = 5;

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

  // (c) With no correction the incremental predictor equals a re-base from
  // the exact epoch pose, THROUGH step.mo: the ring, the live-window carry,
  // the release bookkeeping and the state machine are all inside this one.
  final parameter Real incremental[3] =
    Tests.HorizonChecks.incrementalResidual(
      stepTicks, samplePeriod, deltasPerFusion, horizonWindows,
      gravityWorldEnu_m_s2);

  // (d) The Jacobian bias move stands in for re-integration at the new bias.
  // FOH paper Proposition 8, Sec. VI-A, whose bound is (T_D * ||db_g||)^2.
  constant Real gyroscopeBiasMove_rad_s[3] = {1.0e-3, -2.0e-3, 1.5e-3};
  constant Real accelerometerBiasMove_m_s2[3] = {2.0e-2, -1.0e-2, 3.0e-2};
  constant Real specificForceSupremum_m_s2 =
    sqrt(1.5 ^ 2 + 1.1 ^ 2 + (9.81 + 0.8) ^ 2)
    "Envelope of Tests.HorizonChecks.syntheticImu: the three force amplitudes
     taken together. The velocity and position bounds below are the attitude
     bound integrated against this, so they are derived from the same
     proposition and the same stream rather than picked to pass.";
  final parameter Real biasMove[3] = Tests.HorizonChecks.biasMoveResidual(
    horizonEntries, samplePeriod,
    gyroscopeBiasMove_rad_s, accelerometerBiasMove_m_s2);
  final parameter Real windowSpan_s = horizonEntries * samplePeriod;
  final parameter Real gyroscopeBiasMoveMagnitude_rad_s =
    sqrt(gyroscopeBiasMove_rad_s * gyroscopeBiasMove_rad_s);
  final parameter Real accelerometerBiasMoveMagnitude_m_s2 =
    sqrt(accelerometerBiasMove_m_s2 * accelerometerBiasMove_m_s2);
  final parameter Real biasMoveBound_rad =
    (windowSpan_s * gyroscopeBiasMoveMagnitude_rad_s) ^ 2
    "Prop. 8 as the paper states it: the attitude remainder of the first-order
     move, (T_D ||db_g||)^2. 2.90e-7 rad here.";
  final parameter Real biasMoveBound_m_s =
    specificForceSupremum_m_s2 * windowSpan_s ^ 3
      * gyroscopeBiasMoveMagnitude_rad_s ^ 2 / 6.0
    + 0.5 * windowSpan_s ^ 2 * gyroscopeBiasMoveMagnitude_rad_s
      * accelerometerBiasMoveMagnitude_m_s2
    "The velocity remainder has TWO terms and a bound written from the attitude
     statement alone misses the one that dominates. The first is the attitude
     remainder rotating the specific force under the integral,
     sup||a|| T^3 ||db_g||^2 / 6. The second is the cross term: the velocity
     increment is linear in db_a through -int R dt, so moving both biases
     leaves the product of the attitude error and the accelerometer move,
     T^2 ||db_g|| ||db_a|| / 2. At these offsets the two are 1.04e-7 and
     2.01e-6 m/s, so the cross term is twenty times the pure gyroscope term
     and the total bound is 2.12e-6 m/s.";
  final parameter Real biasMoveBound_m =
    specificForceSupremum_m_s2 * windowSpan_s ^ 4
      * gyroscopeBiasMoveMagnitude_rad_s ^ 2 / 24.0
    + windowSpan_s ^ 3 * gyroscopeBiasMoveMagnitude_rad_s
      * accelerometerBiasMoveMagnitude_m_s2 / 6.0
    "The same two terms integrated once more over the window: 1.40e-7 m.";

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

  // ---- (c) incremental predictor equals a re-base at the same epoch -------
  // Both arms run the real state machine over 400 ticks of the same stream,
  // one taking the incremental path on every tick and the other the re-base
  // path on every ready tick from the pose the first arm actually stood on at
  // the fusion instant. Any disagreement means the window folded and the pose
  // it was composed onto do not name the same instant. Measured over 400 ticks:
  // 1.4e-17 m, 3.6e-16 m/s, 2.2e-16 rad. Reverting the epoch fix in step.mo
  // takes it to 7.4e-4 m, 1.2e-2 m/s and 8.7e-4 rad.
  assert(incremental[1] < 1.0e-13,
    "Re-basing through step.mo from the exact epoch pose moves the predicted
     position, so the folded window and the pose disagree about the fusion
     instant");
  assert(incremental[2] < 1.0e-13,
    "Re-basing through step.mo from the exact epoch pose moves the predicted
     velocity, so the folded window and the pose disagree about the fusion
     instant");
  assert(incremental[3] < 1.0e-13,
    "Re-basing through step.mo from the exact epoch pose moves the predicted
     attitude, so the folded window and the pose disagree about the fusion
     instant");

  // ---- (d) the bias move is inside its own theorem's bound ----------------
  // Asserted against the BOUND rather than a hand-picked number, in all three
  // quantities. Measured: 1.39e-9 rad, 8.94e-7 m/s, 6.00e-8 m against bounds of
  // 2.90e-7, 2.12e-6 and 1.40e-7. The previous velocity and position limits
  // were 1.0e-5 and 1.0e-6, which are 4.7 and 7.2 times looser than the
  // theorem the design cites and would have passed a move outside it.
  //
  // A bound written from the attitude statement alone -- 0.5 sup||a|| T^3
  // ||db_g||^2, which is 3.12e-7 m/s here -- is BELOW the measurement, and the
  // reason is recorded on biasMoveBound_m_s: it leaves out the cross term
  // between the attitude error and the accelerometer bias move, which at these
  // offsets is twenty times the term it keeps.

  assert(biasMove[3] < biasMoveBound_rad,
    "Jacobian bias move exceeds the Proposition 8 second-order remainder bound
     in attitude");
  assert(biasMove[2] < biasMoveBound_m_s,
    "Jacobian bias move exceeds the Proposition 8 second-order remainder bound
     in velocity");
  assert(biasMove[1] < biasMoveBound_m,
    "Jacobian bias move exceeds the Proposition 8 second-order remainder bound
     in position");

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
