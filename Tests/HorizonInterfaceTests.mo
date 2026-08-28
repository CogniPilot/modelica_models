within Tests;

model HorizonInterfaceTests
  "The output predictor tracks truth at the inertial rate across a horizon
   correction, while the fusion window slides at the fusion rate"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Real fusionPeriod_s = 0.01 "100 Hz fusion release";
  constant Real fusionHorizon_s = 0.05
    "Five buffered release windows. Shorter than the 200 ms flight horizon on
     purpose: none of the properties under test depend on the length, and a
     shorter buffer reaches its steady state inside a simulation the assertion
     suite can afford to run.";
  constant Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  constant Real worldAcceleration_m_s2 = 0.5
    "Constant east acceleration. Truth is then p = a t^2 / 2 and v = a t in
     closed form, so the predictor is checked against arithmetic rather than
     against a second integrator that could share its mistakes.";
  constant Real specificForceBodyFlu_m_s2[3] =
    {worldAcceleration_m_s2, 0.0, 9.81}
    "Level attitude, so specific force is the world acceleration plus the
     reaction to gravity. At rest the gravity block of the left factor cancels
     it exactly, so any sign error in the time-block coupling shows up here as
     a quadratic position drift.";
  constant Real correctionTime_s = 0.2506
    "Deliberately NOT on a release boundary. The horizon pose is read back
     through pre(), so on a release tick the epoch it is built from would be
     one release stale; off a boundary the epoch is unchanged and the injected
     shift is the only thing that moves.";
  constant Real correctionStep_m[3] = {0.4, -0.25, 0.1};
  constant Real settled_s = 0.08
    "Past the horizon fill and the first release";

  Real elapsed_s(start = 0.0, fixed = true)
    "Continuous anchor. A model assembled only from clocked blocks has no
     continuous equation at all and OpenModelica index reduction refuses to
     build one. Not under test.";

  Estimation.FusionHorizon.OutputPredictor driven(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);
  // A second instance on the same boundary values. The horizon declares ten
  // inputs and reads nothing else, so two instances handed the same boundary
  // must agree exactly. This catches a future change that reached for the
  // clock, a global, or a filter-specific signal to decide anything.
  Estimation.FusionHorizon.OutputPredictor mirror(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);

  // Public rather than protected: these ARE the evidence, and a reader who
  // wants the margin rather than the pass or fail needs them on a plot.
  discrete Boolean correctionApplied(start = false, fixed = true);
  discrete Boolean correctionPulse(start = false, fixed = true);
  discrete Real horizonEpoch_s(start = 0.0, fixed = true);
  discrete Real horizonPosition_m[3](each start = 0.0, each fixed = true);
  discrete Real horizonVelocity_m_s[3](each start = 0.0, each fixed = true);
  discrete Real trackingPositionError_m(start = 0.0, fixed = true);
  discrete Real trackingVelocityError_m_s(start = 0.0, fixed = true);
  discrete Real mirrorPositionDisagreement_m(start = 0.0, fixed = true);
  discrete Real mirrorAttitudeDisagreement(start = 0.0, fixed = true);
  discrete Real predictorMotionPerTick_m(start = 1.0, fixed = true);
  discrete Real fusionEpochAge_s(start = 0.0, fixed = true);
  discrete Real epochAgainstBuffer_s(start = 0.0, fixed = true);

algorithm
  // ---- the boundary the horizon is allowed to see -------------------------
  // A stand-in for whatever filter runs at the fusion instant. It publishes a
  // pose and a shift flag; that is the whole interface. Its pose is truth
  // evaluated at the epoch the horizon itself reports, read through pre() so
  // that driving the predictor and reading it back is not a cycle.
  //
  // Truth carries a one-tick offset because the clause fires at time zero and
  // that first tick integrates the interval ENDING there: the state the block
  // publishes at t is the state at t plus one sample period.
  when sample(0.0, samplePeriod) then
    correctionPulse := time >= correctionTime_s and not pre(correctionApplied);
    correctionApplied := pre(correctionApplied) or time >= correctionTime_s;
    horizonEpoch_s := pre(driven.horizonPacket.timestamp_s) + samplePeriod;
    horizonPosition_m := {0.5 * worldAcceleration_m_s2
        * horizonEpoch_s * horizonEpoch_s, 0.0, 0.0}
      + (if correctionApplied then correctionStep_m else zeros(3));
    horizonVelocity_m_s :=
      {worldAcceleration_m_s2 * horizonEpoch_s, 0.0, 0.0};
  end when;

algorithm
  // ---- what the harness reads back ---------------------------------------
  // A separate section on purpose: an algorithm section is atomic, so driving
  // the predictor and reading it back from one section would be a cycle.
  // These are evaluated ON the tick, so the assertions below see the
  // tick-exact value rather than one held across the interval.
  when sample(0.0, samplePeriod) then
    trackingPositionError_m := max(abs(driven.positionWorldEnu_m
      - ({0.5 * worldAcceleration_m_s2 * (time + samplePeriod)
            * (time + samplePeriod), 0.0, 0.0}
        + (if correctionApplied then correctionStep_m else zeros(3)))));
    trackingVelocityError_m_s := max(abs(driven.velocityWorldEnu_m_s
      - {worldAcceleration_m_s2 * (time + samplePeriod), 0.0, 0.0}));
    mirrorPositionDisagreement_m := max(abs(driven.positionWorldEnu_m
      - mirror.positionWorldEnu_m));
    mirrorAttitudeDisagreement := max(abs(driven.quaternionWorldBody
      - mirror.quaternionWorldBody));
    predictorMotionPerTick_m := max(abs(driven.positionWorldEnu_m
      - pre(driven.positionWorldEnu_m)));
    fusionEpochAge_s := time - driven.horizonPacket.timestamp_s;
    epochAgainstBuffer_s := fusionEpochAge_s
      - driven.bufferedDeltaCount * samplePeriod;
  end when;

equation
  der(elapsed_s) = 1.0;

  driven.reset = false;
  driven.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
  driven.specificForceMeasuredBodyFlu_m_s2 = specificForceBodyFlu_m_s2;
  driven.horizonStateValid = driven.horizonReady;
  driven.horizonStateShifted = correctionPulse;
  driven.horizonPositionWorldEnu_m = horizonPosition_m;
  driven.horizonVelocityWorldEnu_m_s = horizonVelocity_m_s;
  driven.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  driven.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  driven.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  mirror.reset = false;
  mirror.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
  mirror.specificForceMeasuredBodyFlu_m_s2 = specificForceBodyFlu_m_s2;
  mirror.horizonStateValid = driven.horizonReady;
  mirror.horizonStateShifted = correctionPulse;
  mirror.horizonPositionWorldEnu_m = horizonPosition_m;
  mirror.horizonVelocityWorldEnu_m_s = horizonVelocity_m_s;
  mirror.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  mirror.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  mirror.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  // ---- tracking, before and after the horizon moves -----------------------
  // The same bound holds on both sides of the correction. That is the claim
  // worth making: a correction at the horizon produces exactly the injected
  // shift at now and NOTHING else -- no transient, no decay, no gain. A
  // complementary-filter output predictor would fail this on the tick after
  // the correction and keep failing it for several time constants.
  assert(trackingPositionError_m < 1.0e-6 or time < settled_s,
    "The predicted state at now left the closed-form truth in position");
  assert(trackingVelocityError_m_s < 1.0e-6 or time < settled_s,
    "The predicted state at now left the closed-form truth in velocity");

  // ---- the horizon reads nothing but its declared interface ---------------
  assert(mirrorPositionDisagreement_m < 1.0e-12,
    "Two horizons handed identical boundary values disagree in position");
  assert(mirrorAttitudeDisagreement < 1.0e-12,
    "Two horizons handed identical boundary values disagree in attitude");

  // ---- the rate structure -------------------------------------------------
  // Freshness: the predictor moves on EVERY inertial tick. Under a constant
  // acceleration the per-tick displacement is at least v*dt, so a predictor
  // that only republished at the fusion rate would show a zero here on seven
  // ticks out of eight.
  assert(predictorMotionPerTick_m > 0.0 or time < settled_s,
    "The predicted state did not move on an inertial tick, so control is not
     seeing an inertial-rate state");
  // The epoch the filter is standing on is exactly the buffer's own span, to
  // the tick. This is the invariant that makes the re-base meaningful: compose
  // that many deltas onto that pose and you are at now, and an epoch that
  // drifted from the buffer by even one sample would silently bias every
  // correction.
  assert(abs(epochAgainstBuffer_s) < 0.5 * samplePeriod or time < settled_s,
    "The fused epoch and the buffer span disagree, so the state the filter
     corrects is not the state the buffer carries forward");
  // And that span is at least one horizon and never more than one release
  // window beyond it.
  assert(fusionEpochAge_s >= fusionHorizon_s or time < settled_s,
    "The fused epoch is younger than the declared fusion horizon");
  assert(fusionEpochAge_s <= fusionHorizon_s + fusionPeriod_s
      + 2.0 * samplePeriod or time < settled_s,
    "The fused epoch fell more than one release window behind the horizon, so
     the fusion side stopped consuming");

  annotation(experiment(StartTime=0.0, StopTime=0.4,
    Tolerance=1.0e-8, Interval=0.001),
    Documentation(info="<html>
    <p>Simulated as a top-level model through
    <code>Tests/run-horizon.mos</code>.</p>
    <p>The filter itself is a stand-in here, not an ESKF or a UKF. That is a
    limitation of the tool, not of the design: OpenModelica cannot build a
    simulation containing a bare
    <code>Estimation.StrapdownINS.PartialEstimator</code> at all, and the
    existing <code>Tests.StrapdownEstimatorInterfaceTests</code> fails
    identically on an untouched tree, which is why it is compiled by Rumoca and
    never simulated. The real filters are carried through
    <code>Estimation.FusionHorizon.HorizonEstimator</code> in
    <code>Tests.HorizonEstimatorWiring</code>, on the same gate the corpus
    already uses for filter interchange.</p>
    </html>"));
end HorizonInterfaceTests;
