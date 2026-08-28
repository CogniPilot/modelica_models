within Tests;

model HorizonInterfaceTests
  "The output predictor tracks a dead-reckoning reference at the inertial rate
   across a horizon correction taken ON a release boundary"

  constant Real pi = 3.1415926535897932;
  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Real fusionPeriod_s = 0.01 "100 Hz fusion release";
  constant Real fusionHorizon_s = 0.05
    "Five buffered release windows. Shorter than the 200 ms flight horizon on
     purpose: none of the properties under test depend on the length, and a
     shorter buffer reaches its steady state inside a simulation the assertion
     suite can afford to run.";
  constant Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  constant Integer historyDepth = 64
    "Reference poses kept, indexed by age in inertial ticks. The fusion instant
     is at most horizonWindows * deltasPerFusion + deltasPerFusion + 1 = 49
     ticks behind now at these rates, so 64 covers it with margin.";
  constant Real correctionTime_s = 0.25
    "ON a release boundary, deliberately. 0.25 is an exact multiple of
     fusionPeriod_s, so the tick that carries the correction is the same tick
     that adopts a window, releases one, and advances the ring tail, while the
     horizon pose it is handed is still the epoch BEFORE that release. That is
     the tick on which a re-base folded over the advanced tail composes the
     wrong window onto the right pose, and the whole point of taking the
     correction here is that the harness sees it.";
  constant Real correctionStep_m[3] = {0.4, -0.25, 0.1}
    "Position only. A pure position shift of the horizon pose propagates
     additively through composePose -- velocity and attitude are untouched and
     the position shift is carried unchanged -- so the expected answer at now
     is the reference plus this vector, exactly, with no closed form for the
     motion needed.";
  constant Real settled_s = 0.08
    "Past the horizon fill and the first release";

  Real elapsed_s(start = 0.0, fixed = true)
    "Continuous anchor. A model assembled only from clocked blocks has no
     continuous equation at all and OpenModelica index reduction refuses to
     build one. Not under test.";

  // ---- the inertial stream -----------------------------------------------
  // Coning-rich and sculling-rich, and that is load-bearing. Under a
  // body-constant stream every buffered window is the SAME group element, so
  // folding the wrong contiguous run of ring slots produces the right answer
  // and a slot-identity error is algebraically invisible. A rotating angular
  // velocity with a steady yaw rate makes composition ORDER matter, and three
  // incommensurate force frequencies make every window differ from every
  // other, so which slots are folded matters too. Same stream as
  // Tests.HorizonChecks.syntheticImu and tools/wcet/driver_horizon.c.
  Real angularVelocityStream_rad_s[3];
  Real specificForceStream_m_s2[3];

  // ---- the predictors -----------------------------------------------------
  // The reference never re-bases: its horizon state is never valid, so it dead
  // reckons the stream from the seed pose by one composition per tick. It is
  // the truth this harness compares against, and it is truth by construction
  // rather than by a closed form, which is what lets the stream be arbitrary.
  Estimation.FusionHorizon.OutputPredictor reference(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);
  // The predictor under test: one correction, on a release boundary.
  Estimation.FusionHorizon.OutputPredictor driven(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);
  // A second instance on the same boundary values. The horizon declares its
  // inputs and reads nothing else, so two instances handed the same boundary
  // must agree exactly. This catches a future change that reached for the
  // clock, a global, or a filter-specific signal to decide anything.
  Estimation.FusionHorizon.OutputPredictor mirror(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);
  // THE EQUIVALENCE PROBE. A perfect filter that accepts a correction on every
  // release and moves nothing: it is handed the exact pose the reference had
  // at the fusion instant, so the re-base has no shift to apply. Re-basing on
  // every tick from the true epoch pose must then reproduce the incremental
  // answer to floating point, on release boundaries and off them alike. This
  // is the sharpest statement of the epoch invariant the harness can make: it
  // exercises the fold, the ring indices, the carried live window and the
  // state machine on every single tick, and it does not depend on the
  // correction being nonzero.
  Estimation.FusionHorizon.OutputPredictor probe(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);

  // Public rather than protected: these ARE the evidence, and a reader who
  // wants the margin rather than the pass or fail needs them on a plot.
  discrete Boolean correctionApplied(start = false, fixed = true);
  discrete Boolean correctionPulse(start = false, fixed = true);
  discrete Integer epochIndex(start = 1, fixed = true);
  discrete Real epochPose[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true);
  discrete Real trackingPositionError_m(start = 0.0, fixed = true);
  discrete Real trackingVelocityError_m_s(start = 0.0, fixed = true);
  discrete Real trackingAttitudeError(start = 0.0, fixed = true);
  discrete Real probePositionDisagreement_m(start = 0.0, fixed = true);
  discrete Real probeVelocityDisagreement_m_s(start = 0.0, fixed = true);
  discrete Real probeAttitudeDisagreement(start = 0.0, fixed = true);
  discrete Real mirrorPositionDisagreement_m(start = 0.0, fixed = true);
  discrete Real mirrorAttitudeDisagreement(start = 0.0, fixed = true);
  discrete Real predictorMotionPerTick_m(start = 1.0, fixed = true);
  discrete Integer drivenRebaseTicks(start = 0, fixed = true);
  discrete Integer referenceRebaseTicks(start = 0, fixed = true);
  discrete Real fusionEpochAge_s(start = 0.0, fixed = true);
  discrete Real epochAgainstBuffer_s(start = 0.0, fixed = true);

protected
  parameter Real historySeed[historyDepth, 10] =
    {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}
     for k in 1:historyDepth}
    "Every history slot starts at the seed pose, which is what the reference
     stood on before its first tick. The oldest slot the first release reads is
     exactly that one, so seeding the history at zeros would hand the probe a
     zero quaternion on the tick readiness latches.";
  discrete Real poseHistory[historyDepth, 10](
    start = historySeed, each fixed = true)
    "Row k is the reference pose published k inertial ticks ago";
  discrete Real previousHistory[historyDepth, 10](
    start = historySeed, each fixed = true);

algorithm
  // ---- the boundary the horizon is allowed to see -------------------------
  // A stand-in for whatever filter runs at the fusion instant. It publishes a
  // pose and a shift flag; that is the whole interface.
  //
  // The pose it publishes is the reference pose AT THE FUSION INSTANT, taken
  // out of the history by age. The age is the buffered delta count as of the
  // PREVIOUS tick plus one, because the horizon pose a tick is handed belongs
  // to the fusion instant reached by the previous release: the count published
  // on a tick already describes the epoch AFTER that tick's release, and the
  // filter has not reached it yet. Getting this index wrong by one tick makes
  // the probe below disagree, which is the point of writing it out.
  when sample(0.0, samplePeriod) then
    previousHistory := pre(poseHistory);
    // Half a tick of slack rather than an exact equality: the sampled instants
    // are k * samplePeriod in binary floating point and 0.25 need not be one
    // of them to the last bit. This selects the tick AT 0.25 and no other.
    correctionPulse := time >= correctionTime_s - 0.5 * samplePeriod
      and not pre(correctionApplied);
    correctionApplied := pre(correctionApplied)
      or time >= correctionTime_s - 0.5 * samplePeriod;
    epochIndex := min(historyDepth,
      max(1, pre(reference.bufferedDeltaCount) + 1));
    epochPose := previousHistory[epochIndex, :];
    poseHistory := cat(1,
      transpose(matrix(cat(1, reference.positionWorldEnu_m,
        reference.velocityWorldEnu_m_s, reference.quaternionWorldBody))),
      previousHistory[1:historyDepth - 1, :]);
  end when;

algorithm
  // ---- what the harness reads back ---------------------------------------
  // A separate section on purpose: an algorithm section is atomic, so driving
  // the predictors and reading them back from one section would be a cycle.
  // These are evaluated ON the tick, so the assertions below see the
  // tick-exact value rather than one held across the interval.
  when sample(0.0, samplePeriod) then
    trackingPositionError_m := max(abs(driven.positionWorldEnu_m
      - reference.positionWorldEnu_m
      - (if correctionApplied then correctionStep_m else zeros(3))));
    trackingVelocityError_m_s := max(abs(driven.velocityWorldEnu_m_s
      - reference.velocityWorldEnu_m_s));
    trackingAttitudeError := max(abs(driven.quaternionWorldBody
      - reference.quaternionWorldBody));
    probePositionDisagreement_m := max(abs(probe.positionWorldEnu_m
      - reference.positionWorldEnu_m));
    probeVelocityDisagreement_m_s := max(abs(probe.velocityWorldEnu_m_s
      - reference.velocityWorldEnu_m_s));
    probeAttitudeDisagreement := max(abs(probe.quaternionWorldBody
      - reference.quaternionWorldBody));
    mirrorPositionDisagreement_m := max(abs(driven.positionWorldEnu_m
      - mirror.positionWorldEnu_m));
    mirrorAttitudeDisagreement := max(abs(driven.quaternionWorldBody
      - mirror.quaternionWorldBody));
    predictorMotionPerTick_m := max(abs(driven.positionWorldEnu_m
      - pre(driven.positionWorldEnu_m)));
    drivenRebaseTicks := pre(drivenRebaseTicks)
      + (if driven.rebased then 1 else 0);
    referenceRebaseTicks := pre(referenceRebaseTicks)
      + (if reference.rebased then 1 else 0);
    fusionEpochAge_s := time - driven.horizonPacket.timestamp_s;
    epochAgainstBuffer_s := fusionEpochAge_s
      - driven.bufferedDeltaCount * samplePeriod;
  end when;

equation
  der(elapsed_s) = 1.0;

  angularVelocityStream_rad_s = {
    0.6 * sin(2.0 * pi * 30.0 * time),
    0.6 * cos(2.0 * pi * 30.0 * time),
    0.35};
  specificForceStream_m_s2 = {
    1.5 * sin(2.0 * pi * 17.0 * time),
    -1.1 * cos(2.0 * pi * 23.0 * time),
    9.81 + 0.8 * sin(2.0 * pi * 11.0 * time)};

  reference.reset = false;
  reference.angularVelocityMeasuredBodyFlu_rad_s = angularVelocityStream_rad_s;
  reference.specificForceMeasuredBodyFlu_m_s2 = specificForceStream_m_s2;
  reference.horizonStateValid = false;
  reference.horizonStateShifted = false;
  reference.horizonPositionWorldEnu_m = zeros(3);
  reference.horizonVelocityWorldEnu_m_s = zeros(3);
  reference.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  reference.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  reference.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  driven.reset = false;
  driven.angularVelocityMeasuredBodyFlu_rad_s = angularVelocityStream_rad_s;
  driven.specificForceMeasuredBodyFlu_m_s2 = specificForceStream_m_s2;
  driven.horizonStateValid = reference.horizonReady;
  driven.horizonStateShifted = correctionPulse;
  driven.horizonPositionWorldEnu_m = epochPose[1:3]
    + (if correctionApplied then correctionStep_m else zeros(3));
  driven.horizonVelocityWorldEnu_m_s = epochPose[4:6];
  driven.horizonQuaternionWorldBody = epochPose[7:10];
  driven.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  driven.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  mirror.reset = false;
  mirror.angularVelocityMeasuredBodyFlu_rad_s = angularVelocityStream_rad_s;
  mirror.specificForceMeasuredBodyFlu_m_s2 = specificForceStream_m_s2;
  mirror.horizonStateValid = reference.horizonReady;
  mirror.horizonStateShifted = correctionPulse;
  mirror.horizonPositionWorldEnu_m = epochPose[1:3]
    + (if correctionApplied then correctionStep_m else zeros(3));
  mirror.horizonVelocityWorldEnu_m_s = epochPose[4:6];
  mirror.horizonQuaternionWorldBody = epochPose[7:10];
  mirror.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  mirror.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  probe.reset = false;
  probe.angularVelocityMeasuredBodyFlu_rad_s = angularVelocityStream_rad_s;
  probe.specificForceMeasuredBodyFlu_m_s2 = specificForceStream_m_s2;
  probe.horizonStateValid = reference.horizonReady;
  probe.horizonStateShifted = reference.horizonReady;
  probe.horizonPositionWorldEnu_m = epochPose[1:3];
  probe.horizonVelocityWorldEnu_m_s = epochPose[4:6];
  probe.horizonQuaternionWorldBody = epochPose[7:10];
  probe.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  probe.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  // ---- the epoch invariant, on every tick ---------------------------------
  // A re-base from the exact epoch pose with nothing to shift must reproduce
  // the incremental answer. Any disagreement is a window that does not belong
  // to the pose it was composed onto: a ring index off by one entry, the live
  // window dropped or counted twice, or the fold reaching a slot this tick has
  // not written. Measured on this stream, the worst disagreement over the run
  // is 1.5e-17 m, 3.3e-16 m/s and 2.2e-16 on the quaternion, which is the last
  // bits of the quantities themselves. Reverting the epoch fix in step.mo
  // takes it to 1e-2 and this assertion is the one that reports it.
  assert(probePositionDisagreement_m < 1.0e-13,
    "Re-basing from the exact epoch pose moved the predicted position, so the
     folded window and the pose it was composed onto do not name the same
     fusion instant");
  assert(probeVelocityDisagreement_m_s < 1.0e-13,
    "Re-basing from the exact epoch pose moved the predicted velocity, so the
     folded window and the pose it was composed onto do not name the same
     fusion instant");
  assert(probeAttitudeDisagreement < 1.0e-13,
    "Re-basing from the exact epoch pose moved the predicted attitude, so the
     folded window and the pose it was composed onto do not name the same
     fusion instant");

  // ---- tracking, before and after the horizon moves -----------------------
  // The same bound holds on both sides of the correction. That is the claim
  // worth making: a correction at the horizon produces exactly the injected
  // shift at now and NOTHING else -- no transient, no decay, no gain. A
  // complementary-filter output predictor would fail this on the tick after
  // the correction and keep failing it for several time constants. Measured:
  // 8.3e-16 m, 6.2e-17 m/s, 2.2e-16 on the quaternion.
  assert(trackingPositionError_m < 1.0e-12 or time < settled_s,
    "The predicted state at now is not the dead-reckoned state plus exactly
     the injected position shift");
  assert(trackingVelocityError_m_s < 1.0e-13 or time < settled_s,
    "A position-only horizon correction moved the predicted velocity");
  assert(trackingAttitudeError < 1.0e-13 or time < settled_s,
    "A position-only horizon correction moved the predicted attitude");

  // ---- the horizon reads nothing but its declared interface ---------------
  assert(mirrorPositionDisagreement_m < 1.0e-13,
    "Two horizons handed identical boundary values disagree in position");
  assert(mirrorAttitudeDisagreement < 1.0e-13,
    "Two horizons handed identical boundary values disagree in attitude");


  // ---- one correction is one re-base --------------------------------------
  // The predictor must not inflate a single shift into a run of folds. The
  // seeding tick re-bases too -- there is no previous answer to compose onto
  // -- so a run with one correction is exactly two re-base ticks, and a run
  // with none is exactly one. This is the block-side half of the rule the
  // filter-side edge exists to keep: the WCET record's correction-rate ceiling
  // is per re-base, and the level trigger it replaced turned one correction
  // into deltasPerFusion of them. It also catches a re-anchor trigger that
  // fires with no bias move to justify it.
  assert(drivenRebaseTicks <= 2,
    "One horizon correction produced more than one re-base, so the fold rate
     is not the correction rate and the WCET budget is being spent more than
     once per correction");
  assert(referenceRebaseTicks <= 1,
    "A predictor that was never handed a valid horizon state re-based anyway");
  assert(drivenRebaseTicks >= 2 or time < correctionTime_s + samplePeriod,
    "The correction under test never reached the predictor");
  // ---- the rate structure -------------------------------------------------
  // Freshness: the predictor moves on EVERY inertial tick. A predictor that
  // only republished at the fusion rate would show a zero here on seven ticks
  // out of eight.
  assert(predictorMotionPerTick_m > 0.0 or time < settled_s,
    "The predicted state did not move on an inertial tick, so control is not
     seeing an inertial-rate state");
  // The epoch the filter is standing on is exactly the buffer's own span, to
  // the tick. This is the invariant that makes the re-base meaningful: compose
  // that many deltas onto that pose and you are at now, and an epoch that
  // drifted from the buffer by even one sample would silently bias every
  // correction.
  assert(abs(epochAgainstBuffer_s) < 0.5 * samplePeriod
      or not driven.horizonReady,
    "The fused epoch and the buffer span disagree, so the state the filter
     corrects is not the state the buffer carries forward");
  // And that span is at least one horizon and never more than one release
  // window beyond it.
  assert(fusionEpochAge_s >= fusionHorizon_s or not driven.horizonReady,
    "The fused epoch is younger than the declared fusion horizon");
  assert(fusionEpochAge_s <= fusionHorizon_s + fusionPeriod_s
      + 2.0 * samplePeriod or not driven.horizonReady,
    "The fused epoch fell more than one release window behind the horizon, so
     the fusion side stopped consuming");
  // Readiness means a packet has been HANDED OVER, not merely that the ring is
  // long enough. For one release window the ring already spans the horizon and
  // no packet exists, and the epoch is still its seed value; a consumer that
  // stood on readiness would be standing on that seed.
  assert(not driven.horizonReady or driven.horizonPacket.timestamp_s >= 0.0,
    "horizonReady is set while the packet epoch is still the pre-release seed
     value, so readiness does not mean a fusion instant exists");
  // No released packet may ever carry a zero span. The consumer divides the
  // rotation increment by it.
  assert(not driven.horizonPacket.valid
      or driven.horizonPacket.integrationTime_s
        > 0.5 * fusionPeriod_s,
    "A released packet carries less than half a release window of integration
     time, which is the zero-span path that reaches the filter as a division
     by zero");

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
    <p>Truth here is a second instance of the block under test, dead reckoning
    the same stream. That is deliberately not a closed form: a closed form
    forces a body-constant stream, and a body-constant stream makes every
    buffered window the same group element, which hides exactly the class of
    defect this model exists to catch.</p>
    </html>"));
end HorizonInterfaceTests;
