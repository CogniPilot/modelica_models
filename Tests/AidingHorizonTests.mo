within Tests;

model AidingHorizonTests
  "Aiding measurements reach the filter at the fusion instant their own
   timestamps name, in order, with a bounded residual, and nothing is lost"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Real fusionPeriod_s = 0.01 "100 Hz fusion release";
  constant Real fusionHorizon_s = 0.05
    "Five buffered release windows. Shorter than the flight horizon so the run
     is short; every property here is about the relation between a timestamp
     and the fusion instant and none of them depends on how many windows
     stand between them.";
  constant Real gpsPeriod_s = 0.02
    "Fifty hertz, faster than the deployed ten, so a short run still carries
     enough fixes for the ordering and conservation claims to mean something";
  constant Real gpsLatency_s = 0.03
    "Transport latency of the simulated fix: it is stamped this far in the
     past when it is presented. Chosen inside the horizon and NOT a multiple
     of the release period, so the residual this test measures is a genuine
     sub-window offset rather than zero by construction.";
  constant Real barometerPeriod_s = 0.02;
  constant Real barometerLatency_s = 0.04;
  constant Real settled_s = 0.12
    "Past the horizon fill, the first release, and the first delivery";
  constant Real stopTime_s = 0.4;

  // THE NUMBER THIS TEST EXISTS FOR. The live-edge path transports a
  // measurement Jacobian back over the age the packet arrived with; this path
  // transports it over the offset between the packet's timestamp and the
  // first fusion instant at or after it. The first is bounded by the sensor's
  // transport latency, the second by one release period, and the assertions
  // below measure both on the same stream.
  constant Real residualBound_s = fusionPeriod_s
    "Derived, not chosen: a measurement ripens on the first release at or
     after its own timestamp, and releases are fusionPeriod_s apart, so the
     offset is in [0, fusionPeriod_s).";

  Real elapsed_s(start = 0.0, fixed = true)
    "Continuous anchor. A model assembled only from clocked blocks has no
     continuous equation at all and OpenModelica index reduction refuses to
     build one. Not under test.";

  // The epoch and the release pulse come from a REAL output predictor rather
  // than from a synthetic clock. That is the point of the composition: the
  // queues and the delta ring have to agree about which instant the filter is
  // standing on, and a test that manufactured its own epoch would agree with
  // itself and prove nothing about the pair.
  Estimation.FusionHorizon.OutputPredictor predictor(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s);

  Estimation.FusionHorizon.AidingBuffer nominal(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gpsPeriod_s=gpsPeriod_s,
    barometerPeriod_s=barometerPeriod_s);

  // A source stamped OLDER than the horizon on arrival. There is no fusion
  // instant left to fuse it at, and the requirement is that it is refused by
  // name rather than transported to meet a state that has passed it.
  Estimation.FusionHorizon.AidingBuffer late(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gpsPeriod_s=gpsPeriod_s,
    barometerPeriod_s=barometerPeriod_s);

  // A source delivering ten times faster than it declares, so its queue fills
  // and has to displace entries. The requirement is that the displacement is
  // counted and that the source keeps delivering: an overflow must not blind
  // a sensor, which is what refusing the ARRIVAL instead would have done.
  Estimation.FusionHorizon.AidingBuffer crowded(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gpsPeriod_s=0.2,
    barometerPeriod_s=barometerPeriod_s)
    "Declares 5 Hz and delivers at 200 Hz, so its queue is four times shorter
     than the backlog its own rate builds while waiting for the horizon";

  discrete Real gpsTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean gpsPresent(start = false, fixed = true);
  discrete Real barometerTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean barometerPresent(start = false, fixed = true);
  discrete Real lateTimestamp_s(start = -1.0, fixed = true);
  discrete Real crowdedTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean crowdedPresent(start = false, fixed = true);

  discrete Real gpsCarried_m(start = 0.0, fixed = true)
    "A payload that identifies WHICH fix is delivered, carried as the fix's
     own timestamp scaled into a position so the two must agree. The queue
     checks that agreement itself, for the reason recorded in the
     documentation below.";
  discrete Real worstArrivalAge_s(start = 0.0, fixed = true)
    "The age a fix carried when it was PRESENTED, which is the interval the
     live-edge path would have transported its Jacobian over. Measured on the
     same stream as the residual, so the comparison is one run rather than
     two.";

algorithm
  when sample(0.0, samplePeriod) then
    // One fix per gpsPeriod_s, stamped gpsLatency_s in the past. Held between
    // pulses with a fixed timestamp, so the queue's novelty rule is what makes
    // it enter exactly once -- the same rule and the same waveform the
    // deployed pulsed GPS driver produces.
    gpsPresent := abs(time / gpsPeriod_s - floor(time / gpsPeriod_s + 0.5))
      < 1.0e-9;
    gpsTimestamp_s := if gpsPresent then time - gpsLatency_s
      else pre(gpsTimestamp_s);
    barometerPresent := abs(time / barometerPeriod_s
      - floor(time / barometerPeriod_s + 0.5)) < 1.0e-9;
    barometerTimestamp_s := if barometerPresent
      then time - barometerLatency_s else pre(barometerTimestamp_s);
    // Older than the horizon by a whole horizon, so it can never be ripe in
    // time however long the run is.
    lateTimestamp_s := if gpsPresent
      then time - 2.0 * fusionHorizon_s - fusionPeriod_s
      else pre(lateTimestamp_s);
    // Ten fixes per declared period, which is what makes the queue overflow.
    crowdedPresent := abs(time / 0.005 - floor(time / 0.005 + 0.5)) < 1.0e-9;
    crowdedTimestamp_s := if crowdedPresent then time - gpsLatency_s
      else pre(crowdedTimestamp_s);
    gpsCarried_m := gpsTimestamp_s * 100.0;

    // The only thing observed from out here is the stream this model DRIVES.
    // Everything about what the queues did with it is read from the queues'
    // own published signals; see the documentation below for why.
    worstArrivalAge_s := if gpsPresent
      then max(pre(worstArrivalAge_s), time - gpsTimestamp_s)
      else pre(worstArrivalAge_s);
  end when;

equation
  der(elapsed_s) = 1.0;

  predictor.reset = false;
  predictor.angularVelocityMeasuredBodyFlu_rad_s =
    {0.4 * sin(11.0 * time), 0.3 * cos(7.0 * time), 0.2 * sin(5.0 * time)};
  predictor.specificForceMeasuredBodyFlu_m_s2 =
    {0.6 * sin(9.0 * time), 0.5 * cos(13.0 * time),
     9.81 + 0.4 * sin(3.0 * time)};
  predictor.horizonStateValid = predictor.horizonReady;
  predictor.horizonStateShifted = false;
  predictor.horizonPositionWorldEnu_m = zeros(3);
  predictor.horizonVelocityWorldEnu_m_s = zeros(3);
  predictor.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  predictor.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  predictor.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  nominal.reset = false;
  nominal.horizonValid = predictor.horizonReady;
  nominal.horizonEpoch_s = predictor.horizonPacket.timestamp_s;
  nominal.horizonReleased = predictor.horizonPacket.valid;
  nominal.mocap.valid = false;
  nominal.mocap.fresh = false;
  nominal.mocap.timestamp_s = time;
  nominal.mocap.positionWorldEnu_m = zeros(3);
  nominal.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  nominal.mocap.positionCovarianceWorld_m2 = identity(3);
  nominal.mocap.attitudeCovarianceBody_rad2 = identity(3);
  nominal.gps.valid = gpsPresent;
  nominal.gps.fresh = gpsPresent;
  nominal.gps.positionValid = true;
  nominal.gps.velocityValid = false;
  nominal.gps.timestamp_s = gpsTimestamp_s;
  nominal.gps.geodetic_deg_m = zeros(3);
  nominal.gps.positionWorldEnu_m = {gpsCarried_m, 0.0, 0.0};
  nominal.gps.velocityWorldEnu_m_s = zeros(3);
  nominal.gps.positionCovarianceWorld_m2 = identity(3);
  nominal.gps.velocityCovarianceWorld_m2_s2 = identity(3);
  nominal.magnetometer.valid = false;
  nominal.magnetometer.fresh = false;
  nominal.magnetometer.timestamp_s = time;
  nominal.magnetometer.magneticFieldBodyFlu_T = zeros(3);
  nominal.magnetometer.covarianceBody_T2 = identity(3);
  nominal.barometer.valid = barometerPresent;
  nominal.barometer.fresh = barometerPresent;
  nominal.barometer.timestamp_s = barometerTimestamp_s;
  nominal.barometer.altitudeWorldEnu_m = 1.0;
  nominal.barometer.variance_m2 = 1.0;
  nominal.opticalFlow.valid = false;
  nominal.opticalFlow.fresh = false;
  nominal.opticalFlow.timestamp_s = time;
  nominal.opticalFlow.integratedLineOfSight_rad = zeros(2);
  nominal.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
  nominal.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
  nominal.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
  nominal.opticalFlow.integrationTime_s = 0.01;
  nominal.opticalFlow.groundDistance_m = 1.0;
  nominal.opticalFlow.groundDistanceVariance_m2 = 0.01;
  nominal.opticalFlow.quality = 1.0;

  late.reset = false;
  late.horizonValid = predictor.horizonReady;
  late.horizonEpoch_s = predictor.horizonPacket.timestamp_s;
  late.horizonReleased = predictor.horizonPacket.valid;
  // FIELD BY FIELD, never `late.mocap = nominal.mocap`. A whole-record
  // equality between two connectors is not something OpenModelica generates
  // code for: it publishes zeros on the target and reports nothing. Measured
  // while this model was written -- the barometer arm aliased that way was
  // never valid, so its queue admitted nothing for the whole run and every
  // assertion about it passed vacuously.
  late.mocap.valid = false;
  late.mocap.fresh = false;
  late.mocap.timestamp_s = time;
  late.mocap.positionWorldEnu_m = zeros(3);
  late.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  late.mocap.positionCovarianceWorld_m2 = identity(3);
  late.mocap.attitudeCovarianceBody_rad2 = identity(3);
  late.gps.valid = gpsPresent;
  late.gps.fresh = gpsPresent;
  late.gps.positionValid = true;
  late.gps.velocityValid = false;
  late.gps.timestamp_s = lateTimestamp_s;
  late.gps.geodetic_deg_m = zeros(3);
  late.gps.positionWorldEnu_m = zeros(3);
  late.gps.velocityWorldEnu_m_s = zeros(3);
  late.gps.positionCovarianceWorld_m2 = identity(3);
  late.gps.velocityCovarianceWorld_m2_s2 = identity(3);
  late.magnetometer.valid = false;
  late.magnetometer.fresh = false;
  late.magnetometer.timestamp_s = time;
  late.magnetometer.magneticFieldBodyFlu_T = zeros(3);
  late.magnetometer.covarianceBody_T2 = identity(3);
  // The late arm carries ONE source and every other stream is off, so its
  // delivered count is a statement about the late fixes alone. With a healthy
  // barometer beside them the count would be nonzero for a reason that has
  // nothing to do with what this arm is measuring.
  late.barometer.valid = false;
  late.barometer.fresh = false;
  late.barometer.timestamp_s = time;
  late.barometer.altitudeWorldEnu_m = 1.0;
  late.barometer.variance_m2 = 1.0;
  late.opticalFlow.valid = false;
  late.opticalFlow.fresh = false;
  late.opticalFlow.timestamp_s = time;
  late.opticalFlow.integratedLineOfSight_rad = zeros(2);
  late.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
  late.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
  late.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
  late.opticalFlow.integrationTime_s = 0.01;
  late.opticalFlow.groundDistance_m = 1.0;
  late.opticalFlow.groundDistanceVariance_m2 = 0.01;
  late.opticalFlow.quality = 1.0;

  crowded.reset = false;
  crowded.horizonValid = predictor.horizonReady;
  crowded.horizonEpoch_s = predictor.horizonPacket.timestamp_s;
  crowded.horizonReleased = predictor.horizonPacket.valid;
  crowded.mocap.valid = false;
  crowded.mocap.fresh = false;
  crowded.mocap.timestamp_s = time;
  crowded.mocap.positionWorldEnu_m = zeros(3);
  crowded.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  crowded.mocap.positionCovarianceWorld_m2 = identity(3);
  crowded.mocap.attitudeCovarianceBody_rad2 = identity(3);
  crowded.gps.valid = crowdedPresent;
  crowded.gps.fresh = crowdedPresent;
  crowded.gps.positionValid = true;
  crowded.gps.velocityValid = false;
  crowded.gps.timestamp_s = crowdedTimestamp_s;
  crowded.gps.geodetic_deg_m = zeros(3);
  crowded.gps.positionWorldEnu_m = zeros(3);
  crowded.gps.velocityWorldEnu_m_s = zeros(3);
  crowded.gps.positionCovarianceWorld_m2 = identity(3);
  crowded.gps.velocityCovarianceWorld_m2_s2 = identity(3);
  crowded.magnetometer.valid = false;
  crowded.magnetometer.fresh = false;
  crowded.magnetometer.timestamp_s = time;
  crowded.magnetometer.magneticFieldBodyFlu_T = zeros(3);
  crowded.magnetometer.covarianceBody_T2 = identity(3);
  crowded.barometer.valid = barometerPresent;
  crowded.barometer.fresh = barometerPresent;
  crowded.barometer.timestamp_s = barometerTimestamp_s;
  crowded.barometer.altitudeWorldEnu_m = 1.0;
  crowded.barometer.variance_m2 = 1.0;
  crowded.opticalFlow.valid = false;
  crowded.opticalFlow.fresh = false;
  crowded.opticalFlow.timestamp_s = time;
  crowded.opticalFlow.integratedLineOfSight_rad = zeros(2);
  crowded.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
  crowded.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
  crowded.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
  crowded.opticalFlow.integrationTime_s = 0.01;
  crowded.opticalFlow.groundDistance_m = 1.0;
  crowded.opticalFlow.groundDistanceVariance_m2 = 0.01;
  crowded.opticalFlow.quality = 1.0;

  // ---- 1. nothing is fused before it was measured -------------------------
  // The whole delayed-fusion contract in one line. A measurement handed to
  // the filter must be one the fusion instant has already REACHED; delivering
  // one ahead of the instant the filter stands on is handing it a measurement
  // from its own future, and that is exactly the state the merged horizon was
  // in before these queues existed, because the filter's epoch moved back and
  // the aiding did not move with it.
  assert(not nominal.deliveryAfterHorizon,
    "A measurement was delivered before the fusion instant reached its
     timestamp, so the filter would fuse a measurement from its own future");
  assert(not late.deliveryAfterHorizon,
    "The refusing queue delivered a measurement ahead of the fusion instant");
  assert(not crowded.deliveryAfterHorizon,
    "The overflowing queue delivered a measurement ahead of the fusion
     instant");

  // ---- 2. the residual is bounded by one release window -------------------
  // Measured: worst delivered residual 0.008750 s against the derived bound
  // of one fusionPeriod_s = 0.01 s, on a stream whose fixes ARRIVED 0.03 s
  // old -- a factor of 3.4 on this lattice.
  // At the flight lattice the same bound is 0.01 s against the 0.25 s the
  // live-edge path admits, which is the factor the cubic-Taylor transport
  // error of the retrodiction is charged against.
  assert(nominal.worstDeliveredAge_s <= residualBound_s + 1.0e-9,
    "A measurement was fused with a residual larger than one release window,
     so the sub-window transport the design rests on is not bounded by the
     release lattice");
  assert(crowded.worstDeliveredAge_s <= residualBound_s + 1.0e-9,
    "An overflowing queue delivered outside the residual bound");

  // ---- 3. the improvement is measured, not asserted in prose --------------
  assert(time < settled_s or worstArrivalAge_s >= gpsLatency_s - 1.0e-9,
    "The simulated fixes did not actually arrive aged, so this model is not
     measuring the delayed-aiding case it claims to");
  assert(worstArrivalAge_s >= nominal.worstDeliveredAge_s - 1.0e-9,
    "Fusing at the horizon transported the measurement Jacobian over a LONGER
     interval than fusing at the live edge would have, which inverts the
     entire argument for the horizon");

  // ---- 4. order -----------------------------------------------------------
  assert(not nominal.deliveryOutOfOrder,
    "Measurements were delivered out of timestamp order");
  assert(not crowded.deliveryOutOfOrder,
    "An overflowing queue delivered out of timestamp order, so refusing an
     arrival disturbed the order of what was already queued");

  // ---- 5. nothing is lost in the nominal case -----------------------------
  // Every source samples no faster than it declares and every fix is inside
  // the horizon, so no refusal of any kind may occur. This is the assertion
  // that fails if a queue silently drops what it cannot place.
  assert(nominal.refusedLateCount == 0,
    "A measurement inside the horizon was refused as later than it");
  assert(nominal.refusedOverflowCount == 0,
    "A queue sized for a whole horizon of its own source overflowed anyway");
  assert(nominal.droppedStaleCount == 0,
    "A queued measurement went stale, which cannot happen while the source
     samples no faster than the release rate");
  assert(not nominal.aidingRefused,
    "The nominal queue reported a refusal");
  assert(time < settled_s or nominal.beforeHorizonCount > 0,
    "No measurement was presented before the first release, so this run does
     not exercise the start-up window at all");
  // A queue that delivers NOTHING satisfies every assertion above. Say what
  // the run is required to have exercised.
  // Measured: 33 deliveries over 0.4 s across the GPS and barometer streams,
  // which is one per source per release from the first ripe measurement
  // onwards, and 6 samples presented before the horizon existed.
  assert(time < stopTime_s - 1.0e-9 or nominal.deliveredCount >= 10,
    "Too few measurements reached the filter for the delivery assertions above
     to mean anything");

  // ---- 6. a measurement older than the horizon is refused BY NAME ---------
  // Measured: 18 refusals and 0 deliveries over the run.
  assert(late.deliveredCount == 0,
    "A measurement the fusion instant had already passed was delivered anyway,
     so it would be fused against a state that has moved past it, which is the
     case fusing at a horizon exists to eliminate");
  assert(time < settled_s or late.refusedLateCount > 0,
    "A measurement older than the horizon was neither delivered nor refused,
     so it vanished with no outcome, which is the predicate exhaustion this
     boundary refuses");

  // ---- 7. overflow refuses, counts, and does not blind the source ---------
  // THE ASSERTION THAT CAUGHT THE POLICY. An overflow that displaced the
  // OLDEST entry made each queue a sliding window of the newest arrivals; on
  // an oversampled source every one of them is still in the filter's future
  // when the next arrival displaces it, so NOTHING ever ripens and the source
  // is silent for the whole flight while every arrival is dutifully stored --
  // and every ordering and residual assertion above still passes, because a
  // queue that delivers nothing violates none of them. Refusing the ARRIVAL
  // keeps the entries the fusion instant is about to reach, and the queue
  // degrades to delivering at the release rate rather than to delivering
  // nothing.
  // Measured with the arrival refused: 34 refusals, 47 deliveries, worst
  // residual 0.00875 s, and 2 entries discarded as stale -- the queue drains
  // and stays inside the residual bound even while it is too short for its
  // source. Measured with the OLDEST entry displaced instead: 0 deliveries.
  assert(time < settled_s or crowded.refusedOverflowCount > 0,
    "A source delivering forty times its declared rate did not overflow its
     queue, so the overflow path is not being exercised");
  assert(time < stopTime_s - 1.0e-9 or crowded.deliveredCount >= 10,
    "An overflowing queue stopped delivering, so one burst blinded the source
     for the rest of the flight. Refusing the ARRIVAL rather than displacing
     the oldest entry is what this assertion protects");

  annotation(experiment(StartTime=0.0, StopTime=0.4,
    Tolerance=1.0e-8, Interval=0.00125),
    Documentation(info="<html>
    <p>Simulated as a top-level model through
    <code>Tests/run-horizon.mos</code>. The epoch and the release pulse come
    from a real <code>Estimation.FusionHorizon.OutputPredictor</code> rather
    than a synthetic clock, because the property under test is that the
    queues and the delta ring agree about which instant the filter stands on,
    and a test that manufactured its own epoch would agree with itself.</p>
    <p>The filter itself is absent, and that is a tool limitation rather than
    a choice: OpenModelica cannot build a simulation containing a navigation
    estimator sub-component, which is recorded in
    <code>Tests.HorizonEstimatorWiring</code> and was re-measured for the
    composed horizon while this model was written -- a build of one
    <code>HorizonEstimator</code> was still running after ten minutes with no
    result. Every property here is a property of the aiding path up to the
    filter's connector, which is where the delayed-fusion contract lives.</p>
    <p><b>Why the ordering and epoch checks are published by the block rather
    than made here.</b> They were made here first, and they could not be
    trusted. Reading a sub-component's output-connector members from the parent
    does not report the written values under OpenModelica 1.27: a delivered
    GPS packet whose row inside the block held timestamp 0.03 and whose
    computed residual was correct read as <code>timestamp_s = 0</code> both in
    a parent when-clause and in the result file, while the Boolean
    <code>valid</code> on the same connector read correctly. The same model
    also failed to report a driven input connector's Boolean at all. Three
    distinct reporting failures, all silent, and each one makes an assertion
    pass rather than fail -- a queue whose delivered payload reads as zeros
    satisfies every check written about its contents.</p>
    <p>So the invariants are checked where the data is, inside
    <code>Estimation.FusionHorizon.AidingBuffer</code>, against the rows
    themselves, and published as <code>deliveryOutOfOrder</code> and
    <code>deliveryAfterHorizon</code>. That is weaker evidence than an external
    check and it is worth saying so: a block that computes its own invariant
    could be wrong in the way that hides it. It is also better supervision than
    the external form would have been, because these are properties of a
    SEQUENCE and a consumer that sees one packet at a time cannot reconstruct
    them. The production path is not affected by any of this -- 
    <code>Estimation.FusionHorizon.HorizonEstimator</code> consumes every
    aiding connector with a whole-record equation into the filter, which is the
    case that works.</p>
    </html>"));
end AidingHorizonTests;
