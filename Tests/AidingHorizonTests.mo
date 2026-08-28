within Tests;

model AidingHorizonTests
  "Aiding measurements reach the filter at the fusion instant their own
   timestamps name, in order, with a bounded residual, and nothing is lost"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Real fusionPeriod_s = 0.01 "100 Hz fusion release";

  // ---- the flight lattice, with the delays the plant actually applies ------
  // Not a scaled-down stand-in. The horizon, the source rates and the source
  // latencies are the ones Vehicles.Rdd2.WaypointVehicleSystem carries, and
  // those latencies are PX4's shipped EKF2 delay defaults, so a residual
  // measured here is the residual the deployed configuration produces rather
  // than one produced by a lattice chosen to make the run short.
  constant Real fusionHorizon_s = 0.2
    "PX4 EKF2_DELAY_MAX, default 200 ms: the delay between now and the
     delayed-time horizon. The same quantity and the same value.";
  constant Real gpsPeriod_s = 0.1 "10 Hz";
  constant Real gpsLatency_s = 0.11
    "PX4 EKF2_GPS_DELAY, v1.15.0 default 110 ms";
  constant Real opticalFlowPeriod_s = 0.01 "100 Hz";
  constant Real opticalFlowLatency_s = 0.02
    "PX4 EKF2_OF_DELAY, main default 20 ms";
  constant Real barometerPeriod_s = 0.02 "50 Hz";
  constant Real barometerLatency_s = 0.0
    "PX4 EKF2_BARO_DELAY, main default 0 ms: an onboard sensor on the
     autopilot's own clock has no transport to model. It is still DELAYED by
     the horizon, which is the point -- a zero-latency source waits the whole
     fusionHorizon_s before it ripens, and that path is exercised here.";
  constant Real declaredSourceDelay_s = gpsLatency_s
    "The slowest source these arms declare. GPS, and the horizon has to cover
     it with margin.";
  constant Real jitterMargin_s = 0.05;

  // ---- the boundary these two bracket -------------------------------------
  // The boundary in TRANSPORT LATENCY is the horizon itself, and working out
  // why took a failed assertion. A measurement is FUSED when it arrives still
  // ahead of the fusion instant, so that it ripens on a later release with an
  // age inside one window. A measurement arriving already ripe cannot be
  // delivered on the tick it arrives -- delivery reads the queue as it stood
  // before that tick's store -- so by the next release the instant has moved a
  // whole window past it. The deliverable condition is therefore a latency
  // below fusionHorizon_s, not below fusionHorizon_s + maximumResidualAge_s.
  //
  // Between the two lies a narrow band, 0.2 to 0.21 s here, where a packet is
  // admitted and then discarded as stale instead of refused as late. Both
  // outcomes are named and counted, and the horizon assertion keeps every
  // DECLARED source far below the band, so only a packet later than its own
  // source declares can reach it. It is recorded rather than tested because
  // what distinguishes the two is which name an anomaly is reported under.
  constant Real marginalLatency_s = fusionHorizon_s - 0.5 * fusionPeriod_s
    "0.195 s: arrives 0.005 s ahead of the fusion instant, half a release
     window inside the boundary. Must be DELIVERED";
  constant Real lateLatency_s = fusionHorizon_s + 1.5 * fusionPeriod_s
    "0.215 s: arrives 0.015 s past the fusion instant, beyond the residual
     bound. Must be REFUSED by name";

  constant Real settled_s = 0.45
    "Past the horizon fill, the first release, and the first GPS delivery";
  constant Real stopTime_s = 1.2;
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

  // Every arm below shrinks the mocap queue to its minimum by declaring a
  // slow mocap rate, which is the documented way to stop carrying a horizon of
  // slots for a stream a vehicle does not have. RDD2 has none.
  Estimation.FusionHorizon.AidingBuffer flight(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    mocapPeriod_s=1.0,
    gpsPeriod_s=gpsPeriod_s,
    magnetometerPeriod_s=1.0,
    barometerPeriod_s=barometerPeriod_s,
    opticalFlowPeriod_s=opticalFlowPeriod_s,
    maximumSourceDelay_s=declaredSourceDelay_s,
    horizonJitterMargin_s=jitterMargin_s);

  // Just INSIDE the refusal boundary. The declared delay stays legal; what is
  // near the boundary is the packet, not the configuration, which is the right
  // way round -- the assertion on the horizon governs what a deployment says
  // its sensors do, and the refusal path governs what a packet actually did.
  Estimation.FusionHorizon.AidingBuffer marginal(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    mocapPeriod_s=1.0,
    gpsPeriod_s=gpsPeriod_s,
    magnetometerPeriod_s=1.0,
    barometerPeriod_s=1.0,
    opticalFlowPeriod_s=1.0,
    maximumSourceDelay_s=declaredSourceDelay_s,
    horizonJitterMargin_s=jitterMargin_s);

  // Just OUTSIDE it.
  Estimation.FusionHorizon.AidingBuffer late(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    mocapPeriod_s=1.0,
    gpsPeriod_s=gpsPeriod_s,
    magnetometerPeriod_s=1.0,
    barometerPeriod_s=1.0,
    opticalFlowPeriod_s=1.0,
    maximumSourceDelay_s=declaredSourceDelay_s,
    horizonJitterMargin_s=jitterMargin_s);

  // A source delivering far faster than it declares, so its queue fills and
  // has to refuse. Kept on a SHORT horizon because the property is a mechanism
  // and not a rate, and a short horizon reaches it in a fraction of the run.
  constant Real crowdedHorizon_s = 0.05;
  Estimation.FusionHorizon.OutputPredictor crowdedPredictor(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=crowdedHorizon_s);
  Estimation.FusionHorizon.AidingBuffer crowded(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=crowdedHorizon_s,
    mocapPeriod_s=1.0,
    gpsPeriod_s=0.2,
    magnetometerPeriod_s=1.0,
    barometerPeriod_s=1.0,
    opticalFlowPeriod_s=1.0,
    maximumSourceDelay_s=0.03,
    horizonJitterMargin_s=0.01);

  // THE DEGENERATE-CASE ANCHOR. One release window of horizon, the shortest
  // the block admits, and measurements arriving with no latency at all. There
  // the delay a measurement experiences is at most one release period, which
  // is the delay the live-edge path imposes by sampling, so a queue at the
  // minimum horizon is indistinguishable from handing the measurement to the
  // filter on its next tick. If the queue machinery has a cost that does not
  // vanish as the horizon shrinks, this is the arm that shows it.
  Estimation.FusionHorizon.OutputPredictor degeneratePredictor(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionPeriod_s);
  Estimation.FusionHorizon.AidingBuffer degenerate(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionPeriod_s,
    mocapPeriod_s=1.0,
    gpsPeriod_s=0.02,
    magnetometerPeriod_s=1.0,
    barometerPeriod_s=1.0,
    opticalFlowPeriod_s=1.0,
    maximumSourceDelay_s=0.0,
    horizonJitterMargin_s=0.0);

  discrete Boolean gpsPresent(start = false, fixed = true);
  discrete Real gpsTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean barometerPresent(start = false, fixed = true);
  discrete Real barometerTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean opticalFlowPresent(start = false, fixed = true);
  discrete Real opticalFlowTimestamp_s(start = -1.0, fixed = true);
  discrete Real marginalTimestamp_s(start = -1.0, fixed = true);
  discrete Real lateTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean crowdedPresent(start = false, fixed = true);
  discrete Real crowdedTimestamp_s(start = -1.0, fixed = true);
  discrete Boolean degeneratePresent(start = false, fixed = true);

  discrete Real worstArrivalAge_s(start = 0.0, fixed = true)
    "The age a GPS fix carried when it was PRESENTED, which is the interval
     the live-edge path would have transported its Jacobian over. Measured on
     the same stream as the residual, so the comparison is one run rather than
     two.";

algorithm
  when sample(0.0, samplePeriod) then
    // Each source pulses at its own rate and is stamped its own latency in the
    // past. Held between pulses with a fixed timestamp, so the queue's novelty
    // rule is what makes it enter exactly once -- the same rule and the same
    // waveform the deployed pulsed GPS driver produces.
    gpsPresent := abs(time / gpsPeriod_s - floor(time / gpsPeriod_s + 0.5))
      < 1.0e-9;
    gpsTimestamp_s := if gpsPresent then time - gpsLatency_s
      else pre(gpsTimestamp_s);
    barometerPresent := abs(time / barometerPeriod_s
      - floor(time / barometerPeriod_s + 0.5)) < 1.0e-9;
    barometerTimestamp_s := if barometerPresent
      then time - barometerLatency_s else pre(barometerTimestamp_s);
    opticalFlowPresent := abs(time / opticalFlowPeriod_s
      - floor(time / opticalFlowPeriod_s + 0.5)) < 1.0e-9;
    opticalFlowTimestamp_s := if opticalFlowPresent
      then time - opticalFlowLatency_s else pre(opticalFlowTimestamp_s);
    marginalTimestamp_s := if gpsPresent then time - marginalLatency_s
      else pre(marginalTimestamp_s);
    lateTimestamp_s := if gpsPresent then time - lateLatency_s
      else pre(lateTimestamp_s);
    crowdedPresent := abs(time / 0.005 - floor(time / 0.005 + 0.5)) < 1.0e-9;
    crowdedTimestamp_s := if crowdedPresent then time - 0.03
      else pre(crowdedTimestamp_s);
    degeneratePresent := abs(time / 0.02 - floor(time / 0.02 + 0.5)) < 1.0e-9;

    // The only thing observed from out here is the stream this model DRIVES.
    // Everything about what the queues did with it is read from the queues'
    // own published signals; see the documentation below for why.
    // Measured over the whole run, not just the settled part: it is a property
    // of the stream rather than of the queue.
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

  crowdedPredictor.reset = false;
  crowdedPredictor.angularVelocityMeasuredBodyFlu_rad_s =
    predictor.angularVelocityMeasuredBodyFlu_rad_s;
  crowdedPredictor.specificForceMeasuredBodyFlu_m_s2 =
    predictor.specificForceMeasuredBodyFlu_m_s2;
  crowdedPredictor.horizonStateValid = crowdedPredictor.horizonReady;
  crowdedPredictor.horizonStateShifted = false;
  crowdedPredictor.horizonPositionWorldEnu_m = zeros(3);
  crowdedPredictor.horizonVelocityWorldEnu_m_s = zeros(3);
  crowdedPredictor.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  crowdedPredictor.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  crowdedPredictor.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  degeneratePredictor.reset = false;
  degeneratePredictor.angularVelocityMeasuredBodyFlu_rad_s =
    predictor.angularVelocityMeasuredBodyFlu_rad_s;
  degeneratePredictor.specificForceMeasuredBodyFlu_m_s2 =
    predictor.specificForceMeasuredBodyFlu_m_s2;
  degeneratePredictor.horizonStateValid = degeneratePredictor.horizonReady;
  degeneratePredictor.horizonStateShifted = false;
  degeneratePredictor.horizonPositionWorldEnu_m = zeros(3);
  degeneratePredictor.horizonVelocityWorldEnu_m_s = zeros(3);
  degeneratePredictor.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  degeneratePredictor.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  degeneratePredictor.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  // ---- the flight arm: three sources at their real rates and latencies -----
  // FIELD BY FIELD, never a whole-record equality between two connectors. That
  // is not something OpenModelica generates code for: it publishes zeros on
  // the target and reports nothing. Measured while this model was written --
  // an arm aliased that way was never valid, so its queue admitted nothing for
  // the whole run and every assertion about it passed vacuously.
  flight.reset = false;
  flight.horizonValid = predictor.horizonReady;
  flight.horizonEpoch_s = predictor.horizonPacket.timestamp_s;
  flight.horizonReleased = predictor.horizonPacket.valid;
  flight.mocap.valid = false;
  flight.mocap.fresh = false;
  flight.mocap.timestamp_s = time;
  flight.mocap.positionWorldEnu_m = zeros(3);
  flight.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  flight.mocap.positionCovarianceWorld_m2 = identity(3);
  flight.mocap.attitudeCovarianceBody_rad2 = identity(3);
  flight.gps.valid = gpsPresent;
  flight.gps.fresh = gpsPresent;
  flight.gps.positionValid = true;
  flight.gps.velocityValid = true;
  flight.gps.timestamp_s = gpsTimestamp_s;
  flight.gps.geodetic_deg_m = zeros(3);
  flight.gps.positionWorldEnu_m = zeros(3);
  flight.gps.velocityWorldEnu_m_s = zeros(3);
  flight.gps.positionCovarianceWorld_m2 = identity(3);
  flight.gps.velocityCovarianceWorld_m2_s2 = identity(3);
  flight.magnetometer.valid = false;
  flight.magnetometer.fresh = false;
  flight.magnetometer.timestamp_s = time;
  flight.magnetometer.magneticFieldBodyFlu_T = zeros(3);
  flight.magnetometer.covarianceBody_T2 = identity(3);
  flight.barometer.valid = barometerPresent;
  flight.barometer.fresh = barometerPresent;
  flight.barometer.timestamp_s = barometerTimestamp_s;
  flight.barometer.altitudeWorldEnu_m = 1.0;
  flight.barometer.variance_m2 = 1.0;
  flight.opticalFlow.valid = opticalFlowPresent;
  flight.opticalFlow.fresh = opticalFlowPresent;
  flight.opticalFlow.timestamp_s = opticalFlowTimestamp_s;
  flight.opticalFlow.integratedLineOfSight_rad = zeros(2);
  flight.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
  flight.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
  flight.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
  flight.opticalFlow.integrationTime_s = 0.01;
  flight.opticalFlow.groundDistance_m = 1.0;
  flight.opticalFlow.groundDistanceVariance_m2 = 0.01;
  flight.opticalFlow.quality = 1.0;

  marginal.reset = false;
  marginal.horizonValid = predictor.horizonReady;
  marginal.horizonEpoch_s = predictor.horizonPacket.timestamp_s;
  marginal.horizonReleased = predictor.horizonPacket.valid;
  marginal.mocap.valid = false;
  marginal.mocap.fresh = false;
  marginal.mocap.timestamp_s = time;
  marginal.mocap.positionWorldEnu_m = zeros(3);
  marginal.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  marginal.mocap.positionCovarianceWorld_m2 = identity(3);
  marginal.mocap.attitudeCovarianceBody_rad2 = identity(3);
  marginal.gps.valid = gpsPresent;
  marginal.gps.fresh = gpsPresent;
  marginal.gps.positionValid = true;
  marginal.gps.velocityValid = false;
  marginal.gps.timestamp_s = marginalTimestamp_s;
  marginal.gps.geodetic_deg_m = zeros(3);
  marginal.gps.positionWorldEnu_m = zeros(3);
  marginal.gps.velocityWorldEnu_m_s = zeros(3);
  marginal.gps.positionCovarianceWorld_m2 = identity(3);
  marginal.gps.velocityCovarianceWorld_m2_s2 = identity(3);
  marginal.magnetometer.valid = false;
  marginal.magnetometer.fresh = false;
  marginal.magnetometer.timestamp_s = time;
  marginal.magnetometer.magneticFieldBodyFlu_T = zeros(3);
  marginal.magnetometer.covarianceBody_T2 = identity(3);
  marginal.barometer.valid = false;
  marginal.barometer.fresh = false;
  marginal.barometer.timestamp_s = time;
  marginal.barometer.altitudeWorldEnu_m = 0.0;
  marginal.barometer.variance_m2 = 1.0;
  marginal.opticalFlow.valid = false;
  marginal.opticalFlow.fresh = false;
  marginal.opticalFlow.timestamp_s = time;
  marginal.opticalFlow.integratedLineOfSight_rad = zeros(2);
  marginal.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
  marginal.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
  marginal.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
  marginal.opticalFlow.integrationTime_s = 0.01;
  marginal.opticalFlow.groundDistance_m = 1.0;
  marginal.opticalFlow.groundDistanceVariance_m2 = 0.01;
  marginal.opticalFlow.quality = 1.0;

  late.reset = false;
  late.horizonValid = predictor.horizonReady;
  late.horizonEpoch_s = predictor.horizonPacket.timestamp_s;
  late.horizonReleased = predictor.horizonPacket.valid;
  late.mocap.valid = false;
  late.mocap.fresh = false;
  late.mocap.timestamp_s = time;
  late.mocap.positionWorldEnu_m = zeros(3);
  late.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  late.mocap.positionCovarianceWorld_m2 = identity(3);
  late.mocap.attitudeCovarianceBody_rad2 = identity(3);
  // The late arm carries ONE source and every other stream is off, so its
  // delivered count is a statement about the late fixes alone.
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
  late.barometer.valid = false;
  late.barometer.fresh = false;
  late.barometer.timestamp_s = time;
  late.barometer.altitudeWorldEnu_m = 0.0;
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
  crowded.horizonValid = crowdedPredictor.horizonReady;
  crowded.horizonEpoch_s = crowdedPredictor.horizonPacket.timestamp_s;
  crowded.horizonReleased = crowdedPredictor.horizonPacket.valid;
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
  crowded.barometer.valid = false;
  crowded.barometer.fresh = false;
  crowded.barometer.timestamp_s = time;
  crowded.barometer.altitudeWorldEnu_m = 0.0;
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

  degenerate.reset = false;
  degenerate.horizonValid = degeneratePredictor.horizonReady;
  degenerate.horizonEpoch_s = degeneratePredictor.horizonPacket.timestamp_s;
  degenerate.horizonReleased = degeneratePredictor.horizonPacket.valid;
  degenerate.mocap.valid = false;
  degenerate.mocap.fresh = false;
  degenerate.mocap.timestamp_s = time;
  degenerate.mocap.positionWorldEnu_m = zeros(3);
  degenerate.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  degenerate.mocap.positionCovarianceWorld_m2 = identity(3);
  degenerate.mocap.attitudeCovarianceBody_rad2 = identity(3);
  // ZERO LATENCY: stamped at the instant it is presented.
  degenerate.gps.valid = degeneratePresent;
  degenerate.gps.fresh = degeneratePresent;
  degenerate.gps.positionValid = true;
  degenerate.gps.velocityValid = false;
  degenerate.gps.timestamp_s = time;
  degenerate.gps.geodetic_deg_m = zeros(3);
  degenerate.gps.positionWorldEnu_m = zeros(3);
  degenerate.gps.velocityWorldEnu_m_s = zeros(3);
  degenerate.gps.positionCovarianceWorld_m2 = identity(3);
  degenerate.gps.velocityCovarianceWorld_m2_s2 = identity(3);
  degenerate.magnetometer.valid = false;
  degenerate.magnetometer.fresh = false;
  degenerate.magnetometer.timestamp_s = time;
  degenerate.magnetometer.magneticFieldBodyFlu_T = zeros(3);
  degenerate.magnetometer.covarianceBody_T2 = identity(3);
  degenerate.barometer.valid = false;
  degenerate.barometer.fresh = false;
  degenerate.barometer.timestamp_s = time;
  degenerate.barometer.altitudeWorldEnu_m = 0.0;
  degenerate.barometer.variance_m2 = 1.0;
  degenerate.opticalFlow.valid = false;
  degenerate.opticalFlow.fresh = false;
  degenerate.opticalFlow.timestamp_s = time;
  degenerate.opticalFlow.integratedLineOfSight_rad = zeros(2);
  degenerate.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
  degenerate.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
  degenerate.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
  degenerate.opticalFlow.integrationTime_s = 0.01;
  degenerate.opticalFlow.groundDistance_m = 1.0;
  degenerate.opticalFlow.groundDistanceVariance_m2 = 0.01;
  degenerate.opticalFlow.quality = 1.0;

  // ---- 1. nothing is fused before it was measured -------------------------
  // The whole delayed-fusion contract in one line. A measurement handed to
  // the filter must be one the fusion instant has already REACHED; delivering
  // one ahead of the instant the filter stands on is handing it a measurement
  // from its own future, and that is exactly the state the merged horizon was
  // in before these queues existed, because the filter's epoch moved back and
  // the aiding did not move with it.
  assert(not flight.deliveryAfterHorizon,
    "A measurement was delivered before the fusion instant reached its
     timestamp, so the filter would fuse a measurement from its own future");
  assert(not marginal.deliveryAfterHorizon,
    "The marginal arm delivered ahead of the fusion instant");
  assert(not late.deliveryAfterHorizon,
    "The refusing queue delivered a measurement ahead of the fusion instant");
  assert(not crowded.deliveryAfterHorizon,
    "The overflowing queue delivered ahead of the fusion instant");
  assert(not degenerate.deliveryAfterHorizon,
    "The degenerate case delivered ahead of its own fusion instant");

  // ---- 2. the residual is bounded by one release window -------------------
  // AT THE FLIGHT LATTICE AND THE FLIGHT LATENCIES. Measured: worst delivered
  // residual 0.008750 s against the derived bound of one fusionPeriod_s, on a
  // stream whose fixes ARRIVED 0.110 s old -- a factor of 12.6 on this
  // lattice, and the interval the transport error is cubic in. That ratio is the whole
  // quantitative claim: the interval the measurement Jacobian is transported
  // over falls from the sensor's transport latency to one release window, and
  // the transport error is cubic in it.
  assert(flight.worstDeliveredAge_s <= residualBound_s + 1.0e-9,
    "A measurement was fused with a residual larger than one release window,
     so the sub-window transport the design rests on is not bounded by the
     release lattice");
  assert(marginal.worstDeliveredAge_s <= residualBound_s + 1.0e-9,
    "The marginal arm delivered outside the residual bound");
  assert(crowded.worstDeliveredAge_s <= residualBound_s + 1.0e-9,
    "An overflowing queue delivered outside the residual bound");
  assert(degenerate.worstDeliveredAge_s <= residualBound_s + 1.0e-9,
    "The degenerate case delivered outside the residual bound, so the bound is
     not a property of the release lattice");

  // ---- 3. the improvement is measured, not asserted in prose --------------
  assert(time < settled_s or worstArrivalAge_s >= gpsLatency_s - 1.0e-9,
    "The simulated fixes did not actually arrive aged, so this model is not
     measuring the delayed-aiding case it claims to");
  assert(worstArrivalAge_s >= flight.worstDeliveredAge_s - 1.0e-9,
    "Fusing at the horizon transported the measurement Jacobian over a LONGER
     interval than fusing at the live edge would have, which inverts the
     entire argument for the horizon");

  // ---- 4. order -----------------------------------------------------------
  assert(not flight.deliveryOutOfOrder,
    "Measurements were delivered out of timestamp order");
  assert(not crowded.deliveryOutOfOrder,
    "An overflowing queue delivered out of timestamp order, so refusing an
     arrival disturbed the order of what was already queued");
  assert(not degenerate.deliveryOutOfOrder,
    "The degenerate case delivered out of timestamp order");

  // ---- 5. nothing is lost at the flight lattice ---------------------------
  // Every source samples no faster than it declares and every latency is
  // inside the horizon by the declared margin, so no refusal of any kind may
  // occur. This is the assertion that fails if a queue silently drops what it
  // cannot place, and it is the one that says the horizon is long enough for
  // the sensors the vehicle actually carries.
  assert(flight.refusedLateCount == 0,
    "A measurement inside the horizon was refused as later than it, so the
     horizon is not long enough for the declared sensor latencies");
  assert(flight.refusedOverflowCount == 0,
    "A queue sized for a whole horizon of its own source overflowed anyway");
  assert(flight.droppedStaleCount == 0,
    "A queued measurement went stale, which cannot happen while the source
     samples no faster than the release rate");
  assert(not flight.aidingRefused,
    "The flight arm reported a refusal");
  assert(time < settled_s or flight.beforeHorizonCount > 0,
    "No measurement was presented before the first release, so this run does
     not exercise the start-up window at all");
  // A queue that delivers NOTHING satisfies every assertion above. Say what
  // the run is required to have exercised.
  // Measured: 129 deliveries over 1.2 s across GPS, barometer and flow, and
  // 35 samples presented before the horizon existed.
  assert(time < stopTime_s - 1.0e-9 or flight.deliveredCount >= 100,
    "Too few measurements reached the filter for the delivery assertions above
     to mean anything");

  // ---- 5b. the degenerate case is a pass-through --------------------------
  // At the shortest horizon the block admits and with no sensor latency, the
  // queue must cost nothing: everything delivered, nothing refused, nothing
  // stale. This is the anchor the delayed case is measured against, and it is
  // what says the machinery has no fixed overhead that survives shrinking the
  // horizon.
  // Measured: 59 deliveries over 1.2 s, worst residual 0.008750 s, no
  // refusals -- the same residual the flight arm shows, which is the point:
  // the bound is a property of the release lattice and not of the horizon.
  assert(degenerate.refusedLateCount == 0,
    "A zero-latency measurement was refused as later than a horizon of one
     release window, so the queue is not a pass-through in the degenerate
     case");
  assert(degenerate.refusedOverflowCount == 0,
    "A zero-latency measurement overflowed a queue at the shortest horizon");
  assert(degenerate.droppedStaleCount == 0,
    "A zero-latency measurement went stale at the shortest horizon");
  assert(time < stopTime_s - 1.0e-9 or degenerate.deliveredCount >= 30,
    "The degenerate case delivered almost nothing, so the assertions above
     hold vacuously and the anchor is not anchoring anything");

  // ---- 6. the refusal boundary, from both sides --------------------------
  // The boundary in transport latency is fusionHorizon_s = 0.2 s, for the
  // reason recorded where the two latencies are declared. These arms sit on
  // otherwise identical configurations half a release window inside it and one
  // and a half windows outside it, which is what makes them a boundary test
  // rather than two unrelated cases.
  // Measured: marginal 9 delivered, 0 refused late, 0 dropped stale; late 0
  // delivered, 10 refused late.
  assert(marginal.refusedLateCount == 0,
    "A fix arriving half a release window INSIDE the horizon was refused as
     late, so the boundary is tighter than the horizon declares");
  assert(time < stopTime_s - 1.0e-9 or marginal.deliveredCount >= 5,
    "The marginal arm delivered almost nothing, so the inside half of the
     boundary test is not exercised");
  assert(late.deliveredCount == 0,
    "A measurement the fusion instant had already passed was delivered anyway,
     so it would be fused against a state that has moved past it, which is the
     case fusing at a horizon exists to eliminate");
  assert(time < settled_s or late.refusedLateCount > 0,
    "A fix arriving half a release window OUTSIDE the refusal boundary was
     neither delivered nor refused, so it vanished with no outcome, which is
     the predicate exhaustion this boundary refuses");

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
  // Measured with the arrival refused: 114 refusals, 110 deliveries.
  // Measured with the oldest displaced: 0 deliveries.
  assert(time < settled_s or crowded.refusedOverflowCount > 0,
    "A source delivering forty times its declared rate did not overflow its
     queue, so the overflow path is not being exercised");
  assert(time < stopTime_s - 1.0e-9 or crowded.deliveredCount >= 30,
    "An overflowing queue stopped delivering, so one burst blinded the source
     for the rest of the flight. Refusing the ARRIVAL rather than displacing
     the oldest entry is what this assertion protects");

  annotation(experiment(StartTime=0.0, StopTime=1.2,
    Tolerance=1.0e-8, Interval=0.00125),
    Documentation(info="<html>
    <p>Simulated as a top-level model through
    <code>Tests/run-horizon.mos</code>. The epoch and the release pulse come
    from a real <code>Estimation.FusionHorizon.OutputPredictor</code> rather
    than a synthetic clock, because the property under test is that the
    queues and the delta ring agree about which instant the filter stands on,
    and a test that manufactured its own epoch would agree with itself.</p>
    <p><b>The flight arm runs the real lattice and the real latencies.</b> The
    horizon, the source rates and the source transport delays are the ones
    <code>Vehicles.Rdd2.WaypointVehicleSystem</code> carries, and those delays
    are PX4's shipped EKF2 defaults, so the residual measured here is the one
    the deployed configuration produces rather than one produced by a lattice
    chosen to make the run short. Two further arms bracket the refusal
    boundary at <code>fusionHorizon_s + maximumResidualAge_s</code> by half a
    release window on each side; the short-horizon arms remain for the
    mechanism properties, overflow and the degenerate anchor, where the rate is
    not what is under test.</p>
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
