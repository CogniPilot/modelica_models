within Estimation.FusionHorizon;

block AidingBuffer
  "Bounded per-source measurement queues that hand each aiding packet to the
   filter at the fusion instant its own timestamp names"

  parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.00125
    "Inertial tick. Arrivals are admitted at this rate so no sensor pulse is
     missed between releases; deliveries happen only on a release";
  parameter Real fusionPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "Filter release interval. It is also the width of the sub-window a
     delivered measurement can still be offset from the fusion instant by,
     which is what bounds maximumResidualAge_s below";
  parameter Real fusionHorizon_s(unit = "s", min = 1.0e-9) = 0.2
    "Lag of the fusion instant behind now, the same horizon the delta ring is
     sized for. It is what each queue has to hold, so it is structural here
     for the same reason the ring length is structural there"
    annotation(Evaluate = true);
  // The declared source rates and the slack are the ONLY tuning surface for
  // the queue capacities, and they are structural: a queue length is an array
  // dimension, so it has to carry a value when the code is generated rather
  // than at run time. Evaluate = true says that, and the alternative is a
  // dynamically sized buffer, which is the thing a flight timing record cannot
  // be written against and which the code generator is right to refuse.
  parameter Real mocapPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    annotation(Evaluate = true);
  parameter Real gpsPeriod_s(unit = "s", min = 1.0e-9) = 0.1
    annotation(Evaluate = true);
  parameter Real magnetometerPeriod_s(unit = "s", min = 1.0e-9) = 0.05
    annotation(Evaluate = true);
  parameter Real barometerPeriod_s(unit = "s", min = 1.0e-9) = 0.02
    annotation(Evaluate = true);
  parameter Real opticalFlowPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "Shortest interval each source is declared to deliver at. It sets that
     source's queue capacity and nothing else; a source delivering FASTER than
     it says overflows its queue and reports the drop rather than corrupting
     anything. A deployment that knows a stream is absent shrinks its queue by
     declaring a longer period here, which is one place to state a source's
     rate rather than two"
    annotation(Evaluate = true);
  parameter Integer depthSlack(min = 0) = 2
    "Slots each queue carries beyond one horizon of its own source, so a queue
     reaching exactly one horizon on the tick the fusion instant starts moving
     does not have to drop its oldest entry to accept the next arrival"
    annotation(Evaluate = true);
  parameter Real maximumResidualAge_s(unit = "s", min = 0.0) = fusionPeriod_s
    "How far the fusion instant may stand past a measurement's own timestamp
     and still fuse it. THE bound the delayed-aiding argument rests on.

     A measurement inside the horizon ripens on the first release at or after
     its own timestamp, so the residual is in [0, fusionPeriod_s) by
     construction and this bound is never the binding constraint in flight.
     What it does is name the two failures at the edges: a packet that arrives
     after the fusion instant has already passed it, and a packet that could
     not be drained fast enough. Both are refused with a named outcome rather
     than fused against a state that has moved past them.

     It replaces Estimation.StrapdownINS.PartialEstimator.maximumAidingDelay_s
     as the quantity the transport error is charged against, and the two are
     not the same size: that one is 0.25 s and admits a cubic-Taylor transport
     error the design record puts at 12 to 27 percent, this one is one release
     period. At the flight lattice that is 0.01 s against 0.25 s.";
  parameter Real maximumSourceDelay_s(unit = "s", min = 0.0) = 0.11
    "Worst end-to-end age any aiding source is declared to deliver at. It is
     what the horizon has to be long enough to cover, and it is stated rather
     than inferred because this block cannot see a sensor: it sees timestamps.

     The default is the largest plant-side latency the deployed vehicle
     carries, GPS at 110 ms, which is PX4's last shipped EKF2_GPS_DELAY
     default. A deployment with a slower source must raise this, and raising it
     past the horizon is refused below rather than silently aging that source
     out of every fusion.";
  parameter Real horizonJitterMargin_s(unit = "s", min = 0.0) = 0.05
    "Headroom the horizon must keep beyond the worst declared source delay.

     A delay is a nominal, not a bound: a driver that usually delivers at
     110 ms occasionally delivers later, and a horizon sized exactly to the
     nominal turns every one of those into a refused measurement. 50 ms is
     roughly half the GPS delay it is protecting, which is the same order as
     the jitter a scheduler and a link contribute together, and the flight
     configuration clears the sum by a further 40 ms.

     PX4 states the same relation without the margin: EKF2_DELAY_MAX, the
     delay between now and its delayed-time horizon, `should be at least as
     large as the largest EKF2_XXX_DELAY parameter`. This is that rule with
     headroom, and it is an assertion rather than a note.";
  parameter Real epochTolerance_s(unit = "s", min = 0.0) = 1.0e-7
    "Slack on the ripeness comparison. A measurement stamped AT the fusion
     instant has to be ripe at it, and a carried epoch advanced by repeated
     addition does not land on a sensor's timestamp exactly.

     It is an order of magnitude BELOW the 1e-6 s at which the filters reject
     a negatively aged measurement, and that ordering is the point rather than
     a coincidence. The filter recomputes the age itself from the same two
     numbers, so a measurement this block admits as ripe within its tolerance
     reaches the filter with an age no more negative than that tolerance; if
     the two were equal the outcome would sit on a knife edge and a
     measurement could be released here and refused there with nothing saying
     why. Both are far below the 10 ms release period and far above the
     representation error of either quantity over a flight.";

  // ---- queue capacities ---------------------------------------------------
  // A whole horizon of each source, rounded up, plus the slack. It is worth
  // saying why the answer is the WHOLE horizon rather than the part of it a
  // measurement actually waits out. In steady flight a measurement stamped
  // t_m reaches the driver at t_m + L and ripens when the fusion instant
  // reaches t_m, at t_m + D, so it waits D - L and the queue stands at
  // (D - L) / sourcePeriod entries. STARTUP is the worse case and it is the
  // one the capacity has to cover: the fusion instant does not begin advancing
  // until the delta ring has filled, so everything a source delivers during
  // that first horizon is queued at once, which is D / sourcePeriod entries.
  //
  // Written inline rather than as a call to a named function, and that is a
  // COMPILER ACCOMMODATION rather than a preference, recorded so it can be
  // reverted. Rumoca 0.10.0 folds this arithmetic in an array-dimension
  // position but does not fold a user function call in the same position once
  // the block is a SUB-COMPONENT: standalone the call folds, composed it
  // reports the dimension unevaluable. The two forms compute the same integer
  // and the named one read better.
  final parameter Integer mocapDepth(min = 2) =
    integer(ceil(fusionHorizon_s / mocapPeriod_s - 1.0e-9)) + depthSlack;
  final parameter Integer gpsDepth(min = 2) =
    integer(ceil(fusionHorizon_s / gpsPeriod_s - 1.0e-9)) + depthSlack;
  final parameter Integer magnetometerDepth(min = 2) =
    integer(ceil(fusionHorizon_s / magnetometerPeriod_s - 1.0e-9)) + depthSlack;
  final parameter Integer barometerDepth(min = 2) =
    integer(ceil(fusionHorizon_s / barometerPeriod_s - 1.0e-9)) + depthSlack;
  final parameter Integer opticalFlowDepth(min = 2) =
    integer(ceil(fusionHorizon_s / opticalFlowPeriod_s - 1.0e-9)) + depthSlack
    "Queue capacities, derived from the horizon and each source's declared
     rate. Final rather than overridable, because a capacity and the rate it
     is derived from must not be settable independently: a queue declared
     shorter than the rate beside it implies would drop measurements on the
     ground and report a source rate that nothing in the model contradicts.
     Nothing here is allocated at run time -- every capacity is fixed at
     translation, every walk is fixed-length, and every index wraps at most
     once";

  input Boolean reset;
  input Boolean horizonValid
    "A fusion instant exists. Before the first release the epoch below is
     still its seed value, so nothing may be measured against it and nothing
     is delivered";
  input Real horizonEpoch_s(unit = "s")
    "Timestamp of the fusion instant the filter is standing on. The same
     number Estimation.FusionHorizon.OutputPredictor stamps on the inertial
     packet it releases, and it must be: the two packets the filter consumes
     on one tick have to name one instant, which is the whole content of
     fusing at a horizon";
  input Boolean horizonReleased
    "The horizon handed a window over on this tick. Deliveries are pulsed onto
     this signal so an aiding packet and the inertial packet reach the filter
     together";

  Avionics.MocapSampleInput mocap;
  Avionics.GpsSampleInput gps;
  Avionics.MagnetometerSampleInput magnetometer;
  Avionics.BarometerSampleInput barometer;
  Avionics.OpticalFlowSampleInput opticalFlow;

  discrete Avionics.MocapSampleOutput mocapAtHorizon;
  discrete Avionics.GpsSampleOutput gpsAtHorizon;
  discrete Avionics.MagnetometerSampleOutput magnetometerAtHorizon;
  discrete Avionics.BarometerSampleOutput barometerAtHorizon;
  discrete Avionics.OpticalFlowSampleOutput opticalFlowAtHorizon
    "The aiding streams as the filter sees them: each packet held back until
     the fusion instant reaches its own timestamp, then delivered PULSED on
     the release tick. valid and fresh are the same boolean, as they are on
     the inertial packet and for the same reason -- a measurement either was
     handed over on this tick or was not";

  discrete output Real worstDeliveredAge_s(
    unit = "s", start = 0.0, fixed = true)
    "Largest residual any source has been delivered with since the last reset.
     The quantity the filter still transports its measurement Jacobian over,
     published so the bound is observable rather than argued";
  discrete output Integer refusedLateCount(start = 0, fixed = true)
    "Measurements refused at arrival because the fusion instant had already
     passed them. On a correctly sized horizon this stays at zero, and a
     nonzero value says the horizon is shorter than the transport latency of
     some source, which is a configuration fact rather than a filter fault";
  discrete output Integer droppedStaleCount(start = 0, fixed = true)
    "Queued measurements discarded at delivery for the same reason";
  discrete output Integer refusedOverflowCount(start = 0, fixed = true)
    "Arrivals refused because the queue was full. Nonzero means a source is
     delivering faster than its declared period, so its capacity is short. The
     queue keeps the entries it already holds, which are the ones the fusion
     instant is about to reach; see AidingRefusedOverflow for why that is the
     right way round for a delayed queue and the wrong way round for a
     live-edge one";
  discrete output Integer deliveredCount(start = 0, fixed = true)
    "Measurements handed to the filter since the last reset, across every
     source. Published because every other supervision signal here is a count
     of something going WRONG, and a queue that delivers nothing satisfies all
     of them: without this a silent source and a healthy one are the same
     reading";
  discrete output Boolean deliveryOutOfOrder(start = false, fixed = true)
    "LATCHED. Some source delivered a measurement whose timestamp did not
     advance on the one before it. The queues are FIFOs and the fusion instant
     advances monotonically, so this cannot happen; it is checked here, beside
     the data, because it is an invariant of this block and because a consumer
     cannot check it -- the aiding connectors carry one packet at a time and
     the ordering is a property of the sequence.";
  discrete output Boolean deliveryAfterHorizon(start = false, fixed = true)
    "LATCHED. Some source delivered a measurement the fusion instant had NOT
     yet reached, which is a measurement from the filter's own future. This is
     the delayed-fusion contract itself, and it is asserted where the epoch and
     the timestamp are both in hand rather than reconstructed downstream.";
  discrete output Integer beforeHorizonCount(start = 0, fixed = true)
    "Samples presented before the first release. NOT a refusal and not part of
     aidingRefused: the horizon costs one horizon of start-up and this counts
     it. Published so a reader can tell a start-up window from a source that
     is genuinely arriving late, which is the distinction the aggregate
     counters exist to preserve";
  discrete output Boolean aidingRefused(start = false, fixed = true)
    "Any of the three above happened on this tick. A level for one tick, so a
     supervisor sees the event rather than having to difference the counts";

protected
  discrete Real mocapQueue[mocapDepth, MocapMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer mocapHead(start = 1, fixed = true);
  discrete Integer mocapTail(start = 1, fixed = true);
  discrete Integer mocapCount(start = 0, fixed = true);
  discrete Real mocapAdmitted_s(start = -TimestampMagnitudeLimit, fixed = true)
    "Timestamp of the last sample admitted from this source. Novelty by
     timestamp is the exactly-once handshake across the sensor and inertial
     clocks, the same rule the filters use on the same records: a level-held
     sample is admitted on the first tick it is presented and on no other";
  discrete Real mocapStoredRow[MocapMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer mocapStoreSlot(start = 0, fixed = true);
  discrete Real mocapDeliveredRow[MocapMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Real mocapAge_s(start = 0.0, fixed = true);
  discrete Boolean mocapDelivered(start = false, fixed = true);
  discrete Integer mocapArrival(start = 0, fixed = true);
  discrete Integer mocapDelivery(start = 0, fixed = true);

  discrete Real gpsQueue[gpsDepth, GpsMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer gpsHead(start = 1, fixed = true);
  discrete Integer gpsTail(start = 1, fixed = true);
  discrete Integer gpsCount(start = 0, fixed = true);
  discrete Real gpsAdmitted_s(start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real gpsStoredRow[GpsMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer gpsStoreSlot(start = 0, fixed = true);
  discrete Real gpsDeliveredRow[GpsMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Real gpsAge_s(start = 0.0, fixed = true);
  discrete Boolean gpsDelivered(start = false, fixed = true);
  discrete Integer gpsArrival(start = 0, fixed = true);
  discrete Integer gpsDelivery(start = 0, fixed = true);

  discrete Real magnetometerQueue[
    magnetometerDepth, MagnetometerMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer magnetometerHead(start = 1, fixed = true);
  discrete Integer magnetometerTail(start = 1, fixed = true);
  discrete Integer magnetometerCount(start = 0, fixed = true);
  discrete Real magnetometerAdmitted_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real magnetometerStoredRow[MagnetometerMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer magnetometerStoreSlot(start = 0, fixed = true);
  discrete Real magnetometerDeliveredRow[MagnetometerMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Real magnetometerAge_s(start = 0.0, fixed = true);
  discrete Boolean magnetometerDelivered(start = false, fixed = true);
  discrete Integer magnetometerArrival(start = 0, fixed = true);
  discrete Integer magnetometerDelivery(start = 0, fixed = true);

  discrete Real barometerQueue[barometerDepth, BarometerMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer barometerHead(start = 1, fixed = true);
  discrete Integer barometerTail(start = 1, fixed = true);
  discrete Integer barometerCount(start = 0, fixed = true);
  discrete Real barometerAdmitted_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real barometerStoredRow[BarometerMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer barometerStoreSlot(start = 0, fixed = true);
  discrete Real barometerDeliveredRow[BarometerMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Real barometerAge_s(start = 0.0, fixed = true);
  discrete Boolean barometerDelivered(start = false, fixed = true);
  discrete Integer barometerArrival(start = 0, fixed = true);
  discrete Integer barometerDelivery(start = 0, fixed = true);

  discrete Real opticalFlowQueue[
    opticalFlowDepth, OpticalFlowMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer opticalFlowHead(start = 1, fixed = true);
  discrete Integer opticalFlowTail(start = 1, fixed = true);
  discrete Integer opticalFlowCount(start = 0, fixed = true);
  discrete Real opticalFlowAdmitted_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real opticalFlowStoredRow[OpticalFlowMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Integer opticalFlowStoreSlot(start = 0, fixed = true);
  discrete Real opticalFlowDeliveredRow[OpticalFlowMeasurementLength](
    each start = 0.0, each fixed = true);
  discrete Real opticalFlowAge_s(start = 0.0, fixed = true);
  discrete Boolean opticalFlowDelivered(start = false, fixed = true);
  discrete Integer opticalFlowArrival(start = 0, fixed = true);
  discrete Integer opticalFlowDelivery(start = 0, fixed = true);

  // The unpacked payload is landed in ORDINARY VARIABLES first and copied to
  // the connector field by field afterwards. Naming connector components as
  // the outputs of a multiple-output function call does not survive
  // OpenModelica 1.27 here: measured, every Real member so assigned published
  // zero while the row the call was given was correct and every scalar the
  // same block computed from that row was correct. The failure is silent,
  // which is what makes it worth a comment rather than a shrug -- a queue
  // that delivers a valid packet full of zeros passes every ordering and
  // residual assertion written about it.
  discrete Real mocapOutTimestamp_s;
  discrete Real mocapOutPosition_m[3];
  discrete Real mocapOutQuaternion[4];
  discrete Real mocapOutPositionCovariance_m2[3, 3];
  discrete Real mocapOutAttitudeCovariance_rad2[3, 3];
  discrete Real gpsOutTimestamp_s;
  discrete Boolean gpsOutPositionValid;
  discrete Boolean gpsOutVelocityValid;
  discrete Real gpsOutGeodetic_deg_m[3];
  discrete Real gpsOutPosition_m[3];
  discrete Real gpsOutVelocity_m_s[3];
  discrete Real gpsOutPositionCovariance_m2[3, 3];
  discrete Real gpsOutVelocityCovariance_m2_s2[3, 3];
  discrete Real magnetometerOutTimestamp_s;
  discrete Real magnetometerOutField_T[3];
  discrete Real magnetometerOutCovariance_T2[3, 3];
  discrete Real barometerOutTimestamp_s;
  discrete Real barometerOutAltitude_m;
  discrete Real barometerOutVariance_m2;
  discrete Real opticalFlowOutTimestamp_s;
  discrete Real opticalFlowOutLineOfSight_rad[2];
  discrete Real opticalFlowOutLineOfSightCovariance_rad2[2, 2];
  discrete Real opticalFlowOutGyroscope_rad[3];
  discrete Real opticalFlowOutGyroscopeCovariance_rad2[3, 3];
  discrete Real opticalFlowOutIntegrationTime_s;
  discrete Real opticalFlowOutGroundDistance_m;
  discrete Real opticalFlowOutGroundDistanceVariance_m2;
  discrete Real opticalFlowOutQuality;

  discrete Real mocapLastDelivered_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real gpsLastDelivered_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real magnetometerLastDelivered_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real barometerLastDelivered_s(
    start = -TimestampMagnitudeLimit, fixed = true);
  discrete Real opticalFlowLastDelivered_s(
    start = -TimestampMagnitudeLimit, fixed = true);

  discrete Real mocapArrivedRow[MocapMeasurementLength];
  discrete Real gpsArrivedRow[GpsMeasurementLength];
  discrete Real magnetometerArrivedRow[MagnetometerMeasurementLength];
  discrete Real barometerArrivedRow[BarometerMeasurementLength];
  discrete Real opticalFlowArrivedRow[OpticalFlowMeasurementLength];
  discrete Boolean mocapArrived(start = false, fixed = true);
  discrete Boolean gpsArrived(start = false, fixed = true);
  discrete Boolean magnetometerArrived(start = false, fixed = true);
  discrete Boolean barometerArrived(start = false, fixed = true);
  discrete Boolean opticalFlowArrived(start = false, fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    // Five queues, one kernel, five copies of the same twelve lines. The
    // repetition is not an accident and it is not a loop waiting to be
    // written: the five sources pack to five different widths, so a loop over
    // them would need one width for all five and would carry a mocap-sized
    // row for a barometer. Everything that could differ between the five is in
    // stepQueue, which is written once.
    //
    // ARRIVALS ARE ADMITTED AT THE INERTIAL RATE, deliveries only on a
    // release. A sensor pulse that lands between two releases must not be
    // missed, and novelty by timestamp makes admitting on every tick the same
    // as admitting once.

    // ---- motion capture ---------------------------------------------------
    mocapArrivedRow := Estimation.FusionHorizon.packMocap(mocap);
    mocapArrived := (not reset) and mocap.valid
      and abs(mocap.timestamp_s) < TimestampMagnitudeLimit
      and mocap.timestamp_s > pre(mocapAdmitted_s) + 1.0e-9;
    (mocapStoreSlot,
     mocapStoredRow,
     mocapHead,
     mocapTail,
     mocapCount,
     mocapDeliveredRow,
     mocapAge_s,
     mocapDelivered,
     mocapArrival,
     mocapDelivery) := Estimation.FusionHorizon.stepQueue(
      reset, pre(mocapQueue), pre(mocapHead), pre(mocapTail), pre(mocapCount),
      mocapArrived, mocapArrivedRow, horizonReleased, horizonValid,
      horizonEpoch_s, maximumResidualAge_s, epochTolerance_s);
    mocapQueue := Estimation.FusionHorizon.storeMeasurement(
      pre(mocapQueue), mocapStoreSlot, mocapStoredRow);
    // Advanced on ADMISSION rather than on delivery, and on a refusal too. A
    // sample refused for being later than the horizon, or for arriving at a
    // full queue, has still been SEEN; re-offering it on every subsequent tick
    // would re-refuse it on every subsequent tick and report a storm of
    // refusals for one packet. The one outcome that does NOT advance it is
    // AidingBeforeHorizon, because that sample was never offered a fusion
    // instant and must still be admitted once one exists.
    mocapAdmitted_s := if reset then -TimestampMagnitudeLimit
      elseif mocapArrival <> AidingNoArrival
        and mocapArrival <> AidingBeforeHorizon
      then mocap.timestamp_s else pre(mocapAdmitted_s);
    (mocapOutTimestamp_s,
     mocapOutPosition_m,
     mocapOutQuaternion,
     mocapOutPositionCovariance_m2,
     mocapOutAttitudeCovariance_rad2) :=
      Estimation.FusionHorizon.unpackMocap(mocapDeliveredRow);
    mocapAtHorizon.timestamp_s := mocapOutTimestamp_s;
    mocapAtHorizon.positionWorldEnu_m := mocapOutPosition_m;
    mocapAtHorizon.quaternionWorldBody := mocapOutQuaternion;
    mocapAtHorizon.positionCovarianceWorld_m2 := mocapOutPositionCovariance_m2;
    mocapAtHorizon.attitudeCovarianceBody_rad2 :=
      mocapOutAttitudeCovariance_rad2;
    mocapAtHorizon.valid := mocapDelivered;
    mocapAtHorizon.fresh := mocapDelivered;

    // ---- GPS --------------------------------------------------------------
    gpsArrivedRow := Estimation.FusionHorizon.packGps(gps);
    gpsArrived := (not reset) and gps.valid
      and abs(gps.timestamp_s) < TimestampMagnitudeLimit
      and gps.timestamp_s > pre(gpsAdmitted_s) + 1.0e-9;
    (gpsStoreSlot,
     gpsStoredRow,
     gpsHead,
     gpsTail,
     gpsCount,
     gpsDeliveredRow,
     gpsAge_s,
     gpsDelivered,
     gpsArrival,
     gpsDelivery) := Estimation.FusionHorizon.stepQueue(
      reset, pre(gpsQueue), pre(gpsHead), pre(gpsTail), pre(gpsCount),
      gpsArrived, gpsArrivedRow, horizonReleased, horizonValid,
      horizonEpoch_s, maximumResidualAge_s, epochTolerance_s);
    gpsQueue := Estimation.FusionHorizon.storeMeasurement(
      pre(gpsQueue), gpsStoreSlot, gpsStoredRow);
    gpsAdmitted_s := if reset then -TimestampMagnitudeLimit
      elseif gpsArrival <> AidingNoArrival
        and gpsArrival <> AidingBeforeHorizon
      then gps.timestamp_s else pre(gpsAdmitted_s);
    (gpsOutTimestamp_s,
     gpsOutPositionValid,
     gpsOutVelocityValid,
     gpsOutGeodetic_deg_m,
     gpsOutPosition_m,
     gpsOutVelocity_m_s,
     gpsOutPositionCovariance_m2,
     gpsOutVelocityCovariance_m2_s2) :=
      Estimation.FusionHorizon.unpackGps(gpsDeliveredRow);
    gpsAtHorizon.timestamp_s := gpsOutTimestamp_s;
    // A fix that was not delivered offers neither half of its solution. The
    // stored flags say which half the SOLUTION carried; delivery says whether
    // there is a solution at all, and both have to hold.
    gpsAtHorizon.positionValid := gpsOutPositionValid and gpsDelivered;
    gpsAtHorizon.velocityValid := gpsOutVelocityValid and gpsDelivered;
    gpsAtHorizon.geodetic_deg_m := gpsOutGeodetic_deg_m;
    gpsAtHorizon.positionWorldEnu_m := gpsOutPosition_m;
    gpsAtHorizon.velocityWorldEnu_m_s := gpsOutVelocity_m_s;
    gpsAtHorizon.positionCovarianceWorld_m2 := gpsOutPositionCovariance_m2;
    gpsAtHorizon.velocityCovarianceWorld_m2_s2 :=
      gpsOutVelocityCovariance_m2_s2;
    gpsAtHorizon.valid := gpsDelivered;
    gpsAtHorizon.fresh := gpsDelivered;

    // ---- magnetometer -----------------------------------------------------
    magnetometerArrivedRow :=
      Estimation.FusionHorizon.packMagnetometer(magnetometer);
    magnetometerArrived := (not reset) and magnetometer.valid
      and abs(magnetometer.timestamp_s) < TimestampMagnitudeLimit
      and magnetometer.timestamp_s > pre(magnetometerAdmitted_s) + 1.0e-9;
    (magnetometerStoreSlot,
     magnetometerStoredRow,
     magnetometerHead,
     magnetometerTail,
     magnetometerCount,
     magnetometerDeliveredRow,
     magnetometerAge_s,
     magnetometerDelivered,
     magnetometerArrival,
     magnetometerDelivery) := Estimation.FusionHorizon.stepQueue(
      reset, pre(magnetometerQueue), pre(magnetometerHead),
      pre(magnetometerTail), pre(magnetometerCount),
      magnetometerArrived, magnetometerArrivedRow, horizonReleased,
      horizonValid, horizonEpoch_s, maximumResidualAge_s, epochTolerance_s);
    magnetometerQueue := Estimation.FusionHorizon.storeMeasurement(
      pre(magnetometerQueue), magnetometerStoreSlot, magnetometerStoredRow);
    magnetometerAdmitted_s := if reset then -TimestampMagnitudeLimit
      elseif magnetometerArrival <> AidingNoArrival
        and magnetometerArrival <> AidingBeforeHorizon
      then magnetometer.timestamp_s else pre(magnetometerAdmitted_s);
    (magnetometerOutTimestamp_s,
     magnetometerOutField_T,
     magnetometerOutCovariance_T2) :=
      Estimation.FusionHorizon.unpackMagnetometer(magnetometerDeliveredRow);
    magnetometerAtHorizon.timestamp_s := magnetometerOutTimestamp_s;
    magnetometerAtHorizon.magneticFieldBodyFlu_T := magnetometerOutField_T;
    magnetometerAtHorizon.covarianceBody_T2 := magnetometerOutCovariance_T2;
    magnetometerAtHorizon.valid := magnetometerDelivered;
    magnetometerAtHorizon.fresh := magnetometerDelivered;

    // ---- barometer --------------------------------------------------------
    barometerArrivedRow :=
      Estimation.FusionHorizon.packBarometer(barometer);
    barometerArrived := (not reset) and barometer.valid
      and abs(barometer.timestamp_s) < TimestampMagnitudeLimit
      and barometer.timestamp_s > pre(barometerAdmitted_s) + 1.0e-9;
    (barometerStoreSlot,
     barometerStoredRow,
     barometerHead,
     barometerTail,
     barometerCount,
     barometerDeliveredRow,
     barometerAge_s,
     barometerDelivered,
     barometerArrival,
     barometerDelivery) := Estimation.FusionHorizon.stepQueue(
      reset, pre(barometerQueue), pre(barometerHead), pre(barometerTail),
      pre(barometerCount), barometerArrived, barometerArrivedRow,
      horizonReleased, horizonValid, horizonEpoch_s, maximumResidualAge_s,
      epochTolerance_s);
    barometerQueue := Estimation.FusionHorizon.storeMeasurement(
      pre(barometerQueue), barometerStoreSlot, barometerStoredRow);
    barometerAdmitted_s := if reset then -TimestampMagnitudeLimit
      elseif barometerArrival <> AidingNoArrival
        and barometerArrival <> AidingBeforeHorizon
      then barometer.timestamp_s else pre(barometerAdmitted_s);
    (barometerOutTimestamp_s,
     barometerOutAltitude_m,
     barometerOutVariance_m2) :=
      Estimation.FusionHorizon.unpackBarometer(barometerDeliveredRow);
    barometerAtHorizon.timestamp_s := barometerOutTimestamp_s;
    barometerAtHorizon.altitudeWorldEnu_m := barometerOutAltitude_m;
    barometerAtHorizon.variance_m2 := barometerOutVariance_m2;
    barometerAtHorizon.valid := barometerDelivered;
    barometerAtHorizon.fresh := barometerDelivered;

    // ---- optical flow -----------------------------------------------------
    opticalFlowArrivedRow :=
      Estimation.FusionHorizon.packOpticalFlow(opticalFlow);
    opticalFlowArrived := (not reset) and opticalFlow.valid
      and abs(opticalFlow.timestamp_s) < TimestampMagnitudeLimit
      and opticalFlow.timestamp_s > pre(opticalFlowAdmitted_s) + 1.0e-9;
    (opticalFlowStoreSlot,
     opticalFlowStoredRow,
     opticalFlowHead,
     opticalFlowTail,
     opticalFlowCount,
     opticalFlowDeliveredRow,
     opticalFlowAge_s,
     opticalFlowDelivered,
     opticalFlowArrival,
     opticalFlowDelivery) := Estimation.FusionHorizon.stepQueue(
      reset, pre(opticalFlowQueue), pre(opticalFlowHead),
      pre(opticalFlowTail), pre(opticalFlowCount),
      opticalFlowArrived, opticalFlowArrivedRow, horizonReleased,
      horizonValid, horizonEpoch_s, maximumResidualAge_s, epochTolerance_s);
    opticalFlowQueue := Estimation.FusionHorizon.storeMeasurement(
      pre(opticalFlowQueue), opticalFlowStoreSlot, opticalFlowStoredRow);
    opticalFlowAdmitted_s := if reset then -TimestampMagnitudeLimit
      elseif opticalFlowArrival <> AidingNoArrival
        and opticalFlowArrival <> AidingBeforeHorizon
      then opticalFlow.timestamp_s else pre(opticalFlowAdmitted_s);
    (opticalFlowOutTimestamp_s,
     opticalFlowOutLineOfSight_rad,
     opticalFlowOutLineOfSightCovariance_rad2,
     opticalFlowOutGyroscope_rad,
     opticalFlowOutGyroscopeCovariance_rad2,
     opticalFlowOutIntegrationTime_s,
     opticalFlowOutGroundDistance_m,
     opticalFlowOutGroundDistanceVariance_m2,
     opticalFlowOutQuality) :=
      Estimation.FusionHorizon.unpackOpticalFlow(opticalFlowDeliveredRow);
    opticalFlowAtHorizon.timestamp_s := opticalFlowOutTimestamp_s;
    opticalFlowAtHorizon.integratedLineOfSight_rad :=
      opticalFlowOutLineOfSight_rad;
    opticalFlowAtHorizon.integratedLineOfSightCovariance_rad2 :=
      opticalFlowOutLineOfSightCovariance_rad2;
    opticalFlowAtHorizon.integratedGyroscopeBodyFlu_rad :=
      opticalFlowOutGyroscope_rad;
    opticalFlowAtHorizon.integratedGyroscopeCovariance_rad2 :=
      opticalFlowOutGyroscopeCovariance_rad2;
    opticalFlowAtHorizon.integrationTime_s := opticalFlowOutIntegrationTime_s;
    opticalFlowAtHorizon.groundDistance_m := opticalFlowOutGroundDistance_m;
    opticalFlowAtHorizon.groundDistanceVariance_m2 :=
      opticalFlowOutGroundDistanceVariance_m2;
    opticalFlowAtHorizon.quality := opticalFlowOutQuality;
    opticalFlowAtHorizon.valid := opticalFlowDelivered;
    opticalFlowAtHorizon.fresh := opticalFlowDelivered;

    // ---- supervision ------------------------------------------------------
    // Counts rather than levels, for the reason the accepted-correction count
    // exists on the estimator status boundary: a level held for one tick of a
    // fast clock is invisible to a slower consumer, and a monotonic count is
    // the only signal with a well-defined edge across a rate change.
    refusedLateCount := if reset then 0 else pre(refusedLateCount)
      + (if mocapArrival == AidingRefusedLate then 1 else 0)
      + (if gpsArrival == AidingRefusedLate then 1 else 0)
      + (if magnetometerArrival == AidingRefusedLate then 1 else 0)
      + (if barometerArrival == AidingRefusedLate then 1 else 0)
      + (if opticalFlowArrival == AidingRefusedLate then 1 else 0);
    refusedOverflowCount := if reset then 0 else pre(refusedOverflowCount)
      + (if mocapArrival == AidingRefusedOverflow then 1 else 0)
      + (if gpsArrival == AidingRefusedOverflow then 1 else 0)
      + (if magnetometerArrival == AidingRefusedOverflow then 1 else 0)
      + (if barometerArrival == AidingRefusedOverflow then 1 else 0)
      + (if opticalFlowArrival == AidingRefusedOverflow then 1 else 0);
    droppedStaleCount := if reset then 0 else pre(droppedStaleCount)
      + (if mocapDelivery == AidingDroppedStale then 1 else 0)
      + (if gpsDelivery == AidingDroppedStale then 1 else 0)
      + (if magnetometerDelivery == AidingDroppedStale then 1 else 0)
      + (if barometerDelivery == AidingDroppedStale then 1 else 0)
      + (if opticalFlowDelivery == AidingDroppedStale then 1 else 0);
    beforeHorizonCount := if reset then 0 else pre(beforeHorizonCount)
      + (if mocapArrival == AidingBeforeHorizon then 1 else 0)
      + (if gpsArrival == AidingBeforeHorizon then 1 else 0)
      + (if magnetometerArrival == AidingBeforeHorizon then 1 else 0)
      + (if barometerArrival == AidingBeforeHorizon then 1 else 0)
      + (if opticalFlowArrival == AidingBeforeHorizon then 1 else 0);
    deliveredCount := if reset then 0 else pre(deliveredCount)
      + (if mocapDelivered then 1 else 0)
      + (if gpsDelivered then 1 else 0)
      + (if magnetometerDelivered then 1 else 0)
      + (if barometerDelivered then 1 else 0)
      + (if opticalFlowDelivered then 1 else 0);
    // ORDER AND EPOCH, checked against the ROWS rather than against the
    // connectors. Both are invariants of this block, and both are latched: a
    // single violation anywhere in a flight has to remain visible, which a
    // comparison written against the current tick alone would forget on the
    // next one.
    deliveryOutOfOrder := (not reset) and (pre(deliveryOutOfOrder)
      or (mocapDelivered
        and mocapDeliveredRow[1] <= pre(mocapLastDelivered_s))
      or (gpsDelivered and gpsDeliveredRow[1] <= pre(gpsLastDelivered_s))
      or (magnetometerDelivered
        and magnetometerDeliveredRow[1] <= pre(magnetometerLastDelivered_s))
      or (barometerDelivered
        and barometerDeliveredRow[1] <= pre(barometerLastDelivered_s))
      or (opticalFlowDelivered
        and opticalFlowDeliveredRow[1] <= pre(opticalFlowLastDelivered_s)));
    deliveryAfterHorizon := (not reset) and (pre(deliveryAfterHorizon)
      or (mocapDelivered
        and mocapDeliveredRow[1] > horizonEpoch_s + epochTolerance_s)
      or (gpsDelivered
        and gpsDeliveredRow[1] > horizonEpoch_s + epochTolerance_s)
      or (magnetometerDelivered
        and magnetometerDeliveredRow[1] > horizonEpoch_s + epochTolerance_s)
      or (barometerDelivered
        and barometerDeliveredRow[1] > horizonEpoch_s + epochTolerance_s)
      or (opticalFlowDelivered
        and opticalFlowDeliveredRow[1] > horizonEpoch_s + epochTolerance_s));
    mocapLastDelivered_s := if reset then -TimestampMagnitudeLimit
      elseif mocapDelivered then mocapDeliveredRow[1]
      else pre(mocapLastDelivered_s);
    gpsLastDelivered_s := if reset then -TimestampMagnitudeLimit
      elseif gpsDelivered then gpsDeliveredRow[1]
      else pre(gpsLastDelivered_s);
    magnetometerLastDelivered_s := if reset then -TimestampMagnitudeLimit
      elseif magnetometerDelivered then magnetometerDeliveredRow[1]
      else pre(magnetometerLastDelivered_s);
    barometerLastDelivered_s := if reset then -TimestampMagnitudeLimit
      elseif barometerDelivered then barometerDeliveredRow[1]
      else pre(barometerLastDelivered_s);
    opticalFlowLastDelivered_s := if reset then -TimestampMagnitudeLimit
      elseif opticalFlowDelivered then opticalFlowDeliveredRow[1]
      else pre(opticalFlowLastDelivered_s);
    aidingRefused := refusedLateCount <> pre(refusedLateCount)
      or refusedOverflowCount <> pre(refusedOverflowCount)
      or droppedStaleCount <> pre(droppedStaleCount);
    // The residual the filter still transports over, published as its running
    // worst case. This is the number the delayed-aiding claim is measured on,
    // so it is a signal and not a comment.
    worstDeliveredAge_s := if reset then 0.0
      else max(max(max(max(max(pre(worstDeliveredAge_s), mocapAge_s),
        gpsAge_s), magnetometerAge_s), barometerAge_s), opticalFlowAge_s);
  end when;

equation
  // ---- preconditions, refused rather than rounded --------------------------
  // A residual bound tighter than one release period cannot be met by a queue
  // that ripens on releases: a measurement stamped just after one release
  // waits until the next, so its residual reaches one release period by
  // construction and every measurement would be discarded as stale. That is a
  // configuration that silently stops all aiding, which is exactly the class
  // of failure the horizon exists to make impossible.
  assert(maximumResidualAge_s >= fusionPeriod_s,
    "maximumResidualAge_s must be at least fusionPeriod_s: a measurement
     ripens on the first release at or after its own timestamp, so its
     residual reaches one release period by construction and a tighter bound
     discards every measurement as stale while reporting a stale drop rather
     than a configuration error");

  // THE HORIZON MUST COVER THE SLOWEST SOURCE, with headroom. This is the
  // relation that makes the older-than-the-horizon refusal an ANOMALY path
  // rather than a routine one: inside it, every declared source ripens with
  // time to spare and only a genuinely late packet is refused. A horizon
  // shorter than the sum turns the refusal into the normal case for the
  // slowest source, which is aiding silently switched off and reported as a
  // per-measurement timestamp fault.
  //
  // PX4 ships the same rule for the same reason: EKF2_DELAY_MAX is documented
  // as needing to be at least the largest EKF2_XXX_DELAY. This adds the
  // jitter headroom and makes it refuse rather than advise.
  assert(fusionHorizon_s >= maximumSourceDelay_s + horizonJitterMargin_s,
    "fusionHorizon_s must cover the slowest declared aiding source plus the
     jitter margin: a horizon shorter than maximumSourceDelay_s +
     horizonJitterMargin_s reaches a measurement's timestamp only after it has
     passed, so that source is refused as late on every packet and its aiding
     is off while the reported fault is per-measurement");

  // CAPACITY IS NOT ASSERTED, and the reason is worth stating rather than
  // leaving as an omission. An earlier version of this block asserted that
  // every queue could hold one horizon of its own source. With the depths
  // derived from the periods above that inequality is an identity -- the
  // depth IS the ceiling of the ratio, plus slack -- so the assertion could
  // not fail for any configuration the block admits. A supervision check that
  // cannot fire is worse than none: it reads as coverage and is not.
  //
  // What actually protects the capacity is that the depth and the rate it is
  // derived from cannot be set independently, which is why the depths are
  // final. A source that delivers faster than its declared period is the
  // remaining case, and it is not a configuration error the block can see at
  // translation; it shows up at run time as refusedOverflowCount, which is
  // published.

  annotation(Documentation(info = "<html>
    <p>The half of a delayed-fusion architecture that the merged horizon did
    not have. <code>Estimation.FusionHorizon.OutputPredictor</code> moves the
    filter's own epoch back to <code>t - D</code>; without this block the
    aiding streams still arrive at the live edge, so every measurement is
    stamped AHEAD of the instant the filter is standing on and is rejected on
    its own timestamp rule. This block holds each measurement until the fusion
    instant reaches its timestamp and hands it over then.</p>

    <p><b>What this retires.</b> Measurement-age alignment over the whole
    transport latency. On the live-edge path a GPS fix about 100 ms old was
    fused by retrodicting the nominal state over that age and transporting the
    measurement Jacobian back through <code>Phi(-age)</code>, admitted by a
    <code>maximumAidingDelay_s</code> of 0.25 s. Here the fix is fused at the
    instant it was taken. What REMAINS is sub-window alignment: a measurement
    timestamp falls between two fusion instants, so the residual is in
    <code>[0, fusionPeriod_s)</code> and the same transport runs over that.
    The bound the transport error is charged against therefore falls from
    0.25 s to one release period, 0.01 s at the flight lattice, and the
    residual is published as <code>worstDeliveredAge_s</code> rather than
    argued.</p>

    <p><b>What retires by construction rather than by code.</b> The gain and
    the Joseph posterior on the live-edge path neglect the process noise
    accumulated between a measurement's timestamp and the instant it was fused
    at. That window is the residual above, so the neglected term shrinks with
    it by the same factor of twenty-five; it is not removed, and this
    documentation does not claim it is. The claim it does support is that the
    window is now a PARAMETER of the release lattice rather than whatever age
    a packet happened to arrive with.</p>

    <p><b>Ordering.</b> Each source's queue is a FIFO and delivers in
    timestamp order, oldest first. A measurement that arrives out of order
    relative to one already admitted from the same source is refused by the
    novelty rule, exactly as the filters refuse it today; a measurement that
    arrives after the fusion instant has passed it is refused with
    <code>AidingRefusedLate</code>. Between sources there is no ordering to
    keep: each queue is independent and the filter's own correction dispatch
    ranks the sources by authority.</p>

    <p><b>One hazard this does not guard, stated rather than left to be
    found.</b> A measurement stamped in the FUTURE -- past the fusion instant
    by more than the horizon, which no honest sensor produces and a sensor with
    a broken clock does -- occupies the head of its queue and cannot ripen
    until the fusion instant reaches it. The queue behind it fills, and from
    then on every arrival from that source is refused for overflow. The source
    is effectively blind until the epoch catches up.</p>
    <p>It is not silent: <code>refusedOverflowCount</code> climbs on every
    refused arrival, which is the bar every other failure here is held to. It
    is also not guarded, and guarding it would mean declaring how far ahead of
    the fusion instant a timestamp may plausibly be, which is another parameter
    and another negative test. The bound already exists in one direction --
    <code>maximumResidualAge_s</code> refuses a measurement the horizon has
    passed -- and the symmetric one belongs with it rather than bolted on
    here.</p>

    <p><b>Nothing here knows what a filter is.</b> No covariance, no error
    state, no tangent, no injection: this block moves timestamped records in
    time and does nothing else, so it serves an additive-bias ESKF and a
    manifold UKF without either being privileged, which is the same property
    that makes the delta ring estimator-agnostic.</p>
  </html>"));
end AidingBuffer;
