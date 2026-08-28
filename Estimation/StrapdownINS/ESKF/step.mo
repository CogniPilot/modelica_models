within Estimation.StrapdownINS.ESKF;

function step
  "One sampled prediction and at most one aiding correction"
  input Boolean initializedPrevious;
  input Estimation.StrapdownINS.ESKF.State previous;
  input Boolean reset;
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.MagnetometerSample magnetometer;
  input Avionics.BarometerSample barometer;
  input Avionics.OpticalFlowSample opticalFlow;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt;
  input Estimation.StrapdownINS.ESKF.Tuning tuning;
  input Integer consecutiveRejectionsPrevious;
  input Real rejectionElapsedPrevious_s;
  input Integer mocapRejectionsPrevious;
  input Integer gpsRejectionsPrevious;
  input Integer opticalFlowRejectionsPrevious;
  input Integer anchorSourcePrevious
    "Latched anchor from the previous tick. Latched, not momentary: a
     source going quiet between its own samples must not read as a change";
  input Real mocapStalePrevious_s;
  input Real gpsStalePrevious_s;
  input Real opticalFlowStalePrevious_s;
  input Real imuAngularVelocityHeldPrevious_rad_s[3];
  input Real imuSpecificForceHeldPrevious_m_s2[3];
  input Real imuTimestampHeldPrevious_s;
  input Real mocapTimestampConsumedPrevious_s;
  input Real gpsTimestampConsumedPrevious_s;
  input Real magnetometerTimestampConsumedPrevious_s;
  input Real barometerTimestampConsumedPrevious_s;
  input Real opticalFlowTimestampConsumedPrevious_s;
  output Real positionNext[3];
  output Real velocityNext[3];
  output Real quaternionNext[4];
  output Real gyroscopeBiasNext[3];
  output Real accelerometerBiasNext[3];
  output Estimation.StrapdownINS.ESKF.Covariance covarianceNext;
  output Boolean initializedNext;
  output Boolean predictionAccepted;
  output Boolean mocapCorrectionAccepted;
  output Boolean gpsPositionCorrectionAccepted;
  output Boolean gpsVelocityCorrectionAccepted;
  output Boolean magnetometerCorrectionAccepted;
  output Boolean barometerCorrectionAccepted;
  output Boolean opticalFlowCorrectionAccepted;
  output Integer consecutiveRejectionsNext;
  output Real rejectionElapsedNext_s
    "Wall-clock time the anchor source has held without moving the state.
     Advanced on estimator ticks, not on aiding attempts and not on the
     sensors` valid duty cycle. Cleared when the anchor accepts and when a
     genuinely different source takes over the anchor; FROZEN, not
     cleared, while no source is delivering";
  output Integer recoveryStage
    "Estimation.StrapdownINS.Recovery* code for the automatic
     recovery ladder driven by sustained rejection";
  output Integer correctionOutcome
    "Estimation.StrapdownINS.Correction* code for this tick's
     attempted aiding correction";
  output Integer correctionSource
    "Estimation.StrapdownINS.Source* code the outcome refers to";
  output Real normalizedInnovationSquared
    "NIS for this tick's attempted correction, or zero when none was attempted";
  output Boolean estimateValid
    "Whether the published navigation estimate may be consumed";
  output Integer mocapRejectionsNext
    "Consecutive rejections of the mocap source alone";
  output Integer gpsRejectionsNext
    "Consecutive rejections of the GPS source alone";
  output Integer opticalFlowRejectionsNext
    "Consecutive rejections of the optical-flow source alone";
  output Integer anchorSourceNext
    "Estimation.StrapdownINS.Source* code of the anchor in force";
  output Real mocapStaleNext_s;
  output Real gpsStaleNext_s;
  output Real opticalFlowStaleNext_s;
  output Real imuAngularVelocityHeldNext_rad_s[3]
    "IMU angular velocity to PUBLISH: the current sample when it is
     finite, otherwise the last finite one";
  output Real imuSpecificForceHeldNext_m_s2[3]
    "IMU specific force to PUBLISH, same hold rule";
  output Real imuTimestampHeldNext_s
    "IMU timestamp to PUBLISH, same hold rule";
  output Boolean imuPayloadHeld
    "True on a tick whose published IMU-derived outputs are held from an
     earlier sample because the current one was not finite";
  output Real mocapTimestampConsumedNext_s;
  output Real gpsTimestampConsumedNext_s;
  output Real magnetometerTimestampConsumedNext_s;
  output Real barometerTimestampConsumedNext_s;
  output Real opticalFlowTimestampConsumedNext_s;
protected
  Estimation.StrapdownINS.ESKF.State prior;
  Estimation.StrapdownINS.ESKF.State working;
  Real initializationPosition[3];
  Real initializationQuaternion[4];
  Boolean correctionAttempted;
  Boolean correctionAccepted;
  Boolean aidingLive;
  Boolean inflateCovarianceNow;
  Boolean aidingDivergent;
  Boolean anchorExclusive;
  Boolean imuUsable;
  Boolean imuPayloadFinite;
  Boolean ladderConfigured;
  Integer anchorPresent;
  Boolean mocapSeedUsable;
  Boolean gpsSeedUsable;
  Boolean magnetometerSeedUsable;
  Boolean alignmentAccepted;
  Boolean mocapNew;
  Boolean gpsNew;
  Boolean magnetometerNew;
  Boolean barometerNew;
  Boolean opticalFlowNew;
  Real mocapSeedQuaternionNorm;
algorithm
  predictionAccepted := false;
  mocapCorrectionAccepted := false;
  gpsPositionCorrectionAccepted := false;
  gpsVelocityCorrectionAccepted := false;
  magnetometerCorrectionAccepted := false;
  barometerCorrectionAccepted := false;
  opticalFlowCorrectionAccepted := false;
  correctionAttempted := false;
  correctionAccepted := false;
  correctionOutcome := CorrectionNotAttempted;
  correctionSource := SourceNone;
  normalizedInnovationSquared := 0.0;
  consecutiveRejectionsNext := 0;
  rejectionElapsedNext_s := 0.0;
  // A commanded reset and first initialization clear every source's
  // history: the state they describe no longer exists.
  mocapRejectionsNext := 0;
  gpsRejectionsNext := 0;
  opticalFlowRejectionsNext := 0;
  mocapTimestampConsumedNext_s := mocapTimestampConsumedPrevious_s;
  gpsTimestampConsumedNext_s := gpsTimestampConsumedPrevious_s;
  magnetometerTimestampConsumedNext_s :=
    magnetometerTimestampConsumedPrevious_s;
  barometerTimestampConsumedNext_s := barometerTimestampConsumedPrevious_s;
  opticalFlowTimestampConsumedNext_s :=
    opticalFlowTimestampConsumedPrevious_s;

  // AFFIRMATIVE ADMISSION ON THE PREDICTION SIDE. The aiding path has an
  // innovation gate in front of it; the IMU path has nothing, so a single
  // non-finite inertial sample propagates straight through predictNominal
  // into the state and stays there -- and with the automatic re-seed
  // removed there is no longer anything that would eventually flush it.
  // Measured: one NaN accelerometer sample left position NaN with no
  // recovery over the following 500 clean ticks. The publication guard
  // reports that honestly as estimate.valid = false, but not ingesting it
  // is a stronger remedy than reporting it.
  //
  // A non-finite sample is treated exactly as a missing one: hold the
  // nominal state and grow the covariance. That path already exists for a
  // dropout and is the right response, because an unusable sample and an
  // absent sample carry the same information.
  //
  // Publication usability is valid AND finite, not finite alone. A sample
  // flagged invalid still carries numbers, and those numbers can be finite
  // garbage; publishing them because they happen not to be NaN would pass
  // on data the driver has explicitly disowned, while prediction refuses
  // the very same sample. `fresh` is deliberately NOT required: a sample
  // that is valid but not fresh IS the previous sample, so holding it and
  // publishing it are the same value, and demanding freshness would
  // substitute held values on every non-fresh tick for no gain.
  imuPayloadFinite := imu.valid
    and abs(imu.timestamp_s) < FiniteMagnitudeLimit
    and imu.integrationTime_s > 0.0
    and imu.integrationTime_s < FiniteMagnitudeLimit;
  for i in 1:3 loop
    imuPayloadFinite := imuPayloadFinite
      and abs(imu.angularVelocityBodyFlu_rad_s[i]) < FiniteMagnitudeLimit
      and abs(imu.specificForceBodyFlu_m_s2[i]) < FiniteMagnitudeLimit
      and abs(imu.deltaAngleBodyFlu_rad[i]) < FiniteMagnitudeLimit
      and abs(imu.deltaVelocityBodyFlu_m_s[i]) < FiniteMagnitudeLimit
      and abs(imu.deltaPositionBodyFlu_m[i]) < FiniteMagnitudeLimit
      and abs(imu.gyroscopeBiasLinearizationBodyFlu_rad_s[i])
        < FiniteMagnitudeLimit
      and abs(imu.accelerometerBiasLinearizationBodyFlu_m_s2[i])
        < FiniteMagnitudeLimit;
    for j in 1:3 loop
      imuPayloadFinite := imuPayloadFinite
        and abs(imu.deltaRotationGyroscopeBiasJacobian_s[i, j])
          < FiniteMagnitudeLimit
        and abs(imu.deltaVelocityGyroscopeBiasJacobian_m[i, j])
          < FiniteMagnitudeLimit
        and abs(imu.deltaVelocityAccelerometerBiasJacobian_s[i, j])
          < FiniteMagnitudeLimit
        and abs(imu.deltaPositionGyroscopeBiasJacobian_m_s[i, j])
          < FiniteMagnitudeLimit
        and abs(imu.deltaPositionAccelerometerBiasJacobian_s2[i, j])
          < FiniteMagnitudeLimit;
    end for;
  end for;
  for i in 1:4 loop
    imuPayloadFinite := imuPayloadFinite
      and abs(imu.deltaQuaternionBodyFlu[i]) < FiniteMagnitudeLimit;
  end for;
  // Cross-clock delivery is level-triggered: the producer holds the most
  // recent packet, and timestamp novelty makes consumption exactly once.
  imuUsable := imuPayloadFinite
    and imu.timestamp_s > imuTimestampHeldPrevious_s + 1.0e-9;
  mocapNew := mocap.valid
    and abs(mocap.timestamp_s) < FiniteMagnitudeLimit
    and mocap.timestamp_s > mocapTimestampConsumedPrevious_s + 1.0e-9;
  gpsNew := gps.valid
    and abs(gps.timestamp_s) < FiniteMagnitudeLimit
    and gps.timestamp_s > gpsTimestampConsumedPrevious_s + 1.0e-9;
  magnetometerNew := magnetometer.valid
    and abs(magnetometer.timestamp_s) < FiniteMagnitudeLimit
    and magnetometer.timestamp_s
      > magnetometerTimestampConsumedPrevious_s + 1.0e-9;
  barometerNew := barometer.valid
    and abs(barometer.timestamp_s) < FiniteMagnitudeLimit
    and barometer.timestamp_s > barometerTimestampConsumedPrevious_s + 1.0e-9;
  opticalFlowNew := opticalFlow.valid
    and abs(opticalFlow.timestamp_s) < FiniteMagnitudeLimit
    and opticalFlow.timestamp_s
      > opticalFlowTimestampConsumedPrevious_s + 1.0e-9;

  // Initialization payloads must satisfy the same finite-data contract as
  // correction payloads before they can seed persistent state.
  mocapSeedUsable := mocap.valid;
  for i in 1:3 loop
    mocapSeedUsable := mocapSeedUsable
      and abs(mocap.positionWorldEnu_m[i]) < FiniteMagnitudeLimit;
  end for;
  mocapSeedQuaternionNorm := 0.0;
  for i in 1:4 loop
    mocapSeedUsable := mocapSeedUsable
      and abs(mocap.quaternionWorldBody[i]) < FiniteMagnitudeLimit;
    mocapSeedQuaternionNorm := mocapSeedQuaternionNorm
      + mocap.quaternionWorldBody[i] * mocap.quaternionWorldBody[i];
  end for;
  // Normalizability, not just finiteness: a finite but vanishing
  // quaternion survives every magnitude check and then becomes a
  // full-scale garbage rotation when initialize() normalizes it.
  mocapSeedUsable := mocapSeedUsable
    and sqrt(mocapSeedQuaternionNorm) >= MinimumSeedQuaternionNorm;
  gpsSeedUsable := gps.valid and gps.positionValid;
  for i in 1:3 loop
    gpsSeedUsable := gpsSeedUsable
      and abs(gps.positionWorldEnu_m[i]) < FiniteMagnitudeLimit;
  end for;
  magnetometerSeedUsable := magnetometer.valid;
  for i in 1:3 loop
    magnetometerSeedUsable := magnetometerSeedUsable
      and abs(magnetometer.magneticFieldBodyFlu_T[i]) < FiniteMagnitudeLimit
      and abs(tuning.localMagneticFieldWorldEnu_T[i]) < FiniteMagnitudeLimit
      and magnetometer.covarianceBody_T2[i, i] > 0.0;
  end for;

  // PUBLICATION HOLD FOR THE IMU PASSTHROUGH OUTPUTS.
  //
  // Rejecting a non-finite IMU sample for PREDICTION is not enough, because
  // three published fields are not computed from the state at all. Angular
  // velocity, world acceleration and the estimate timestamp are computed
  // from the RAW sample -- corrected only by a bias -- so they bypass every
  // guard the state path has. Observed: prediction correctly refused the
  // sample, and the same tick published bodyRate = NaN and accel = NaN
  // beside a perfectly finite nominal position, with valid = 1. The
  // finiteness guard on the nominal state cannot see this: the state IS
  // finite. It is the payload that is not.
  //
  // The contract is hold-last-finite: publish the most recent finite sample
  // rather than the bad one. Chosen over zeroing because these outputs feed
  // the rate loop, and a zero body rate is not a neutral value there -- it
  // asserts "not rotating", which is a lie a controller will act on. A
  // one-tick-stale rate is what a dropped sample would have produced
  // anyway, so the substitution is inside the noise of normal operation.
  // Before any finite sample has ever arrived the held values are the
  // block's zero start values, which are bounded and honest at rest.
  //
  // NO NaN ESCAPES ON ANY INPUT: every published field is now either a
  // state field, which nominalStateFinite guards, or one of these three,
  // which this holds.
  imuPayloadHeld := not imuPayloadFinite;
  if imuPayloadFinite then
    imuAngularVelocityHeldNext_rad_s := imu.angularVelocityBodyFlu_rad_s;
    imuSpecificForceHeldNext_m_s2 := imu.specificForceBodyFlu_m_s2;
    imuTimestampHeldNext_s := imu.timestamp_s;
  else
    imuAngularVelocityHeldNext_rad_s := imuAngularVelocityHeldPrevious_rad_s;
    imuSpecificForceHeldNext_m_s2 := imuSpecificForceHeldPrevious_m_s2;
    imuTimestampHeldNext_s := imuTimestampHeldPrevious_s;
  end if;

  // SOURCE STALENESS, measured in wall-clock seconds since each source
  // last delivered a usable fresh sample.
  //
  // Availability cannot be read off `valid` on the current tick. The
  // deployed GPS driver PULSES `valid` -- it is asserted only on the
  // ticks that carry a fix and deasserted in between -- so a rule keyed
  // on `valid` sees GPS as absent for 99 of every 100 ticks at 10 Hz
  // under a 1 kHz estimator. That made the anchor flap to optical flow in
  // every gap, and since flow agreed with the corrupt state it accepted,
  // which cleared the rejection clock, so the ladder never fired and
  // truthful GPS stayed locked out for the entire run. The clock was
  // running on the sensors' valid duty cycle, not on time.
  //
  // A staleness timer is the duty-cycle-independent statement of the same
  // idea: a source is available while it is still delivering, whatever
  // its `valid` waveform looks like between deliveries. It also demotes
  // the opposite pathology -- a source wired permanently valid but never
  // fresh, which the deployed mocap wrapper can produce because it sets
  // valid and fresh independently. Such a source would otherwise hold the
  // anchor forever without ever being attempted, and nothing could clear
  // the clock.
  mocapStaleNext_s := if mocapNew then 0.0
    else mocapStalePrevious_s + dt;
  gpsStaleNext_s := if gpsNew
      and (gps.positionValid or gps.velocityValid) then 0.0
    else gpsStalePrevious_s + dt;
  opticalFlowStaleNext_s := if opticalFlowNew
    then 0.0 else opticalFlowStalePrevious_s + dt;

  // THE ANCHOR SOURCE: the most authoritative source still delivering.
  // The existing correction priority already ranks the sources by
  // authority -- mocap is external ground truth, GPS is an absolute
  // position fix, optical flow is a relative velocity inference that
  // cannot observe position at all -- so the same order names the source
  // the filter is supposed to be anchored to.
  anchorPresent := if mocapStaleNext_s <= tuning.aidingStaleTimeout_s then
      SourceMocap
    elseif gpsStaleNext_s <= tuning.aidingStaleTimeout_s then SourceGps
    elseif opticalFlowStaleNext_s <= tuning.aidingStaleTimeout_s then
      SourceOpticalFlow
    else SourceNone;

  // THE ANCHOR IS LATCHED. Losing the current anchor does not erase which
  // source the filter is anchored TO; it only means that source is not
  // delivering right now.
  //
  // Keying the clock to the momentary anchor made the ladder starvable by
  // any source slower than the staleness timeout. A source lying at
  // 1.67 Hz against a 0.5 s timeout goes stale between its own fixes, so
  // the anchor oscillated source -> none -> source, every return counted
  // as an anchor change, and each change cleared the clock: measured 100
  // anchor changes and the ladder never firing once, at 2 km of position
  // error, with estimate.valid = 1. Degraded GPS below 2 Hz is ordinary,
  // so that is a hole an attacker does not even need to find.
  //
  // Latching closes it without a rate assumption anywhere: the clock is
  // cleared only when a genuinely DIFFERENT source takes over, which is
  // the one case where the elapsed time describes something that is no
  // longer in charge.
  anchorSourceNext := if anchorPresent <> SourceNone then anchorPresent
    else anchorSourcePrevious;
  aidingLive := anchorPresent <> SourceNone;

  // AUTOMATIC RECOVERY LADDER.
  //
  // Both stages are timed in WALL-CLOCK SECONDS on the ANCHOR SOURCE.
  // The tick count they replace conflated "how much aiding has been
  // rejected" with "how fast is this block wired": the same
  // rejectedCorrectionLimit of 50 meant 0.5-2.5 s at the 20-100 Hz aiding
  // the model was designed around, but 51 ms at the 1 kHz RDD2 estimator
  // rate with aiding asserted every tick. Seconds mean the same thing at
  // every wiring and every aiding rate.
  //
  // STAGE 1, at tuning.covarianceInflateWindow_s: begin RAMPING the
  // position and velocity covariance toward the declared mission
  // envelope, and DO NOT TOUCH THE STATE. Withdrawing confidence asserts
  // nothing that could be wrong, and it is sufficient whenever recovery
  // is possible at all: as the gate widens, a genuinely good fix that had
  // been locked out by an over-tight covariance is admitted through the
  // ordinary correction path, continuously and with no discontinuity in
  // the published estimate. The ramp rather than a jump is what keeps
  // that from becoming adoption of whatever arrives first; see
  // inflateCovariance for the measured failure the jump produced.
  //
  // STAGE 2, at tuning.aidingDivergentWindow_s: the covariance has
  // reached the envelope and the anchor still does not fit. This is
  // REPORTED and nothing more. The estimator does not adopt the fix, does
  // not re-seed itself, and does not withdraw estimate.valid.
  //
  // It previously withdrew estimate.valid, on the reasoning that handing
  // over to a failsafe beats guessing. That reasoning was right and the
  // mechanism was wrong: on the deployed stack estimate.valid = false
  // clears RatesValid, zeroes the motors and LATCHES the fault, in every
  // mode including ACRO. One tick of it in the air is a crash, not a
  // handover -- and stage 2 is reachable from causes as ordinary as an
  // anchor that stays valid but stops being fresh. The degraded signal
  // belongs in the status record, where a wrapper can map it to a
  // non-latching mode demotion; estimate.valid stays a statement about
  // whether the published numbers are numbers.
  //
  // What remains true is that the estimator still never re-seeds from a
  // stream it is rejecting. Re-seeding position, attitude and velocity
  // stays available through the commanded `reset` input, where it is a
  // deliberate act rather than a filter's guess.
  // AFFIRMATIVE CONFIGURATION ADMISSION. The recovery ladder must prove its
  // configuration is usable before it is allowed to act on it; this keeps
  // the safety property local to the algorithm in every execution target.
  //
  // Both failure modes this replaces are silent. A NaN window makes every
  // comparison against it false, so the ladder simply never fires and
  // nothing says why -- the exact predicate-exhaustion defect this whole
  // change exists to remove, reappearing in the tuning surface. And a
  // `min` attribute does not reject an out-of-range value, it CLAMPS it,
  // turning a configuration error into a legal but harmful setting: a
  // window clamped to one millisecond arms the ladder permanently on a
  // healthy filter. The `min` attributes have been removed for that
  // reason; an unusable value now disables the ladder and is reported.
  ladderConfigured := tuning.covarianceInflateWindow_s > 0.0
    and tuning.covarianceInflateWindow_s < FiniteMagnitudeLimit
    and tuning.covarianceInflateTimeConstant_s > 0.0
    and tuning.covarianceInflateTimeConstant_s < FiniteMagnitudeLimit
    and tuning.aidingStaleTimeout_s > 0.0
    and tuning.aidingStaleTimeout_s < FiniteMagnitudeLimit
    and tuning.aidingDivergentWindow_s > tuning.covarianceInflateWindow_s
    and tuning.aidingDivergentWindow_s < FiniteMagnitudeLimit;

  inflateCovarianceNow := ladderConfigured and initializedPrevious
    and not reset
    and rejectionElapsedPrevious_s >= tuning.covarianceInflateWindow_s;
  aidingDivergent := ladderConfigured and initializedPrevious and not reset
    and rejectionElapsedPrevious_s >= tuning.aidingDivergentWindow_s;
  recoveryStage := if not ladderConfigured then RecoveryMisconfigured
    elseif aidingDivergent then RecoveryAidingDivergent
    elseif inflateCovarianceNow then RecoveryCovarianceInflated
    else RecoveryNominal;
  // Exclusivity keys to the source PRESENT this tick, not the latched one:
  // while the anchor is between samples there is nothing for it to be
  // exclusive against, and blocking the others would simply stop all
  // correction during re-acquisition.
  anchorExclusive := recoveryStage == RecoveryCovarianceInflated
    and anchorPresent <> SourceNone;


  // The declared initialization policy now runs ONLY at startup and on a
  // commanded reset. It seeds velocity from a parameter and attitude from
  // the identity quaternion when no mocap is present, which is correct on
  // the ground where it was written to run, and is exactly what must
  // never happen mid-flight.
  if not initializedPrevious or reset then
    // AFFIRMATIVE ADMISSION ON THE INITIALIZATION PATH.
    //
    // This branch writes the nominal state DIRECTLY from an aiding payload,
    // with no innovation gate and no Cholesky in front of it -- the two
    // things that guard every other route into the state. A source flagged
    // valid while carrying a NaN therefore seeded the filter with that NaN,
    // and because nothing downstream can subtract a NaN back out, the state
    // stayed poisoned permanently: measured as position NaN with
    // estimate.valid = 0 still, after 500 subsequent clean fixes.
    //
    // It is the same defect class the correction path was hardened against,
    // surviving on the one path that bypasses the correction path entirely.
    // A seed must prove it is finite before it is written, and a source
    // that cannot falls through to the next candidate exactly as though it
    // were absent -- ending at the declared initial state, which is a
    // parameter and therefore always finite.
    initializationPosition := if mocapSeedUsable then
        mocap.positionWorldEnu_m
      elseif gpsSeedUsable then
        gps.positionWorldEnu_m
      else
        tuning.initialState.positionWorldEnu_m;
    if mocapSeedUsable then
      initializationQuaternion := mocap.quaternionWorldBody;
      alignmentAccepted := true;
    elseif imuUsable and magnetometerSeedUsable then
      (initializationQuaternion, alignmentAccepted) :=
        Estimation.StrapdownINS.initialAlignmentQuaternion(
          imu.specificForceBodyFlu_m_s2,
          magnetometer.magneticFieldBodyFlu_T,
          tuning.localMagneticFieldWorldEnu_T,
          tuning.initialState.quaternionWorldBody);
    else
      initializationQuaternion := tuning.initialState.quaternionWorldBody;
      alignmentAccepted := false;
    end if;
    magnetometerCorrectionAccepted := magnetometerSeedUsable
      and alignmentAccepted
      and not mocapSeedUsable;
    working := initialize(
      initializationPosition,
      initializationQuaternion,
      tuning.initialVariances,
      tuning.initialState.velocityWorldEnu_m_s,
      tuning.initialState.gyroscopeBiasBodyFlu_rad_s,
      tuning.initialState.accelerometerBiasBodyFlu_m_s2);
  else
    prior := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=previous.positionWorldEnu_m,
      velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
      quaternionWorldBody=previous.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2,
      // STAGE 1 of the recovery ladder is applied here, to the covariance
      // carried into this tick, so the freshly widened uncertainty is
      // already in force for this tick's own gate test rather than one
      // tick later. Nothing else about the state is touched.
      covariance=if inflateCovarianceNow then
          inflateCovariance(previous.covariance, tuning.varianceLimits,
            dt, tuning.covarianceInflateTimeConstant_s)
        else
          previous.covariance);
    // AFFIRMATIVE ADMISSION ON THE PREDICTION SIDE TOO. The aiding path
    // has an innovation gate in front of it; the IMU path has nothing, so
    // a single non-finite inertial sample propagates straight through
    // predictNominal into the state and stays there -- and with the
    // automatic re-seed removed there is no longer anything that would
    // eventually flush it. Measured: one NaN accelerometer sample left
    // position NaN with no recovery over the following 500 clean ticks.
    // The publication guard reports that honestly as estimate.valid =
    // false, but reporting is a weaker remedy than not ingesting it.
    //
    // A non-finite sample is treated exactly as a missing one: hold the
    // nominal state and grow the covariance. That path already exists for
    // a dropout and is the right response, because an unusable sample and
    // an absent sample carry the same information.
    if imuUsable then
      working := predictPreintegrated(
        prior, imu, gravityWorldEnu_m_s2, tuning.processNoise);
      predictionAccepted := true;
    elseif not imuPayloadFinite then
      // No IMU this tick: the nominal state is held, but the covariance
      // must still GROW by the process noise for the elapsed interval.
      // Holding the covariance unchanged (the previous behaviour) made
      // the filter claim that standing still for an unknown interval
      // costs no certainty, so any aiding that arrived during an IMU
      // dropout contracted the covariance with nothing ever re-expanding
      // it -- measured monotonic shrink over a blind window. An estimator
      // that gets MORE confident while blind is the precondition for
      // gating out the very measurements that would fix it.
      working := Estimation.StrapdownINS.ESKF.State(
        positionWorldEnu_m=prior.positionWorldEnu_m,
        velocityWorldEnu_m_s=prior.velocityWorldEnu_m_s,
        quaternionWorldBody=prior.quaternionWorldBody,
        gyroscopeBiasBodyFlu_rad_s=prior.gyroscopeBiasBodyFlu_rad_s,
        accelerometerBiasBodyFlu_m_s2=
          prior.accelerometerBiasBodyFlu_m_s2,
        covariance=holdCovariance(
          prior.covariance, dt, tuning.processNoise));
    else
      // A valid held packet means the high-rate preintegrator is still
      // accumulating the next delta-angle/delta-velocity observation. It is
      // neither a new prediction nor an IMU dropout, so do not propagate the
      // nominal state or add process noise a second time.
      working := Estimation.StrapdownINS.ESKF.State(
        positionWorldEnu_m=prior.positionWorldEnu_m,
        velocityWorldEnu_m_s=prior.velocityWorldEnu_m_s,
        quaternionWorldBody=prior.quaternionWorldBody,
        gyroscopeBiasBodyFlu_rad_s=prior.gyroscopeBiasBodyFlu_rad_s,
        accelerometerBiasBodyFlu_m_s2=
          prior.accelerometerBiasBodyFlu_m_s2,
        covariance=prior.covariance);
    end if;
    working := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=working.positionWorldEnu_m,
      velocityWorldEnu_m_s=working.velocityWorldEnu_m_s,
      quaternionWorldBody=working.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=working.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=working.accelerometerBiasBodyFlu_m_s2,
      covariance=limitCovariance(working.covariance, tuning.varianceLimits));

    // ANCHOR EXCLUSIVITY, and only during stage 1.
    //
    // Stage 1 deliberately widens the gate, and a widened gate is open to
    // every source at once, so whichever samples FASTEST re-anchors the
    // state rather than whichever is most AUTHORITATIVE. Witnessed: 50 Hz
    // optical flow with a 10x velocity bias recaptured the filter in every
    // gap between 10 Hz GPS samples, pinning the estimate at a tenth of
    // true speed while truthful GPS stayed gated out -- all nine
    // re-acquisition sweeps failed, the worst at -179 m/s against a 7 m/s
    // truth. Giving the anchor exclusive use of its own re-acquisition
    // window fixes that.
    //
    // It is confined to stage 1 because the exclusivity is a claim that
    // the anchor deserves first refusal, and stage 2 is precisely the
    // finding that it does not: an anchor that still cannot fit at full
    // envelope has forfeited it. Left in force at stage 2 the rule
    // inverted into its own failure mode -- a GPS lying by 2 km starved a
    // truthful 50 Hz flow, blocking 464 of 500 samples and collapsing the
    // velocity estimate to -0.21 m/s against a 4 m/s truth. Isolation of a
    // broken high-authority source is the rejection CLOCK being anchored
    // to it; that works without any veto over the other sources.
    if mocapNew
        and (not anchorExclusive or anchorPresent == SourceMocap) then
      (working, mocapCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctMocap(working, mocap, tuning.innovationGate,
          imuTimestampHeldNext_s - mocap.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s);
      correctionAttempted := true;
      correctionAccepted := mocapCorrectionAccepted;
      correctionSource := SourceMocap;
      mocapTimestampConsumedNext_s := mocap.timestamp_s;
    elseif gpsNew and gps.positionValid
        and gps.velocityValid
        and (not anchorExclusive or anchorPresent == SourceGps) then
      (working, gpsPositionCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctGps(working, gps, tuning.innovationGate,
          imuTimestampHeldNext_s - gps.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s);
      gpsVelocityCorrectionAccepted := gpsPositionCorrectionAccepted;
      correctionAttempted := true;
      correctionAccepted := gpsPositionCorrectionAccepted;
      correctionSource := SourceGps;
      gpsTimestampConsumedNext_s := gps.timestamp_s;
    elseif gpsNew and gps.positionValid
        and (not anchorExclusive or anchorPresent == SourceGps) then
      (working, gpsPositionCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctGpsPosition(working, gps, tuning.innovationGate,
          imuTimestampHeldNext_s - gps.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s);
      correctionAttempted := true;
      correctionAccepted := gpsPositionCorrectionAccepted;
      correctionSource := SourceGps;
      gpsTimestampConsumedNext_s := gps.timestamp_s;
    elseif gpsNew and gps.velocityValid
        and (not anchorExclusive or anchorPresent == SourceGps) then
      (working, gpsVelocityCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctGpsVelocity(working, gps, tuning.innovationGate,
          imuTimestampHeldNext_s - gps.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s);
      correctionAttempted := true;
      correctionAccepted := gpsVelocityCorrectionAccepted;
      correctionSource := SourceGps;
      gpsTimestampConsumedNext_s := gps.timestamp_s;
    elseif barometerNew then
      (working, barometerCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctBarometer(working, barometer,
          tuning.barometerBias_m, tuning.barometerBiasVariance_m2,
          tuning.innovationGate,
          imuTimestampHeldNext_s - barometer.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s);
      correctionAttempted := true;
      correctionAccepted := barometerCorrectionAccepted;
      correctionSource := SourceBarometer;
      barometerTimestampConsumedNext_s := barometer.timestamp_s;
    elseif opticalFlowNew
        and (not anchorExclusive or anchorPresent == SourceOpticalFlow) then
      (working, opticalFlowCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctOpticalFlow(working, opticalFlow, tuning.innovationGate,
          imuTimestampHeldNext_s - opticalFlow.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s,
          tuning.minimumOpticalFlowQuality,
          tuning.minimumOpticalFlowGroundDistance_m);
      correctionAttempted := true;
      correctionAccepted := opticalFlowCorrectionAccepted;
      correctionSource := SourceOpticalFlow;
      opticalFlowTimestampConsumedNext_s := opticalFlow.timestamp_s;
    elseif magnetometerNew then
      (working, magnetometerCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctMagnetometer(working, magnetometer,
          tuning.localMagneticFieldWorldEnu_T, tuning.innovationGate,
          imuTimestampHeldNext_s - magnetometer.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s);
      correctionAttempted := true;
      correctionAccepted := magnetometerCorrectionAccepted;
      correctionSource := SourceMagnetometer;
      magnetometerTimestampConsumedNext_s := magnetometer.timestamp_s;
    end if;

    // RECOVERY TIMER, measured on the ANCHOR SOURCE ONLY.
    //
    // It tracks how long the most authoritative live source has been
    // unable to move the state. Tying it to the anchor rather than to
    // "any acceptance at all" is what stops a broken lower-authority
    // sensor from hiding a broken higher-authority one: a shared timer
    // that any acceptance resets means a 50 Hz optical-flow sensor
    // cheerfully agreeing with a wrong state clears the timer faster
    // than a 10 Hz GPS disagreeing with it can advance it, so the ladder
    // never fires and truthful GPS stays gated out indefinitely. That
    // deadlock was witnessed as a permanent loss of position with
    // estimate.valid still true.
    //
    // Only the anchor's own verdict counts. An accepted correction from
    // a lower-authority source neither clears the timer nor advances it.
    //
    // The clock advances on ESTIMATOR TICKS, not on aiding attempts and
    // not on the sensors' `valid` duty cycle. An earlier form advanced it
    // only while `aidingLive` -- read from `valid` on the current tick --
    // and only once a rejection was already outstanding. Under the
    // deployed pulsed GPS wiring that made the elapsed time a function of
    // the sensor's duty cycle rather than of time, and it never reached
    // the window at all: measured stage1 never firing and GPS locked out
    // for a whole 10 s run. Counting ticks while the anchor holds and has
    // not accepted is what makes "one second" mean one second.
    //
    // Three things clear it, and all three mean the same thing -- the
    // question the clock was asking no longer applies:
    //   - the anchor accepted a correction, so it can move the state;
    //   - the anchor CHANGED, so the elapsed time describes a source that
    //     is no longer in charge;
    //   - there is no anchor at all, so nothing is being rejected. A total
    //     aiding dropout must not fire a ladder about aiding disagreement.
    if anchorPresent == SourceNone then
      // Total aiding loss FREEZES the clock; it does not clear it.
      // Clearing here was wrong in a way that reads as correct: losing
      // every source is not evidence that a divergence has been resolved,
      // it is a strictly worse situation, and de-escalating the report to
      // nominal at that moment tells the wrapper the opposite of the
      // truth. Measured: a divergent stage fell back to nominal 403 ms
      // after total dropout, while completely unaided. Frozen, the report
      // persists and resumes from where it stood if aiding returns.
      consecutiveRejectionsNext := consecutiveRejectionsPrevious;
      rejectionElapsedNext_s := rejectionElapsedPrevious_s;
    elseif anchorSourceNext <> anchorSourcePrevious then
      // A different live source has taken over. Its elapsed time describes
      // the source that just lost the anchor, so it does not apply.
      consecutiveRejectionsNext := 0;
      rejectionElapsedNext_s := 0.0;
    elseif correctionAttempted and correctionSource == anchorPresent then
      if correctionAccepted then
        consecutiveRejectionsNext := 0;
        rejectionElapsedNext_s := 0.0;
      else
        consecutiveRejectionsNext := consecutiveRejectionsPrevious + 1;
        rejectionElapsedNext_s := rejectionElapsedPrevious_s + dt;
      end if;
    else
      // The anchor held but said nothing this tick, whether because it is
      // between samples or because it has gone quiet. Either way the
      // state has spent another tick unanchored, so the clock advances.
      consecutiveRejectionsNext := consecutiveRejectionsPrevious;
      rejectionElapsedNext_s := rejectionElapsedPrevious_s + dt;
    end if;

    // PER-SOURCE rejection counters: fault isolation, not control flow.
    // The global counter above cannot see a broken source while another
    // healthy one keeps resetting it -- a completely dead optical-flow
    // sensor was rejected on every one of its own samples for ten
    // seconds while healthy GPS held the global counter at 4, so nothing
    // in the published health record ever indicated a failed sensor.
    // These counters advance only on the source that was actually
    // attempted, so each one reports its own sensor's record and a
    // persistently failing source becomes visible immediately.
    mocapRejectionsNext := if correctionSource == SourceMocap then
        (if correctionAccepted then 0 else mocapRejectionsPrevious + 1)
      else mocapRejectionsPrevious;
    gpsRejectionsNext := if correctionSource == SourceGps then
        (if correctionAccepted then 0 else gpsRejectionsPrevious + 1)
      else gpsRejectionsPrevious;
    opticalFlowRejectionsNext :=
      if correctionSource == SourceOpticalFlow then
        (if correctionAccepted then 0 else opticalFlowRejectionsPrevious + 1)
      else opticalFlowRejectionsPrevious;
  end if;

  positionNext := working.positionWorldEnu_m;
  velocityNext := working.velocityWorldEnu_m_s;
  quaternionNext := working.quaternionWorldBody;
  gyroscopeBiasNext := working.gyroscopeBiasBodyFlu_rad_s;
  accelerometerBiasNext := working.accelerometerBiasBodyFlu_m_s2;
  covarianceNext := working.covariance;
  initializedNext := true;
  // Publication guard, and DELIBERATELY NOT a ladder output.
  //
  // estimate.valid answers exactly one question: are the published
  // numbers numbers? It is false when the filter has not initialized, or
  // when a component is NaN or infinite -- defence in depth behind the
  // affirmative acceptance predicates, and the only guard covering the
  // prediction side, where a non-finite IMU sample reaches predictNominal
  // with no gate of any kind in front of it.
  //
  // The recovery stage is NOT a term here. Routing degradation through
  // this flag looked like handing over to a failsafe and is not: on the
  // deployed stack estimate.valid = false clears RatesValid, zeroes the
  // motors and latches the fault in every mode including ACRO, because
  // the rate loop takes body rates from the estimator rather than from
  // the raw gyro. A single tick of it in the air is a crash. Worse, the
  // condition was reachable from causes as mild as an anchor that stays
  // valid but stops being fresh -- measured: estimate.valid = 0 with a
  // healthy 10 Hz GPS streaming throughout. Degradation is reported on
  // status.recoveryStage for a wrapper to map onto a non-latching mode
  // demotion.
  estimateValid := initializedNext
    and nominalStateFinite(positionNext, velocityNext, quaternionNext);
end step;
