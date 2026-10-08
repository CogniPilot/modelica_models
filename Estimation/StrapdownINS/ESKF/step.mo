within Estimation.StrapdownINS.ESKF;

function step
  "Predict, select absolute aiding, and fuse independent flow, height and heading"
  input Boolean initializedPrevious;
  input State previous;
  input Boolean reset;
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.MagnetometerSample magnetometer;
  input Avionics.BarometerSample barometer;
  input Avionics.OpticalFlowSample opticalFlow;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt;
  input Tuning tuning;
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
  input Real quietElapsedPrevious_s;
  input Real alignmentWaitPrevious_s;
  input Real pseudoPositionHoldPrevious_m[3];
  input Integer alignmentSourcePrevious;
  input Real alignmentSpecificForcePrevious_m_s2[3];
  input Real quietAngularRatePrevious_rad_s[3];
  input Boolean vehicleAtRest = false;
  output Real positionNext[3];
  output Real velocityNext[3];
  output Real quaternionNext[4];
  output Real gyroscopeBiasNext[3];
  output Real accelerometerBiasNext[3];
  output Covariance covarianceNext;
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
  output Boolean reseeded
    "True on the tick the automatic recovery ladder re-seeded the state
     from a fresh anchor sample after a sustained rejection window";
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
  output Real alignmentWaitNext_s
    "Time with a usable IMU spent waiting for the alignment gate; zero once
     aligned";
  output Real quietElapsedNext_s
    "Unbroken time the IMU has reported a quasi-static vehicle: specific
     force within tolerance of gravity and angular rate below the limit";
  output Real pseudoPositionHoldNext_m[3]
    "Position the synthetic hold-position measurement is taken at: the
     current estimate while an anchor is live, frozen when it drops";
  output Integer alignmentSource
    "Estimation.StrapdownINS.Alignment* code of the alignment in force";
  output Real alignmentSpecificForce_m_s2[3]
    "Low-pass specific force the quasi-static test reads and the initial
     alignment levels on; the raw sample when no filter is configured";
  output Real quietAngularRateNext_rad_s[3]
    "Low-pass angular rate the quasi-static test reads";
  output Boolean pseudoPositionCorrectionAccepted;
  output Boolean zeroVelocityCorrectionAccepted;
  output Boolean stationaryImuCorrectionAccepted;
  output Covariance covarianceRootNext;
  output Real barometerBiasCrossCovarianceNext[TangentLength];
  output Real barometerBiasNext_m;
  output Real barometerBiasVarianceNext_m2;
protected
  State prior;
  State working;
  Real initializationPosition[3];
  Real initializationQuaternion[4];
  Boolean correctionAttempted;
  Boolean correctionAccepted;
  Boolean absoluteAidingAttempted;
  Boolean opticalFlowAttempted;
  Integer aidingOutcome;
  Real aidingNis;
  Boolean aidingLive;
  Boolean inflateCovarianceNow;
  Boolean aidingDivergent;
  Boolean reseedConfigured;
  Boolean reseedNow;
  Integer reseedSource;
  Real reseedPosition[3];
  Real reseedVelocity[3];
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
  Real specificForceMagnitude;
  Real angularRateMagnitude;
  Real gravityMagnitude;
  Real sampleSpecificForce_m_s2[3];
  Real sampleAngularRate_rad_s[3];
  Real quietFilterGain;
  Boolean quietFilterSeeded;
  Boolean imuQuiet;
  Boolean alignmentGateConfigured;
  Boolean alignmentReady;
  Boolean alignmentTimedOut;
  Boolean alignmentPending;
  Boolean pseudoPositionConfigured;
  Boolean zeroVelocityConfigured;
  Real alignmentNoiseWeight;
  Real alignmentSpecificForceCovariance[3, 3];
  Covariance heldDynamics;
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
  absoluteAidingAttempted := false;
  opticalFlowAttempted := false;
  aidingOutcome := CorrectionNotAttempted;
  aidingNis := 0.0;
  correctionOutcome := CorrectionNotAttempted;
  correctionSource := SourceNone;
  normalizedInnovationSquared := 0.0;
  reseeded := false;
  pseudoPositionCorrectionAccepted := false;
  zeroVelocityCorrectionAccepted := false;
  stationaryImuCorrectionAccepted := false;
  alignmentSource := alignmentSourcePrevious;
  alignmentSpecificForce_m_s2 := alignmentSpecificForcePrevious_m_s2;
  pseudoPositionHoldNext_m := pseudoPositionHoldPrevious_m;
  reseedNow := false;
  reseedSource := SourceNone;
  reseedPosition := zeros(3);
  reseedVelocity := zeros(3);
  // Initialization-branch scratch, defined here so every read is dominated
  // by a definition on all control-flow paths through the function.
  alignmentAccepted := false;
  initializationPosition := zeros(3);
  initializationQuaternion := {1.0, 0.0, 0.0, 0.0};
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

  imuPayloadFinite := imu.valid and imuSampleFinite(imu);
  // Cross-clock delivery is level-triggered: the producer holds the most
  // recent packet, and timestamp novelty makes consumption exactly once.
  imuUsable := imuPayloadFinite
    and imu.timestamp_s > imuTimestampHeldPrevious_s + 1.0e-9;
  sampleSpecificForce_m_s2 := if imu.integrationTime_s > 1.0e-6
    then imu.deltaVelocityBodyFlu_m_s / imu.integrationTime_s
    else imu.specificForceBodyFlu_m_s2;
  sampleAngularRate_rad_s := if imu.integrationTime_s > 1.0e-6
    then imu.deltaAngleBodyFlu_rad / imu.integrationTime_s
    else imu.angularVelocityBodyFlu_rad_s;
  quietFilterGain := if tuning.quietFilterTimeConstant_s > 0.0
      and tuning.quietFilterTimeConstant_s < FiniteMagnitudeLimit
    then min(1.0, dt / tuning.quietFilterTimeConstant_s) else 1.0;
  quietFilterSeeded := alignmentSpecificForcePrevious_m_s2
    * alignmentSpecificForcePrevious_m_s2 > 0.0;
  if not imuUsable then
    alignmentSpecificForce_m_s2 := alignmentSpecificForcePrevious_m_s2;
    quietAngularRateNext_rad_s := quietAngularRatePrevious_rad_s;
  elseif not quietFilterSeeded then
    alignmentSpecificForce_m_s2 := sampleSpecificForce_m_s2;
    quietAngularRateNext_rad_s := sampleAngularRate_rad_s;
  else
    alignmentSpecificForce_m_s2 := alignmentSpecificForcePrevious_m_s2
      + quietFilterGain
        * (sampleSpecificForce_m_s2 - alignmentSpecificForcePrevious_m_s2);
    quietAngularRateNext_rad_s := quietAngularRatePrevious_rad_s
      + quietFilterGain
        * (sampleAngularRate_rad_s - quietAngularRatePrevious_rad_s);
  end if;
  specificForceMagnitude := sqrt(alignmentSpecificForce_m_s2
    * alignmentSpecificForce_m_s2);
  angularRateMagnitude := sqrt(quietAngularRateNext_rad_s
    * quietAngularRateNext_rad_s);
  gravityMagnitude := sqrt(gravityWorldEnu_m_s2 * gravityWorldEnu_m_s2);
  imuQuiet := imuUsable
    and abs(specificForceMagnitude - gravityMagnitude)
      <= tuning.quietSpecificForceTolerance_m_s2
    and angularRateMagnitude <= tuning.quietAngularRateLimit_rad_s;
  // The window is a property of the inertial stream alone: a commanded
  // reset re-aligns from it but does not restart it, and a consumer that
  // holds reset asserted until the filter reports initialized must still
  // see the gate open. A tick that fails the test costs the window one
  // tick rather than all of it, so an isolated noise excursion delays the
  // gate instead of restarting it; sustained motion still drains it.
  quietElapsedNext_s := if not imuQuiet
    then max(0.0, quietElapsedPrevious_s - dt)
    else quietElapsedPrevious_s + dt;
  alignmentGateConfigured := tuning.initialAlignmentWindow_s > 0.0
    and tuning.initialAlignmentWindow_s < FiniteMagnitudeLimit;
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
  for axis in 1:3 loop
    mocapSeedUsable := mocapSeedUsable
      and abs(mocap.positionWorldEnu_m[axis]) < FiniteMagnitudeLimit;
  end for;
  for component in 1:4 loop
    mocapSeedUsable := mocapSeedUsable
      and abs(mocap.quaternionWorldBody[component]) < FiniteMagnitudeLimit;
  end for;
  // Normalizability, not just finiteness: a finite but vanishing
  // quaternion survives every magnitude check and then becomes a
  // full-scale garbage rotation when initialize() normalizes it.
  mocapSeedQuaternionNorm := mocap.quaternionWorldBody
    * mocap.quaternionWorldBody;
  mocapSeedUsable := mocapSeedUsable
    and sqrt(mocapSeedQuaternionNorm) >= MinimumSeedQuaternionNorm;
  gpsSeedUsable := gps.valid and gps.positionValid;
  for axis in 1:3 loop
    gpsSeedUsable := gpsSeedUsable
      and abs(gps.positionWorldEnu_m[axis]) < FiniteMagnitudeLimit;
  end for;
  magnetometerSeedUsable := magnetometer.valid;
  for axis in 1:3 loop
    magnetometerSeedUsable := magnetometerSeedUsable
      and abs(magnetometer.magneticFieldBodyFlu_T[axis]) < FiniteMagnitudeLimit
      and abs(tuning.localMagneticFieldWorldEnu_T[axis]) < FiniteMagnitudeLimit
      and magnetometer.covarianceBody_T2[axis, axis] > 0.0;
  end for;

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

  anchorSourceNext := if anchorPresent <> SourceNone then anchorPresent
    else anchorSourcePrevious;
  aidingLive := anchorPresent <> SourceNone;

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
  // STAGE 3 configuration admission. Re-seed is opt-in: a non-positive or
  // non-finite window disables it, and an enabled window that does not
  // exceed the divergent window disables it too. A misconfigured value
  // therefore silences only the re-seed and never disturbs the two
  // always-on stages, the same affirmative-admission discipline the rest of
  // the ladder uses on its own windows.
  reseedConfigured := ladderConfigured
    and tuning.aidingReseedWindow_s > 0.0
    and tuning.aidingReseedWindow_s < FiniteMagnitudeLimit
    and tuning.aidingReseedWindow_s > tuning.aidingDivergentWindow_s;
  // The re-seed fires only past the re-seed window, on the anchor source
  // PRESENT this tick, and only when that source delivered a fresh sample
  // this tick that passes the same finite-data seed contract the
  // initialization branch enforces (mocap position and a normalizable
  // attitude; GPS position with positionValid). A non-finite payload falls
  // through exactly as an absent one, so a NaN can never seed the state.
  reseedNow := reseedConfigured and initializedPrevious and not reset
    and rejectionElapsedPrevious_s >= tuning.aidingReseedWindow_s
    and ((anchorPresent == SourceMocap and mocapNew and mocapSeedUsable)
      or (anchorPresent == SourceGps and gpsNew and gpsSeedUsable));
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


  alignmentWaitNext_s := if initializedPrevious and not reset then 0.0
    elseif imuUsable then alignmentWaitPrevious_s + dt
    else alignmentWaitPrevious_s;
  alignmentTimedOut := tuning.initialAlignmentTimeout_s > 0.0
    and tuning.initialAlignmentTimeout_s < FiniteMagnitudeLimit
    and alignmentWaitNext_s >= tuning.initialAlignmentTimeout_s;
  alignmentReady := mocapSeedUsable or not alignmentGateConfigured
    or (imuUsable
      and (quietElapsedNext_s >= tuning.initialAlignmentWindow_s
        or alignmentTimedOut));
  alignmentPending := (not initializedPrevious or reset)
    and not alignmentReady;
  if alignmentPending then
    working := copyState(previous);
    alignmentSource := AlignmentNone;
  elseif not initializedPrevious or reset then
    initializationPosition := if mocapSeedUsable then
        mocap.positionWorldEnu_m
      elseif gpsSeedUsable then
        gps.positionWorldEnu_m
      else
        tuning.initialState.positionWorldEnu_m;
    if mocapSeedUsable then
      initializationQuaternion := mocap.quaternionWorldBody;
      alignmentAccepted := true;
      alignmentSource := AlignmentMocap;
    elseif imuUsable and magnetometerSeedUsable then
      (initializationQuaternion, alignmentAccepted) :=
        Estimation.StrapdownINS.initialAlignmentQuaternion(
          alignmentSpecificForce_m_s2,
          magnetometer.magneticFieldBodyFlu_T,
          tuning.localMagneticFieldWorldEnu_T,
          tuning.initialState.quaternionWorldBody);
      alignmentSource := if alignmentAccepted
        then AlignmentAccelerometerMagnetometer else AlignmentFallback;
    elseif imuUsable and alignmentGateConfigured then
      // No magnetometer: level on the filtered specific force and leave the
      // heading at zero. Only offered behind the quiet gate, so the vector
      // is one taken at rest.
      (initializationQuaternion, alignmentAccepted) :=
        Estimation.StrapdownINS.initialAlignmentQuaternion(
          alignmentSpecificForce_m_s2,
          zeros(3),
          tuning.localMagneticFieldWorldEnu_T,
          tuning.initialState.quaternionWorldBody,
          false);
      alignmentSource := if alignmentAccepted
        then AlignmentAccelerometer else AlignmentFallback;
    else
      initializationQuaternion := tuning.initialState.quaternionWorldBody;
      alignmentAccepted := false;
      alignmentSource := AlignmentFallback;
    end if;
    pseudoPositionHoldNext_m := initializationPosition;
    magnetometerCorrectionAccepted := magnetometerSeedUsable
      and alignmentAccepted
      and not mocapSeedUsable;
    working := initialize(
      initializationPosition,
      initializationQuaternion,
      tuning.initialVariances,
      tuning.initialState.velocityWorldEnu_m_s,
      tuning.initialState.gyroscopeBiasBodyFlu_rad_s,
      tuning.initialState.accelerometerBiasBodyFlu_m_s2,
      if mocapSeedUsable then mocap.positionCovarianceWorld_m2
      elseif gpsSeedUsable then gps.positionCovarianceWorld_m2
      else zeros(3, 3), tuning.useSquareRootCovariance,
      tuning.barometerBias_m, tuning.barometerBiasVariance_m2,
      tuning.useJointBarometerBias);
    if tuning.useGeometricAlignment and alignmentGateConfigured
        and imuQuiet and not alignmentTimedOut and alignmentAccepted
        and (alignmentSource == AlignmentAccelerometer
          or alignmentSource == AlignmentAccelerometerMagnetometer) then
      alignmentNoiseWeight := (1.0 - quietFilterGain)
        ^ (2.0 * max(alignmentWaitNext_s / dt - 1.0, 0.0));
      alignmentSpecificForceCovariance := tuning.processNoise.accelerometer_m2_s3
        / dt * (alignmentNoiseWeight + (1.0 - alignmentNoiseWeight)
          * quietFilterGain / (2.0 - quietFilterGain));
      working := conditionAlignment(working, alignmentSpecificForce_m_s2,
        alignmentSpecificForceCovariance, gravityWorldEnu_m_s2,
        tuning.innovationGate, tuning.useSemiDirectBias);
      if magnetometerSeedUsable then
        (working, magnetometerCorrectionAccepted, aidingOutcome, aidingNis) :=
          correctMagnetometer(working, magnetometer,
            tuning.localMagneticFieldWorldEnu_T, tuning.innovationGate,
            imuTimestampHeldNext_s - magnetometer.timestamp_s,
            imuAngularVelocityHeldNext_rad_s,
            imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
            tuning.maximumAidingDelay_s, tuning.useEquivariantMagnetometer,
            tuning.useSemiDirectBias);
        magnetometerTimestampConsumedNext_s := magnetometer.timestamp_s;
      end if;
    end if;
    // A seed is already an observation of the position. Consume that packet
    // here so the same held noise is not fused again on the next IMU tick.
    if mocapSeedUsable then
      mocapTimestampConsumedNext_s := if abs(mocap.timestamp_s) < FiniteMagnitudeLimit
        then mocap.timestamp_s else mocapTimestampConsumedPrevious_s;
    elseif gpsSeedUsable then
      gpsTimestampConsumedNext_s := if abs(gps.timestamp_s) < FiniteMagnitudeLimit
        then gps.timestamp_s else gpsTimestampConsumedPrevious_s;
    end if;
  else
    prior := if inflateCovarianceNow then
      inflateStateCovariance(previous, tuning.varianceLimits,
        dt, tuning.covarianceInflateTimeConstant_s) else copyState(previous);
    if imuUsable then
      if vehicleAtRest and tuning.useStationaryImu then
        working := predictStationary(
          prior, imu.integrationTime_s, tuning.processNoise);
        (working, stationaryImuCorrectionAccepted, aidingOutcome, aidingNis) :=
          correctStationaryImu(working, imu, gravityWorldEnu_m_s2,
            tuning.processNoise, tuning.innovationGate, tuning.useSemiDirectBias);
      else
        working := predictPreintegrated(
          prior, imu, gravityWorldEnu_m_s2, tuning.processNoise);
      end if;
      predictionAccepted := true;
    elseif not imuPayloadFinite then
      if prior.useSquareRootCovariance then
        heldDynamics := continuousTransition(zeros(3), zeros(3));
        working := predictCovariance(prior, discreteTransition(heldDynamics, dt),
          heldDynamics, tuning.processNoise, dt);
      else
        working := withDenseCovariance(prior,
          holdCovariance(prior.covariance, dt, tuning.processNoise),
          discreteTransition(continuousTransition(zeros(3), zeros(3)), dt)
            * prior.barometerBiasCrossCovariance);
      end if;
    else
      // A valid held packet means the high-rate preintegrator is still
      // accumulating the next delta-angle/delta-velocity observation. It is
      // neither a new prediction nor an IMU dropout, so do not propagate the
      // nominal state or add process noise a second time.
      working := copyState(prior);
    end if;
    working := limitStateCovariance(working, tuning.varianceLimits);

    if reseedNow then
      reseedSource := anchorPresent;
      if anchorPresent == SourceMocap then
        reseedPosition := mocap.positionWorldEnu_m;
        reseedVelocity := working.velocityWorldEnu_m_s;
      else
        reseedPosition := gps.positionWorldEnu_m;
        reseedVelocity := if gps.velocityValid then
          gps.velocityWorldEnu_m_s else working.velocityWorldEnu_m_s;
      end if;
      // Position and velocity, and only their covariance blocks, are replaced;
      // the attitude and bias states and their variances are carried through
      // untouched. The record assembly lives in reseed() so it does not share
      // a common-subexpression temporary with the prediction-side assemblies.
      working := reseed(working, reseedPosition, reseedVelocity,
        tuning.initialVariances);
      reseeded := true;
    end if;

    absoluteAidingAttempted := not reseedNow and
      ((mocapNew and (not anchorExclusive or anchorPresent == SourceMocap))
        or (gpsNew and (gps.positionValid or gps.velocityValid)
          and (not anchorExclusive or anchorPresent == SourceGps)));
    opticalFlowAttempted := not reseedNow and opticalFlowNew
      and (not anchorExclusive or anchorPresent == SourceOpticalFlow);
    if reseedNow then
      // A re-seed happened this tick: the state is already on the anchor
      // sample with the restored prior, so no correction is attempted.
      correctionAttempted := false;
    elseif mocapNew
        and (not anchorExclusive or anchorPresent == SourceMocap) then
      (working, mocapCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctMocap(working, mocap, tuning.innovationGate,
          imuTimestampHeldNext_s - mocap.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s,
          tuning.useSemiDirectBias);
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
          tuning.maximumAidingDelay_s,
          if predictionAccepted and imu.integrationTime_s > 1.0e-6 then
            cat(1,
              cat(2, tuning.processNoise.gyroscope_rad2_s, zeros(3, 3)),
              cat(2, zeros(3, 3), tuning.processNoise.accelerometer_m2_s3))
              / imu.integrationTime_s
          else zeros(6, 6),
          if predictionAccepted and imu.integrationTime_s > 1.0e-6 then
            imu.integrationTime_s else 0.0,
          tuning.useSemiDirectBias);
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
          tuning.maximumAidingDelay_s,
          tuning.useSemiDirectBias);
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
          tuning.maximumAidingDelay_s,
          tuning.useSemiDirectBias);
      correctionAttempted := true;
      correctionAccepted := gpsVelocityCorrectionAccepted;
      correctionSource := SourceGps;
      gpsTimestampConsumedNext_s := gps.timestamp_s;
    end if;

    if opticalFlowAttempted then
      (working, opticalFlowCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctOpticalFlow(working, opticalFlow, tuning.innovationGate,
          imuTimestampHeldNext_s - opticalFlow.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s,
          tuning.minimumOpticalFlowQuality,
          tuning.minimumOpticalFlowGroundDistance_m,
          tuning.useSemiDirectBias);
      if not absoluteAidingAttempted then
        correctionAccepted := opticalFlowCorrectionAccepted;
        correctionSource := SourceOpticalFlow;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
      opticalFlowTimestampConsumedNext_s := opticalFlow.timestamp_s;
    end if;
    if not reseedNow and barometerNew then
      (working, barometerCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctBarometer(working, barometer,
          tuning.barometerBias_m, tuning.barometerBiasVariance_m2,
          tuning.innovationGate,
          imuTimestampHeldNext_s - barometer.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s,
          tuning.useSemiDirectBias, tuning.useBarometerBiasConsider,
          tuning.barometerBiasProcessNoise_m2_s);
      if not absoluteAidingAttempted and not opticalFlowAttempted then
        correctionAccepted := barometerCorrectionAccepted;
        correctionSource := SourceBarometer;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
      barometerTimestampConsumedNext_s := barometer.timestamp_s;
    end if;
    if not reseedNow and magnetometerNew then
      (working, magnetometerCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctMagnetometer(working, magnetometer,
          tuning.localMagneticFieldWorldEnu_T, tuning.innovationGate,
          imuTimestampHeldNext_s - magnetometer.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s, tuning.useEquivariantMagnetometer,
          tuning.useSemiDirectBias);
      if not absoluteAidingAttempted and not opticalFlowAttempted
          and not barometerNew then
        correctionAccepted := magnetometerCorrectionAccepted;
        correctionSource := SourceMagnetometer;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
      magnetometerTimestampConsumedNext_s := magnetometer.timestamp_s;
    end if;
    correctionAttempted := not reseedNow and (absoluteAidingAttempted
      or opticalFlowAttempted or barometerNew or magnetometerNew);
    pseudoPositionConfigured := tuning.pseudoPositionVariance_m2 > 0.0
      and tuning.pseudoPositionVariance_m2 < FiniteMagnitudeLimit;
    zeroVelocityConfigured := tuning.zeroVelocityVariance_m2_s2 > 0.0
      and tuning.zeroVelocityVariance_m2_s2 < FiniteMagnitudeLimit;
    if vehicleAtRest and imuUsable and not reseedNow
        and tuning.stationaryVelocityVariance_m2_s2 > 0.0
        and tuning.stationaryVelocityVariance_m2_s2 < FiniteMagnitudeLimit then
      (working, zeroVelocityCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctZeroVelocity(working,
          tuning.stationaryVelocityVariance_m2_s2, 0.0,
          tuning.useSemiDirectBias);
      if not correctionAttempted then
        correctionAccepted := zeroVelocityCorrectionAccepted;
        correctionSource := SourceZeroVelocity;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
    end if;
    if not correctionAttempted and not zeroVelocityCorrectionAccepted
        and (not aidingLive or (ladderConfigured
          and rejectionElapsedPrevious_s
            >= tuning.aidingDivergentWindow_s)) then
      if zeroVelocityConfigured and imuQuiet then
        (working, zeroVelocityCorrectionAccepted, correctionOutcome,
         normalizedInnovationSquared) :=
          correctZeroVelocity(working, tuning.zeroVelocityVariance_m2_s2,
            0.0, tuning.useSemiDirectBias);
        correctionAccepted := zeroVelocityCorrectionAccepted;
        correctionSource := SourceZeroVelocity;
      elseif pseudoPositionConfigured then
        (working, pseudoPositionCorrectionAccepted, correctionOutcome,
         normalizedInnovationSquared) :=
          correctPseudoPosition(working, pseudoPositionHoldPrevious_m,
            tuning.pseudoPositionVariance_m2, 0.0,
            tuning.useSemiDirectBias);
        correctionAccepted := pseudoPositionCorrectionAccepted;
        correctionSource := SourcePseudoPosition;
      end if;
    end if;
    pseudoPositionHoldNext_m := if aidingLive
      then working.positionWorldEnu_m else pseudoPositionHoldPrevious_m;

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

    mocapRejectionsNext := if correctionSource == SourceMocap then
        (if correctionAccepted then 0 else mocapRejectionsPrevious + 1)
      else mocapRejectionsPrevious;
    gpsRejectionsNext := if correctionSource == SourceGps then
        (if correctionAccepted then 0 else gpsRejectionsPrevious + 1)
      else gpsRejectionsPrevious;
    opticalFlowRejectionsNext :=
      if opticalFlowAttempted then
        (if opticalFlowCorrectionAccepted then 0
          else opticalFlowRejectionsPrevious + 1)
      else opticalFlowRejectionsPrevious;

    // STAGE 3 telemetry and clock reset. The re-seed is a deliberate
    // replacement of the state, so the tick reports CorrectionReseeded
    // rather than the gate acceptance the same-tick fusion recorded, and it
    // clears the rejection clock and every per-source counter: the
    // divergence they were measuring has been resolved by construction.
    if reseeded then
      correctionOutcome := CorrectionReseeded;
      correctionSource := reseedSource;
      consecutiveRejectionsNext := 0;
      rejectionElapsedNext_s := 0.0;
      mocapRejectionsNext := 0;
      gpsRejectionsNext := 0;
      opticalFlowRejectionsNext := 0;
    end if;
  end if;

  positionNext := working.positionWorldEnu_m;
  velocityNext := working.velocityWorldEnu_m_s;
  quaternionNext := working.quaternionWorldBody;
  gyroscopeBiasNext := working.gyroscopeBiasBodyFlu_rad_s;
  accelerometerBiasNext := working.accelerometerBiasBodyFlu_m_s2;
  covarianceNext := working.covariance;
  covarianceRootNext := working.covarianceRoot;
  barometerBiasCrossCovarianceNext := working.barometerBiasCrossCovariance;
  barometerBiasNext_m := working.barometerBias_m;
  barometerBiasVarianceNext_m2 := working.barometerBiasVariance_m2;
  initializedNext := not alignmentPending;
  estimateValid := initializedNext
    and nominalStateFinite(positionNext, velocityNext, quaternionNext);
end step;
