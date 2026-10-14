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
  State working;
  Boolean correctionAttempted;
  Boolean correctionAccepted;
  Boolean aidingLive;
  Boolean inflateCovarianceNow;
  Boolean aidingDivergent;
  Boolean reseedConfigured;
  Boolean reseedNow;
  Integer reseedSource;
  Boolean anchorExclusive;
  Boolean imuUsable;
  Boolean imuPayloadFinite;
  Boolean ladderConfigured;
  Integer anchorPresent;
  Boolean mocapSeedUsable;
  Boolean gpsSeedUsable;
  Boolean magnetometerSeedUsable;
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
  reseeded := false;
  pseudoPositionCorrectionAccepted := false;
  zeroVelocityCorrectionAccepted := false;
  stationaryImuCorrectionAccepted := false;
  alignmentSource := alignmentSourcePrevious;
  alignmentSpecificForce_m_s2 := alignmentSpecificForcePrevious_m_s2;
  pseudoPositionHoldNext_m := pseudoPositionHoldPrevious_m;
  reseedNow := false;
  reseedSource := SourceNone;
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
    (working, pseudoPositionHoldNext_m, alignmentSource,
     magnetometerCorrectionAccepted, magnetometerTimestampConsumedNext_s) :=
      initializeTick(mocap, gps, magnetometer, tuning,
        mocapSeedUsable, gpsSeedUsable, magnetometerSeedUsable, imuUsable,
        alignmentGateConfigured, imuQuiet, alignmentTimedOut,
        alignmentSpecificForce_m_s2, quietFilterGain, alignmentWaitNext_s,
        dt, gravityWorldEnu_m_s2, imuTimestampHeldNext_s,
        imuAngularVelocityHeldNext_rad_s, imuSpecificForceHeldNext_m_s2,
        magnetometerTimestampConsumedPrevious_s);
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
    (
      working, predictionAccepted, stationaryImuCorrectionAccepted,
      mocapCorrectionAccepted, gpsPositionCorrectionAccepted, gpsVelocityCorrectionAccepted,
      magnetometerCorrectionAccepted, barometerCorrectionAccepted, opticalFlowCorrectionAccepted,
      correctionAttempted, correctionAccepted, correctionOutcome,
      correctionSource, normalizedInnovationSquared, reseeded,
      reseedSource, zeroVelocityCorrectionAccepted, pseudoPositionCorrectionAccepted,
      pseudoPositionHoldNext_m, mocapTimestampConsumedNext_s, gpsTimestampConsumedNext_s,
      magnetometerTimestampConsumedNext_s, barometerTimestampConsumedNext_s, opticalFlowTimestampConsumedNext_s
    ) := advanceStateTick(
      previous, imu, mocap,
      gps, magnetometer, barometer,
      opticalFlow, tuning, gravityWorldEnu_m_s2,
      dt, inflateCovarianceNow, imuUsable,
      imuPayloadFinite, vehicleAtRest, reseedNow,
      anchorExclusive, mocapNew, gpsNew,
      magnetometerNew, barometerNew, opticalFlowNew,
      aidingLive, ladderConfigured, imuQuiet,
      anchorPresent, rejectionElapsedPrevious_s, imuTimestampHeldNext_s,
      imuAngularVelocityHeldNext_rad_s, imuSpecificForceHeldNext_m_s2, pseudoPositionHoldPrevious_m,
      mocapTimestampConsumedPrevious_s, gpsTimestampConsumedPrevious_s, magnetometerTimestampConsumedPrevious_s,
      barometerTimestampConsumedPrevious_s, opticalFlowTimestampConsumedPrevious_s
    );

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
      if not reseedNow and opticalFlowNew
          and (not anchorExclusive or anchorPresent == SourceOpticalFlow) then
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
