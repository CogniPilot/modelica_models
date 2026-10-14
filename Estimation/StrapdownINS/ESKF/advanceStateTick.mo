within Estimation.StrapdownINS.ESKF;

function advanceStateTick
  "Advance the state through prediction, recovery and aiding as one bounded stage"
  input State previous;
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.MagnetometerSample magnetometer;
  input Avionics.BarometerSample barometer;
  input Avionics.OpticalFlowSample opticalFlow;
  input Tuning tuning;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt;
  input Boolean inflateCovarianceNow;
  input Boolean imuUsable;
  input Boolean imuPayloadFinite;
  input Boolean vehicleAtRest;
  input Boolean reseedNow;
  input Boolean anchorExclusive;
  input Boolean mocapNew;
  input Boolean gpsNew;
  input Boolean magnetometerNew;
  input Boolean barometerNew;
  input Boolean opticalFlowNew;
  input Boolean aidingLive;
  input Boolean ladderConfigured;
  input Boolean imuQuiet;
  input Integer anchorPresent;
  input Real rejectionElapsedPrevious_s;
  input Real imuTimestampHeldNext_s;
  input Real imuAngularVelocityHeldNext_rad_s[3];
  input Real imuSpecificForceHeldNext_m_s2[3];
  input Real pseudoPositionHoldPrevious_m[3];
  input Real mocapTimestampConsumedPrevious_s;
  input Real gpsTimestampConsumedPrevious_s;
  input Real magnetometerTimestampConsumedPrevious_s;
  input Real barometerTimestampConsumedPrevious_s;
  input Real opticalFlowTimestampConsumedPrevious_s;
  output State working;
  output Boolean predictionAccepted;
  output Boolean stationaryImuCorrectionAccepted;
  output Boolean mocapCorrectionAccepted;
  output Boolean gpsPositionCorrectionAccepted;
  output Boolean gpsVelocityCorrectionAccepted;
  output Boolean magnetometerCorrectionAccepted;
  output Boolean barometerCorrectionAccepted;
  output Boolean opticalFlowCorrectionAccepted;
  output Boolean correctionAttempted;
  output Boolean correctionAccepted;
  output Integer correctionOutcome;
  output Integer correctionSource;
  output Real normalizedInnovationSquared;
  output Boolean reseeded;
  output Integer reseedSource;
  output Boolean zeroVelocityCorrectionAccepted;
  output Boolean pseudoPositionCorrectionAccepted;
  output Real pseudoPositionHoldNext_m[3];
  output Real mocapTimestampConsumedNext_s;
  output Real gpsTimestampConsumedNext_s;
  output Real magnetometerTimestampConsumedNext_s;
  output Real barometerTimestampConsumedNext_s;
  output Real opticalFlowTimestampConsumedNext_s;
protected
  Boolean stationaryVelocityRequested;
  Boolean fallbackRequested;
  Boolean fallbackAttempted;
  Boolean fallbackZeroAccepted;
  Integer fallbackSource;
  Boolean absoluteAidingAttempted;
  Boolean opticalFlowAttempted;
  Integer aidingOutcome;
  Real aidingNis;
algorithm
  predictionAccepted := false;
  stationaryImuCorrectionAccepted := false;
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
  reseedSource := SourceNone;
  zeroVelocityCorrectionAccepted := false;
  pseudoPositionCorrectionAccepted := false;
  pseudoPositionHoldNext_m := pseudoPositionHoldPrevious_m;
  mocapTimestampConsumedNext_s := mocapTimestampConsumedPrevious_s;
  gpsTimestampConsumedNext_s := gpsTimestampConsumedPrevious_s;
  magnetometerTimestampConsumedNext_s := magnetometerTimestampConsumedPrevious_s;
  barometerTimestampConsumedNext_s := barometerTimestampConsumedPrevious_s;
  opticalFlowTimestampConsumedNext_s := opticalFlowTimestampConsumedPrevious_s;
  aidingOutcome := CorrectionNotAttempted;
  aidingNis := 0.0;
  absoluteAidingAttempted := false;
  opticalFlowAttempted := false;
  (working, predictionAccepted, stationaryImuCorrectionAccepted) :=
    predictTick(previous, imu, tuning, gravityWorldEnu_m_s2, dt,
      inflateCovarianceNow, imuUsable, imuPayloadFinite, vehicleAtRest);

  (working, reseeded, reseedSource) := TickUpdates.ReseedStateTick(
    working, reseedNow, anchorPresent, mocap, gps, tuning.initialVariances);

  absoluteAidingAttempted := not reseedNow and
    ((mocapNew and (not anchorExclusive or anchorPresent == SourceMocap))
      or (gpsNew and (gps.positionValid or gps.velocityValid)
        and (not anchorExclusive or anchorPresent == SourceGps)));
  opticalFlowAttempted := not reseedNow and opticalFlowNew
    and (not anchorExclusive or anchorPresent == SourceOpticalFlow);
  (working, mocapCorrectionAccepted, gpsPositionCorrectionAccepted,
   gpsVelocityCorrectionAccepted, correctionAttempted, correctionAccepted,
   correctionOutcome, correctionSource, normalizedInnovationSquared,
   mocapTimestampConsumedNext_s, gpsTimestampConsumedNext_s) :=
    correctAbsoluteAidingTick(working, imu, mocap, gps, tuning,
      gravityWorldEnu_m_s2, predictionAccepted, reseedNow,
      mocapNew, gpsNew, anchorExclusive, anchorPresent,
      imuTimestampHeldNext_s, imuAngularVelocityHeldNext_rad_s,
      imuSpecificForceHeldNext_m_s2, mocapTimestampConsumedPrevious_s,
      gpsTimestampConsumedPrevious_s);

  (working, opticalFlowCorrectionAccepted, aidingOutcome, aidingNis) :=
    TickUpdates.OpticalFlowTick(working, tuning, gravityWorldEnu_m_s2,
      imuTimestampHeldNext_s, imuAngularVelocityHeldNext_rad_s,
      imuSpecificForceHeldNext_m_s2, opticalFlowAttempted, opticalFlow);
  if opticalFlowAttempted then
    if not absoluteAidingAttempted then
      correctionAccepted := opticalFlowCorrectionAccepted;
      correctionSource := SourceOpticalFlow;
      correctionOutcome := aidingOutcome;
      normalizedInnovationSquared := aidingNis;
    end if;
    opticalFlowTimestampConsumedNext_s := opticalFlow.timestamp_s;
  end if;
  (working, barometerCorrectionAccepted, aidingOutcome, aidingNis) :=
    TickUpdates.BarometerTick(working, tuning, gravityWorldEnu_m_s2,
      imuTimestampHeldNext_s, imuAngularVelocityHeldNext_rad_s,
      imuSpecificForceHeldNext_m_s2, not reseedNow and barometerNew, barometer);
  if not reseedNow and barometerNew then
    if not absoluteAidingAttempted and not opticalFlowAttempted then
      correctionAccepted := barometerCorrectionAccepted;
      correctionSource := SourceBarometer;
      correctionOutcome := aidingOutcome;
      normalizedInnovationSquared := aidingNis;
    end if;
    barometerTimestampConsumedNext_s := barometer.timestamp_s;
  end if;
  (working, magnetometerCorrectionAccepted, aidingOutcome, aidingNis) :=
    TickUpdates.MagnetometerTick(working, tuning, gravityWorldEnu_m_s2,
      imuTimestampHeldNext_s, imuAngularVelocityHeldNext_rad_s,
      imuSpecificForceHeldNext_m_s2, not reseedNow and magnetometerNew, magnetometer);
  if not reseedNow and magnetometerNew then
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
  stationaryVelocityRequested := vehicleAtRest and imuUsable and not reseedNow
    and tuning.stationaryVelocityVariance_m2_s2 > 0.0
    and tuning.stationaryVelocityVariance_m2_s2 < FiniteMagnitudeLimit;
  (working, zeroVelocityCorrectionAccepted, aidingOutcome, aidingNis) :=
    TickUpdates.StationaryVelocityTick(working, tuning, stationaryVelocityRequested);
  if stationaryVelocityRequested and not correctionAttempted then
    correctionAccepted := zeroVelocityCorrectionAccepted;
    correctionSource := SourceZeroVelocity;
    correctionOutcome := aidingOutcome;
    normalizedInnovationSquared := aidingNis;
  end if;
  fallbackRequested := not correctionAttempted and not zeroVelocityCorrectionAccepted
    and (not aidingLive or (ladderConfigured
      and rejectionElapsedPrevious_s >= tuning.aidingDivergentWindow_s));
  (working, fallbackAttempted, fallbackZeroAccepted, pseudoPositionCorrectionAccepted,
   aidingOutcome, fallbackSource, aidingNis) := TickUpdates.FallbackHoldTick(
    working, tuning, fallbackRequested, imuQuiet, pseudoPositionHoldPrevious_m);
  zeroVelocityCorrectionAccepted := zeroVelocityCorrectionAccepted or fallbackZeroAccepted;
  if fallbackAttempted then
    correctionAccepted := fallbackZeroAccepted or pseudoPositionCorrectionAccepted;
    correctionSource := fallbackSource;
    correctionOutcome := aidingOutcome;
    normalizedInnovationSquared := aidingNis;
  end if;
  pseudoPositionHoldNext_m := if aidingLive
    then working.positionWorldEnu_m else pseudoPositionHoldPrevious_m;
end advanceStateTick;
