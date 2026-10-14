within Estimation.StrapdownINS.ESKF;

function correctAbsoluteAidingTick
  "Select and fuse one absolute aiding sample without expanding the dispatcher"
  input State previous;
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Tuning tuning;
  input Real gravityWorldEnu_m_s2[3];
  input Boolean predictionAccepted;
  input Boolean reseedNow;
  input Boolean mocapNew;
  input Boolean gpsNew;
  input Boolean anchorExclusive;
  input Integer anchorPresent;
  input Real imuTimestampHeldNext_s;
  input Real imuAngularVelocityHeldNext_rad_s[3];
  input Real imuSpecificForceHeldNext_m_s2[3];
  input Real mocapTimestampConsumedPrevious_s;
  input Real gpsTimestampConsumedPrevious_s;
  output State working;
  output Boolean mocapCorrectionAccepted;
  output Boolean gpsPositionCorrectionAccepted;
  output Boolean gpsVelocityCorrectionAccepted;
  output Boolean correctionAttempted;
  output Boolean correctionAccepted;
  output Integer correctionOutcome;
  output Integer correctionSource;
  output Real normalizedInnovationSquared;
  output Real mocapTimestampConsumedNext_s;
  output Real gpsTimestampConsumedNext_s;
algorithm
  working := copyState(previous);
  mocapCorrectionAccepted := false;
  gpsPositionCorrectionAccepted := false;
  gpsVelocityCorrectionAccepted := false;
  correctionAttempted := false;
  correctionAccepted := false;
  correctionOutcome := CorrectionNotAttempted;
  correctionSource := SourceNone;
  normalizedInnovationSquared := 0.0;
  mocapTimestampConsumedNext_s := mocapTimestampConsumedPrevious_s;
  gpsTimestampConsumedNext_s := gpsTimestampConsumedPrevious_s;
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
end correctAbsoluteAidingTick;
