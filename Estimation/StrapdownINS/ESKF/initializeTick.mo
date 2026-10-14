within Estimation.StrapdownINS.ESKF;

function initializeTick
  "Initialize and condition a navigation state without expanding the step dispatcher"
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.MagnetometerSample magnetometer;
  input Tuning tuning;
  input Boolean mocapSeedUsable;
  input Boolean gpsSeedUsable;
  input Boolean magnetometerSeedUsable;
  input Boolean imuUsable;
  input Boolean alignmentGateConfigured;
  input Boolean imuQuiet;
  input Boolean alignmentTimedOut;
  input Real alignmentSpecificForce_m_s2[3];
  input Real quietFilterGain;
  input Real alignmentWaitNext_s;
  input Real dt;
  input Real gravityWorldEnu_m_s2[3];
  input Real imuTimestampHeldNext_s;
  input Real imuAngularVelocityHeldNext_rad_s[3];
  input Real imuSpecificForceHeldNext_m_s2[3];
  input Real magnetometerTimestampConsumedPrevious_s;
  output State working;
  output Real pseudoPositionHoldNext_m[3];
  output Integer alignmentSource;
  output Boolean magnetometerCorrectionAccepted;
  output Real magnetometerTimestampConsumedNext_s;
protected
  Real initializationPosition[3];
  Real initializationQuaternion[4];
  Boolean alignmentAccepted;
  Real alignmentNoiseWeight;
  Real alignmentSpecificForceCovariance[3, 3];
  Integer aidingOutcome;
  Real aidingNis;
algorithm
  magnetometerTimestampConsumedNext_s := magnetometerTimestampConsumedPrevious_s;
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
end initializeTick;
