within Estimation.MultiSensorInvariant;

function step
  "One sampled prediction and at most one aiding correction"
  input Boolean initializedPrevious;
  input Real positionPrevious[3];
  input Real velocityPrevious[3];
  input Real quaternionPrevious[4];
  input Real gyroscopeBiasPrevious[3];
  input Real accelerometerBiasPrevious[3];
  input Estimation.MultiSensorInvariant.Covariance covariancePrevious;
  input Boolean reset;
  input Boolean imuValid;
  input Boolean imuFresh;
  input Real imuTimestamp_s;
  input Real imuAngularVelocity[3];
  input Real imuSpecificForce[3];
  input Boolean mocapValid;
  input Boolean mocapFresh;
  input Real mocapTimestamp_s;
  input Real mocapPosition[3];
  input Real mocapQuaternion[4];
  input Real mocapPositionCovariance[3, 3];
  input Real mocapAttitudeCovariance[3, 3];
  input Boolean gpsValid;
  input Boolean gpsFresh;
  input Boolean gpsPositionValid;
  input Boolean gpsVelocityValid;
  input Real gpsTimestamp_s;
  input Real gpsGeodetic[3];
  input Real gpsPosition[3];
  input Real gpsVelocity[3];
  input Real gpsPositionCovariance[3, 3];
  input Real gpsVelocityCovariance[3, 3];
  input Boolean opticalFlowValid;
  input Boolean opticalFlowFresh;
  input Real opticalFlowTimestamp_s;
  input Real opticalFlowVelocity[2];
  input Real opticalFlowVelocityCovariance[2, 2];
  input Real opticalFlowIntegratedLineOfSight[2];
  input Real opticalFlowIntegrationTime_s;
  input Real opticalFlowGroundDistance_m;
  input Real opticalFlowQuality;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt;
  input Real initialPositionVariance[3];
  input Real initialVelocityVariance[3];
  input Real initialAttitudeVariance[3];
  input Real initialGyroscopeBiasVariance[3];
  input Real initialAccelerometerBiasVariance[3];
  input Real gyroscopeProcessNoise[3, 3];
  input Real accelerometerProcessNoise[3, 3];
  input Real gyroscopeBiasProcessNoise[3, 3];
  input Real accelerometerBiasProcessNoise[3, 3];
  input Real initialPosition[3];
  input Real initialVelocity[3];
  input Real initialQuaternion[4];
  input Real initialGyroscopeBias[3];
  input Real initialAccelerometerBias[3];
  input Real positionVarianceLimit[3];
  input Real velocityVarianceLimit[3];
  input Real attitudeVarianceLimit[3];
  input Real gyroscopeBiasVarianceLimit[3];
  input Real accelerometerBiasVarianceLimit[3];
  input Real innovationGate
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Integer rejectedCorrectionLimit
    "Consecutive rejected corrections that force re-initialization";
  input Integer consecutiveRejectionsPrevious;
  output Real positionNext[3];
  output Real velocityNext[3];
  output Real quaternionNext[4];
  output Real gyroscopeBiasNext[3];
  output Real accelerometerBiasNext[3];
  output Estimation.MultiSensorInvariant.Covariance covarianceNext;
  output Boolean initializedNext;
  output Boolean predictionAccepted;
  output Boolean mocapCorrectionAccepted;
  output Boolean gpsPositionCorrectionAccepted;
  output Boolean gpsVelocityCorrectionAccepted;
  output Boolean opticalFlowCorrectionAccepted;
  output Integer consecutiveRejectionsNext;
  output Boolean covarianceReinitialized
    "True on a tick where persistent rejection forced re-initialization";
  output Boolean innovationGateRejected
    "True when this tick's attempted correction failed the NIS gate";
protected
  Estimation.MultiSensorInvariant.State previous;
  Estimation.MultiSensorInvariant.State working;
  Avionics.ImuSample imu;
  Avionics.MocapSample mocap;
  Avionics.GpsSample gps;
  Avionics.OpticalFlowSample opticalFlow;
  Estimation.MultiSensorInvariant.InitialVariances initialVariances;
  Estimation.MultiSensorInvariant.VarianceLimits varianceLimits;
  Estimation.MultiSensorInvariant.ProcessNoise processNoise;
  Real initializationPosition[3];
  Real initializationQuaternion[4];
  Boolean correctionAttempted;
  Boolean correctionAccepted;
  Boolean gateRejected;
algorithm
  imu := Avionics.ImuSample(
    valid=imuValid,
    fresh=imuFresh,
    timestamp_s=imuTimestamp_s,
    angularVelocityBodyFlu_rad_s=imuAngularVelocity,
    specificForceBodyFlu_m_s2=imuSpecificForce);
  mocap := Avionics.MocapSample(
    valid=mocapValid,
    fresh=mocapFresh,
    timestamp_s=mocapTimestamp_s,
    positionWorldEnu_m=mocapPosition,
    quaternionWorldBody=mocapQuaternion,
    positionCovarianceWorld_m2=mocapPositionCovariance,
    attitudeCovarianceBody_rad2=mocapAttitudeCovariance);
  gps := Avionics.GpsSample(
    valid=gpsValid,
    fresh=gpsFresh,
    positionValid=gpsPositionValid,
    velocityValid=gpsVelocityValid,
    timestamp_s=gpsTimestamp_s,
    geodetic_deg_m=gpsGeodetic,
    positionWorldEnu_m=gpsPosition,
    velocityWorldEnu_m_s=gpsVelocity,
    positionCovarianceWorld_m2=gpsPositionCovariance,
    velocityCovarianceWorld_m2_s2=gpsVelocityCovariance);
  opticalFlow := Avionics.OpticalFlowSample(
    valid=opticalFlowValid,
    fresh=opticalFlowFresh,
    timestamp_s=opticalFlowTimestamp_s,
    velocityBodyFlu_m_s=opticalFlowVelocity,
    velocityCovarianceBody_m2_s2=opticalFlowVelocityCovariance,
    integratedLineOfSight_rad=opticalFlowIntegratedLineOfSight,
    integrationTime_s=opticalFlowIntegrationTime_s,
    groundDistance_m=opticalFlowGroundDistance_m,
    quality=opticalFlowQuality);
  initialVariances := Estimation.MultiSensorInvariant.InitialVariances(
    position_m2=initialPositionVariance,
    velocity_m2_s2=initialVelocityVariance,
    attitude_rad2=initialAttitudeVariance,
    gyroscopeBias_rad2_s2=initialGyroscopeBiasVariance,
    accelerometerBias_m2_s4=initialAccelerometerBiasVariance);
  varianceLimits := Estimation.MultiSensorInvariant.VarianceLimits(
    position_m2=positionVarianceLimit,
    velocity_m2_s2=velocityVarianceLimit,
    attitude_rad2=attitudeVarianceLimit,
    gyroscopeBias_rad2_s2=gyroscopeBiasVarianceLimit,
    accelerometerBias_m2_s4=accelerometerBiasVarianceLimit);
  processNoise := Estimation.MultiSensorInvariant.ProcessNoise(
    gyroscope_rad2_s=gyroscopeProcessNoise,
    accelerometer_m2_s3=accelerometerProcessNoise,
    gyroscopeBias_rad2_s3=gyroscopeBiasProcessNoise,
    accelerometerBias_m2_s5=accelerometerBiasProcessNoise);
  predictionAccepted := false;
  mocapCorrectionAccepted := false;
  gpsPositionCorrectionAccepted := false;
  gpsVelocityCorrectionAccepted := false;
  opticalFlowCorrectionAccepted := false;
  correctionAttempted := false;
  correctionAccepted := false;
  gateRejected := false;
  innovationGateRejected := false;
  consecutiveRejectionsNext := 0;

  // Auto-reinitialization: the counter only advances on ticks where a
  // fresh aiding sample was actually attempted, so the threshold measures
  // sustained rejection while aiding is streaming, never mere aiding
  // dropout. Once it fires, the full declared initialization policy runs
  // (position and attitude seeded from the active aiding source when
  // available, covariance from the declared initial variances).
  // Reinitializing the covariance alone would deadlock: after long
  // dead-reckoning the position residual can reach kilometers, and no
  // mission-envelope covariance makes such a residual pass a chi-square
  // gate, so every later correction would be re-rejected. The declared
  // initialization path is the one already exercised at startup and on
  // commanded reset; re-running it is the smallest honest recovery.
  covarianceReinitialized := initializedPrevious and not reset
    and consecutiveRejectionsPrevious >= rejectedCorrectionLimit;

  if not initializedPrevious or reset or covarianceReinitialized then
    initializationPosition := if mocap.valid then
        mocap.positionWorldEnu_m
      elseif gps.valid and gps.positionValid then
        gps.positionWorldEnu_m
      else
        initialPosition;
    initializationQuaternion := if mocap.valid then
        mocap.quaternionWorldBody
      else
        initialQuaternion;
    working := initialize(
      initializationPosition,
      initializationQuaternion,
      initialVariances,
      initialVelocity,
      initialGyroscopeBias,
      initialAccelerometerBias);
  else
    previous := Estimation.MultiSensorInvariant.State(
      positionWorldEnu_m=positionPrevious,
      velocityWorldEnu_m_s=velocityPrevious,
      quaternionWorldBody=quaternionPrevious,
      gyroscopeBiasBodyFlu_rad_s=gyroscopeBiasPrevious,
      accelerometerBiasBodyFlu_m_s2=accelerometerBiasPrevious,
      covariance=covariancePrevious);
    if imu.valid then
      working := predict(
        previous,
        imu.angularVelocityBodyFlu_rad_s,
        imu.specificForceBodyFlu_m_s2,
        gravityWorldEnu_m_s2,
        dt,
        processNoise);
      predictionAccepted := true;
    else
      working := Estimation.MultiSensorInvariant.State(
        positionWorldEnu_m=previous.positionWorldEnu_m,
        velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
        quaternionWorldBody=previous.quaternionWorldBody,
        gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
        accelerometerBiasBodyFlu_m_s2=
          previous.accelerometerBiasBodyFlu_m_s2,
        covariance=previous.covariance);
    end if;
    working := Estimation.MultiSensorInvariant.State(
      positionWorldEnu_m=working.positionWorldEnu_m,
      velocityWorldEnu_m_s=working.velocityWorldEnu_m_s,
      quaternionWorldBody=working.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=working.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=working.accelerometerBiasBodyFlu_m_s2,
      covariance=limitCovariance(working.covariance, varianceLimits));

    if mocap.valid and mocap.fresh then
      (working, mocapCorrectionAccepted, gateRejected) :=
        correctMocap(working, mocap, innovationGate);
      correctionAttempted := true;
      correctionAccepted := mocapCorrectionAccepted;
    elseif gps.valid and gps.fresh and gps.positionValid
        and gps.velocityValid then
      (working, gpsPositionCorrectionAccepted, gateRejected) :=
        correctGps(working, gps, innovationGate);
      gpsVelocityCorrectionAccepted := gpsPositionCorrectionAccepted;
      correctionAttempted := true;
      correctionAccepted := gpsPositionCorrectionAccepted;
    elseif gps.valid and gps.fresh and gps.positionValid then
      (working, gpsPositionCorrectionAccepted, gateRejected) :=
        correctGpsPosition(working, gps, innovationGate);
      correctionAttempted := true;
      correctionAccepted := gpsPositionCorrectionAccepted;
    elseif gps.valid and gps.fresh and gps.velocityValid then
      (working, gpsVelocityCorrectionAccepted, gateRejected) :=
        correctGpsVelocity(working, gps, innovationGate);
      correctionAttempted := true;
      correctionAccepted := gpsVelocityCorrectionAccepted;
    elseif opticalFlow.valid and opticalFlow.fresh then
      (working, opticalFlowCorrectionAccepted, gateRejected) :=
        correctOpticalFlow(working, opticalFlow, innovationGate);
      correctionAttempted := true;
      correctionAccepted := opticalFlowCorrectionAccepted;
    end if;

    if correctionAttempted then
      innovationGateRejected := gateRejected;
      if correctionAccepted then
        consecutiveRejectionsNext := 0;
      else
        consecutiveRejectionsNext := consecutiveRejectionsPrevious + 1;
      end if;
    else
      consecutiveRejectionsNext := consecutiveRejectionsPrevious;
    end if;
  end if;

  positionNext := working.positionWorldEnu_m;
  velocityNext := working.velocityWorldEnu_m_s;
  quaternionNext := working.quaternionWorldBody;
  gyroscopeBiasNext := working.gyroscopeBiasBodyFlu_rad_s;
  accelerometerBiasNext := working.accelerometerBiasBodyFlu_m_s2;
  covarianceNext := working.covariance;
  initializedNext := true;
end step;
