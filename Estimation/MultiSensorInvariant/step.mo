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
protected
  Estimation.MultiSensorInvariant.State previous;
  Estimation.MultiSensorInvariant.State working;
  Avionics.ImuSample imu;
  Avionics.MocapSample mocap;
  Avionics.GpsSample gps;
  Avionics.OpticalFlowSample opticalFlow;
  Estimation.MultiSensorInvariant.InitialVariances initialVariances;
  Estimation.MultiSensorInvariant.ProcessNoise processNoise;
  Real initializationPosition[3];
  Real initializationQuaternion[4];
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

  if not initializedPrevious or reset then
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

    if mocap.valid and mocap.fresh then
      (working, mocapCorrectionAccepted) := correctMocap(working, mocap);
    elseif gps.valid and gps.fresh and gps.positionValid
        and gps.velocityValid then
      (working, gpsPositionCorrectionAccepted) := correctGps(working, gps);
      gpsVelocityCorrectionAccepted := gpsPositionCorrectionAccepted;
    elseif gps.valid and gps.fresh and gps.positionValid then
      (working, gpsPositionCorrectionAccepted) :=
        correctGpsPosition(working, gps);
    elseif gps.valid and gps.fresh and gps.velocityValid then
      (working, gpsVelocityCorrectionAccepted) :=
        correctGpsVelocity(working, gps);
    elseif opticalFlow.valid and opticalFlow.fresh then
      (working, opticalFlowCorrectionAccepted) :=
        correctOpticalFlow(working, opticalFlow);
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
