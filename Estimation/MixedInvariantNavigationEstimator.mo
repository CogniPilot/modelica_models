within Estimation;

block MixedInvariantNavigationEstimator
  "Sampled multisensor IEKF using mixed SE_2(3) prediction"
  extends Avionics.PartialNavigationEstimator;

  parameter Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  parameter Real initialPositionWorldEnu_m[3] = zeros(3);
  parameter Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  parameter Real initialQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  parameter Real initialGyroscopeBiasBodyFlu_rad_s[3] = zeros(3);
  parameter Real initialAccelerometerBiasBodyFlu_m_s2[3] = zeros(3);
  parameter Estimation.MultiSensorInvariant.InitialVariances initialVariances =
    Estimation.MultiSensorInvariant.InitialVariances(
      position_m2=fill(1.0, 3),
      velocity_m2_s2=fill(1.0, 3),
      attitude_rad2=fill(0.25, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-4, 3),
      accelerometerBias_m2_s4=fill(1.0e-2, 3));
  parameter Estimation.MultiSensorInvariant.ProcessNoise processNoise =
    Estimation.MultiSensorInvariant.ProcessNoise(
      gyroscope_rad2_s=identity(3) * 1.0e-5,
      accelerometer_m2_s3=identity(3) * 1.0e-3,
      gyroscopeBias_rad2_s3=identity(3) * 1.0e-8,
      accelerometerBias_m2_s5=identity(3) * 1.0e-6);

protected
  discrete Real statePosition[3](each start = 0.0, each fixed = true);
  discrete Real stateVelocity[3](each start = 0.0, each fixed = true);
  discrete Real stateQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real stateGyroscopeBias[3](each start = 0.0, each fixed = true);
  discrete Real stateAccelerometerBias[3](each start = 0.0, each fixed = true);
  discrete Real stateCovariance[15, 15](each start = 0.0, each fixed = true);
  discrete Boolean initialized(start = false, fixed = true);
  discrete Boolean predictionAccepted(start = false, fixed = true);
  discrete Boolean mocapCorrectionAccepted(start = false, fixed = true);
  discrete Boolean gpsPositionCorrectionAccepted(start = false, fixed = true);
  discrete Boolean gpsVelocityCorrectionAccepted(start = false, fixed = true);
  discrete Boolean opticalFlowCorrectionAccepted(start = false, fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    (statePosition,
     stateVelocity,
     stateQuaternion,
     stateGyroscopeBias,
     stateAccelerometerBias,
     stateCovariance,
     initialized,
     predictionAccepted,
     mocapCorrectionAccepted,
     gpsPositionCorrectionAccepted,
     gpsVelocityCorrectionAccepted,
     opticalFlowCorrectionAccepted) :=
      Estimation.MultiSensorInvariant.step(
        pre(initialized),
        pre(statePosition),
        pre(stateVelocity),
        pre(stateQuaternion),
        pre(stateGyroscopeBias),
        pre(stateAccelerometerBias),
        pre(stateCovariance),
        reset,
        imu.valid,
        imu.fresh,
        imu.timestamp_s,
        imu.angularVelocityBodyFlu_rad_s,
        imu.specificForceBodyFlu_m_s2,
        mocap.valid,
        mocap.fresh,
        mocap.timestamp_s,
        mocap.positionWorldEnu_m,
        mocap.quaternionWorldBody,
        mocap.positionCovarianceWorld_m2,
        mocap.attitudeCovarianceBody_rad2,
        gps.valid,
        gps.fresh,
        gps.positionValid,
        gps.velocityValid,
        gps.timestamp_s,
        gps.geodetic_deg_m,
        gps.positionWorldEnu_m,
        gps.velocityWorldEnu_m_s,
        gps.positionCovarianceWorld_m2,
        gps.velocityCovarianceWorld_m2_s2,
        opticalFlow.valid,
        opticalFlow.fresh,
        opticalFlow.timestamp_s,
        opticalFlow.velocityBodyFlu_m_s,
        opticalFlow.velocityCovarianceBody_m2_s2,
        opticalFlow.integratedLineOfSight_rad,
        opticalFlow.integrationTime_s,
        opticalFlow.groundDistance_m,
        opticalFlow.quality,
        gravityWorldEnu_m_s2,
        samplePeriod,
        initialVariances.position_m2,
        initialVariances.velocity_m2_s2,
        initialVariances.attitude_rad2,
        initialVariances.gyroscopeBias_rad2_s2,
        initialVariances.accelerometerBias_m2_s4,
        processNoise.gyroscope_rad2_s,
        processNoise.accelerometer_m2_s3,
        processNoise.gyroscopeBias_rad2_s3,
        processNoise.accelerometerBias_m2_s5,
        initialPositionWorldEnu_m,
        initialVelocityWorldEnu_m_s,
        initialQuaternionWorldBody,
        initialGyroscopeBiasBodyFlu_rad_s,
        initialAccelerometerBiasBodyFlu_m_s2);
    (estimate.valid,
     estimate.timestamp_s,
     estimate.positionWorldEnu_m,
     estimate.velocityWorldEnu_m_s,
     estimate.accelerationWorldEnu_m_s2,
     estimate.quaternionWorldBody,
     estimate.rotationWorldBody,
     estimate.eulerRpy_rad,
     estimate.angularVelocityBodyFlu_rad_s,
     estimate.angularVelocityWorldEnu_rad_s) :=
      Estimation.MultiSensorInvariant.navigationEstimateArrays(
      statePosition,
      stateVelocity,
      stateQuaternion,
      stateGyroscopeBias,
      stateAccelerometerBias,
      stateCovariance,
      imu.valid,
      imu.fresh,
      imu.timestamp_s,
      imu.angularVelocityBodyFlu_rad_s,
      imu.specificForceBodyFlu_m_s2,
      gravityWorldEnu_m_s2,
      initialized);
    status.initialized := initialized;
    status.predictionAccepted := predictionAccepted;
    status.mocapCorrectionAccepted := mocapCorrectionAccepted;
    status.gpsPositionCorrectionAccepted := gpsPositionCorrectionAccepted;
    status.gpsVelocityCorrectionAccepted := gpsVelocityCorrectionAccepted;
    status.opticalFlowCorrectionAccepted := opticalFlowCorrectionAccepted;
  end when;

  annotation(Documentation(info = "<html>
    <p>This block is a concrete implementation of the stable estimator
    contract and can be exported directly as an eFMU. Prediction uses one
    vectorized 15-state covariance, including every cross-axis and bias
    correlation. One fresh aiding packet is accepted per estimator tick with
    deterministic priority mocap, GPS, then optical flow; GPS position and
    velocity are fused jointly when both are valid.</p>
  </html>"));
end MixedInvariantNavigationEstimator;
