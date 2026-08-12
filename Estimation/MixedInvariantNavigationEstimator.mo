within Estimation;

block MixedInvariantNavigationEstimator
  "Sampled geometric error-state EKF using mixed SE_2(3) prediction"
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
  parameter Estimation.MultiSensorInvariant.VarianceLimits varianceLimits =
    Estimation.MultiSensorInvariant.VarianceLimits(
      position_m2=fill(1.0e4, 3),
      velocity_m2_s2=fill(4.0e2, 3),
      attitude_rad2=fill(10.0, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-2, 3),
      accelerometerBias_m2_s4=fill(1.0, 3))
    "Diagonal covariance growth bounds from the state units and mission
     envelope: position sigma 100 m dwarfs the sub-100 m RDD2 flight
     volume; velocity sigma 20 m/s is beyond the airframe speed
     envelope; attitude sigma sqrt(10) ~ pi rad means orientation fully
     unknown; gyro-bias sigma 0.1 rad/s and accelerometer-bias sigma
     1 m/s2 (~0.1 g) bound consumer-MEMS turn-on bias. At these values
     the state is already maximally uninformative, so capping there
     loses no information while keeping the covariance inside the
     dynamic range a binary32 Cholesky can factor.";
  parameter Real innovationGate(unit = "1") = 6.0
    "Chi-square innovation gate per degree of freedom; non-positive
     disables. NIS is chi-square with m degrees of freedom for a
     consistent filter; 6.0 per dof keeps the false-rejection
     probability at or below ~0.25% for the m = 2, 3, 6 corrections
     used here (chi-square tails: 12 @ m=2 -> 0.25%, 18 @ m=3 ->
     0.044%, 36 @ m=6 -> 2.8e-6).";
  parameter Integer rejectedCorrectionLimit(min = 1) = 50
    "Consecutive rejected aiding corrections before automatic
     re-initialization. Aiding streams at 20-100 Hz, so 50 rejections
     spans roughly 0.5-2.5 s: long enough that a short outlier burst
     cannot force a spurious re-init, short enough that dead-reckoning
     drift stays small relative to the mission envelope before the
     aiding-seeded re-initialization recovers.";

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
  discrete Integer consecutiveRejectedCorrections(start = 0, fixed = true);
  discrete Boolean covarianceReinitialized(start = false, fixed = true);
  discrete Boolean innovationGateRejected(start = false, fixed = true);

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
     opticalFlowCorrectionAccepted,
     consecutiveRejectedCorrections,
     covarianceReinitialized,
     innovationGateRejected) :=
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
        initialAccelerometerBiasBodyFlu_m_s2,
        varianceLimits.position_m2,
        varianceLimits.velocity_m2_s2,
        varianceLimits.attitude_rad2,
        varianceLimits.gyroscopeBias_rad2_s2,
        varianceLimits.accelerometerBias_m2_s4,
        innovationGate,
        rejectedCorrectionLimit,
        pre(consecutiveRejectedCorrections));
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
    status.consecutiveRejectedCorrections := consecutiveRejectedCorrections;
    status.covarianceReinitialized := covarianceReinitialized;
    status.innovationGateRejected := innovationGateRejected;
  end when;

  annotation(Documentation(info = "<html>
    <p>This block is a concrete implementation of the stable estimator
    contract and can be exported directly as an eFMU. Prediction uses one
    vectorized 15-state covariance, including every cross-axis and bias
    correlation. One fresh aiding packet is accepted per estimator tick with
    deterministic priority mocap, GPS, then optical flow; GPS position and
    velocity are fused jointly when both are valid.</p>
    <p>The SE_2(3) pose/velocity/position part uses geometric propagation,
    injection, and covariance reset. The additive gyroscope and accelerometer
    bias states make the complete augmented system neither strictly invariant
    nor exactly log-linear, so this implementation is described as a geometric
    error-state EKF rather than an IEKF performance claim.</p>
  </html>"));
end MixedInvariantNavigationEstimator;
