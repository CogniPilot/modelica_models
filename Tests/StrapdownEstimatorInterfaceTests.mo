within Tests;

model StrapdownEstimatorInterfaceTests
  "Both algorithms instantiate and execute through the common replaceable boundary"
  model Harness
    parameter Boolean gpsEnabled = false;
    parameter Real gpsPositionVariance_m2 = 1.0;
    parameter Real gpsInvalidTimestampUntil_s = 0.0;
    replaceable block EstimatorModel = Estimation.StrapdownINS.ESKF.Estimator
      constrainedby Estimation.StrapdownINS.PartialEstimator;
    EstimatorModel estimator(samplePeriod=0.5);
  equation
    estimator.reset = false;
    estimator.imu.valid = true;
    estimator.imu.fresh = true;
    estimator.imu.timestamp_s = time;
    estimator.imu.angularVelocityBodyFlu_rad_s = zeros(3);
    estimator.imu.specificForceBodyFlu_m_s2 = {0.0, 0.0, 9.81};
    estimator.imu.deltaAngleBodyFlu_rad = zeros(3);
    estimator.imu.deltaVelocityBodyFlu_m_s =
      {0.0, 0.0, 9.81 * estimator.samplePeriod};
    estimator.imu.deltaPositionBodyFlu_m =
      {0.0, 0.0, 0.5 * 9.81 * estimator.samplePeriod ^ 2};
    estimator.imu.deltaQuaternionBodyFlu = {1.0, 0.0, 0.0, 0.0};
    estimator.imu.integrationTime_s = estimator.samplePeriod;
    estimator.imu.gyroscopeBiasLinearizationBodyFlu_rad_s = zeros(3);
    estimator.imu.accelerometerBiasLinearizationBodyFlu_m_s2 = zeros(3);
    estimator.imu.deltaRotationGyroscopeBiasJacobian_s =
      -identity(3) * estimator.samplePeriod;
    estimator.imu.deltaVelocityGyroscopeBiasJacobian_m = zeros(3, 3);
    estimator.imu.deltaVelocityAccelerometerBiasJacobian_s =
      -identity(3) * estimator.samplePeriod;
    estimator.imu.deltaPositionGyroscopeBiasJacobian_m_s = zeros(3, 3);
    estimator.imu.deltaPositionAccelerometerBiasJacobian_s2 =
      -0.5 * identity(3) * estimator.samplePeriod ^ 2;
    estimator.mocap.valid = false;
    estimator.mocap.fresh = false;
    estimator.mocap.timestamp_s = time;
    estimator.mocap.positionWorldEnu_m = zeros(3);
    estimator.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
    estimator.mocap.positionCovarianceWorld_m2 = identity(3);
    estimator.mocap.attitudeCovarianceBody_rad2 = identity(3);
    estimator.gps.valid = gpsEnabled;
    estimator.gps.fresh = false;
    estimator.gps.positionValid = gpsEnabled;
    estimator.gps.velocityValid = gpsEnabled;
    // Match the affirmative finite guard with an out-of-range timestamp
    // rather than a constant-folded NaN expression.
    estimator.gps.timestamp_s = if gpsEnabled then
      (if time < gpsInvalidTimestampUntil_s then 1.0e31 else 0.0) else time;
    estimator.gps.geodetic_deg_m = zeros(3);
    estimator.gps.positionWorldEnu_m = zeros(3);
    estimator.gps.velocityWorldEnu_m_s = zeros(3);
    estimator.gps.positionCovarianceWorld_m2 =
      gpsPositionVariance_m2 * identity(3);
    estimator.gps.velocityCovarianceWorld_m2_s2 = identity(3);
    estimator.magnetometer.valid = false;
    estimator.magnetometer.fresh = false;
    estimator.magnetometer.timestamp_s = time;
    estimator.magnetometer.magneticFieldBodyFlu_T =
      estimator.localMagneticFieldWorldEnu_T;
    estimator.magnetometer.covarianceBody_T2 = identity(3) * 1.0e-12;
    estimator.barometer.valid = false;
    estimator.barometer.fresh = false;
    estimator.barometer.timestamp_s = time;
    estimator.barometer.altitudeWorldEnu_m = 0.0;
    estimator.barometer.variance_m2 = 1.0;
    estimator.opticalFlow.valid = false;
    estimator.opticalFlow.fresh = false;
    estimator.opticalFlow.timestamp_s = time;
    estimator.opticalFlow.integratedLineOfSight_rad = zeros(2);
    estimator.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
    estimator.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
    estimator.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
    estimator.opticalFlow.integrationTime_s = 0.5;
    estimator.opticalFlow.groundDistance_m = 1.0;
    estimator.opticalFlow.groundDistanceVariance_m2 = 0.01;
    estimator.opticalFlow.quality = 1.0;
  end Harness;

  Harness eskf;
  Harness ukf(
    redeclare block EstimatorModel = Estimation.StrapdownINS.UKF.Estimator);
  Harness gpsSeed(gpsEnabled=true, gpsPositionVariance_m2=0.25,
    estimator(samplePeriod=0.01,
      initialVariances=Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(0.04, 3), velocity_m2_s2=fill(0.01, 3),
        attitude_rad2=fill(0.01, 3), gyroscopeBias_rad2_s2=fill(0.01, 3),
        accelerometerBias_m2_s4=fill(0.01, 3))));
  Harness invalidSeedTimestamp(gpsEnabled=true,
    gpsInvalidTimestampUntil_s=0.01, estimator(samplePeriod=0.01));
  discrete Boolean validPacketAccepted(start=false, fixed=true);
equation
  assert(time <= 0.015 or validPacketAccepted,
    "An unusable seed timestamp blocked the subsequent valid GPS packet");
  assert(not gpsSeed.estimator.status.gpsPositionCorrectionAccepted,
    "The position seed's held GPS packet was fused again after initialization");
  assert(time <= 0.0 or gpsSeed.estimator.errorCovariance[1, 1] >= 0.25 - 1e-10,
    "GPS-seeded position covariance understates the supplied measurement noise");
algorithm
  when sample(0.0, 0.01) then
    validPacketAccepted := pre(validPacketAccepted)
      or invalidSeedTimestamp.estimator.status.gpsPositionCorrectionAccepted;
  end when;
  annotation(experiment(StartTime=0.0, StopTime=0.02,
    Tolerance=1.0e-8, Interval=0.005));
end StrapdownEstimatorInterfaceTests;
