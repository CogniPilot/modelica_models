within Tests;

model StrapdownEstimatorInterfaceTests
  "Both algorithms instantiate and execute through the common replaceable boundary"
  model Harness
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
    estimator.mocap.valid = false;
    estimator.mocap.fresh = false;
    estimator.mocap.timestamp_s = time;
    estimator.mocap.positionWorldEnu_m = zeros(3);
    estimator.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
    estimator.mocap.positionCovarianceWorld_m2 = identity(3);
    estimator.mocap.attitudeCovarianceBody_rad2 = identity(3);
    estimator.gps.valid = false;
    estimator.gps.fresh = false;
    estimator.gps.positionValid = false;
    estimator.gps.velocityValid = false;
    estimator.gps.timestamp_s = time;
    estimator.gps.geodetic_deg_m = zeros(3);
    estimator.gps.positionWorldEnu_m = zeros(3);
    estimator.gps.velocityWorldEnu_m_s = zeros(3);
    estimator.gps.positionCovarianceWorld_m2 = identity(3);
    estimator.gps.velocityCovarianceWorld_m2_s2 = identity(3);
    estimator.opticalFlow.valid = false;
    estimator.opticalFlow.fresh = false;
    estimator.opticalFlow.timestamp_s = time;
    estimator.opticalFlow.velocityBodyFlu_m_s = zeros(2);
    estimator.opticalFlow.velocityCovarianceBody_m2_s2 = identity(2);
    estimator.opticalFlow.integratedLineOfSight_rad = zeros(2);
    estimator.opticalFlow.integrationTime_s = 0.5;
    estimator.opticalFlow.groundDistance_m = 1.0;
    estimator.opticalFlow.groundDistanceVariance_m2 = 0.01;
    estimator.opticalFlow.quality = 1.0;
  end Harness;

  Harness eskf;
  Harness ukf(
    redeclare block EstimatorModel = Estimation.StrapdownINS.UKF.Estimator);
  annotation(experiment(StartTime=0.0, StopTime=0.02,
    Tolerance=1.0e-8, Interval=0.005));
end StrapdownEstimatorInterfaceTests;
