within Tests;

model MultisensorAidingTests
  "Simultaneous aiding, exactly-once consumption, and independent rejection health"
    Estimation.StrapdownINS.ESKF.Estimator estimator(samplePeriod=0.01,
      barometerBiasCalibrationSamples=5);
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
    estimator.gps.valid = (time < 0.5 or time >= 0.8);
    estimator.gps.fresh = true;
    estimator.gps.positionValid = (time < 0.5 or time >= 0.8);
    estimator.gps.velocityValid = (time < 0.5 or time >= 0.8);
    estimator.gps.timestamp_s = 0.1 * floor((time + 1.0e-8) / 0.1);
    estimator.gps.geodetic_deg_m = zeros(3);
    estimator.gps.positionWorldEnu_m =
      if time >= 0.3 and time < 0.4 then {1000.0, 0.0, 0.0} else zeros(3);
    estimator.gps.velocityWorldEnu_m_s = zeros(3);
    estimator.gps.positionCovarianceWorld_m2 =
      0.04 * identity(3);
    estimator.gps.velocityCovarianceWorld_m2_s2 = identity(3);
    estimator.magnetometer.valid = true;
    estimator.magnetometer.fresh = false;
    estimator.magnetometer.timestamp_s =
      if time >= 0.4 and time < 0.5 then time
      else 0.02 * floor((time + 1.0e-8) / 0.02);
    estimator.magnetometer.magneticFieldBodyFlu_T =
      estimator.localMagneticFieldWorldEnu_T;
    estimator.magnetometer.covarianceBody_T2 = identity(3) * 1.0e-12;
    estimator.barometer.valid = true;
    estimator.barometer.fresh = false;
    estimator.barometer.timestamp_s = estimator.magnetometer.timestamp_s;
    estimator.barometer.altitudeWorldEnu_m = 0.0;
    estimator.barometer.variance_m2 = 1.0;
    estimator.opticalFlow.valid = true;
    estimator.opticalFlow.fresh = false;
    estimator.opticalFlow.timestamp_s = time;
    estimator.opticalFlow.integratedLineOfSight_rad = zeros(2);
    estimator.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2) * 1.0e-8;
    estimator.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
    estimator.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3) * 1.0e-10;
    estimator.opticalFlow.integrationTime_s = 0.01;
    estimator.opticalFlow.groundDistance_m = 1.0;
    estimator.opticalFlow.groundDistanceVariance_m2 = 0.01;
    estimator.opticalFlow.quality = if time >= 0.4 and time < 0.5 then 0.0 else 1.0;
  protected
    discrete Integer lastCount(start=0, fixed=true);
    Integer observedTick;
  equation
    observedTick = integer(floor((time + 1.0e-8) / 0.01));
  algorithm
    when sample(0.005, 0.01) then
      if observedTick > 10 then
        assert(estimator.estimate.valid, "Multisensor aiding invalidated hover");
        assert(estimator.status.acceptedCorrectionCount == pre(lastCount) + 1,
          "Several corrections must increment the fusion-instant count once");
        if observedTick < 40 or observedTick >= 50 then
          assert(estimator.status.opticalFlowCorrectionAccepted,
            "Independent optical flow was starved by another aiding source");
        end if;
        assert(estimator.status.magnetometerCorrectionAccepted ==
            ((observedTick >= 40 and observedTick < 50) or
              mod(observedTick, 2) == 0),
          "Magnetometer must fuse each fresh packet once, alongside flow");
        assert(estimator.status.barometerCorrectionAccepted ==
            estimator.status.magnetometerCorrectionAccepted,
          "Barometer must fuse each fresh packet once, alongside flow");
        if observedTick >= 30 and observedTick < 40 then
          assert(estimator.status.gpsConsecutiveRejections > 0
              and estimator.status.rejectionElapsed_s > 0.0,
            "Supplemental acceptance hid a GPS anchor rejection");
        end if;
        if observedTick >= 40 and observedTick < 50 then
          assert(estimator.status.opticalFlowConsecutiveRejections > 0,
            "Accepted GPS hid a rejected optical-flow sample");
        end if;
      end if;
      lastCount := estimator.status.acceptedCorrectionCount;
    end when;
end MultisensorAidingTests;
