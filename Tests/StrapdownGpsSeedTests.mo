within Tests;

model StrapdownGpsSeedTests
  extends StrapdownEstimatorInterfaceTests.Harness(
    gpsEnabled=true, gpsPositionVariance_m2=0.25,
    estimator(samplePeriod=0.01,
      initialVariances=Estimation.StrapdownINS.InitialVariances(
        position_m2=fill(0.04, 3), velocity_m2_s2=fill(0.01, 3),
        attitude_rad2=fill(0.01, 3), gyroscopeBias_rad2_s2=fill(0.01, 3),
        accelerometerBias_m2_s4=fill(0.01, 3))));
equation
  assert(not estimator.status.gpsPositionCorrectionAccepted,
    "The position seed's held GPS packet was fused again after initialization");
  assert(time <= 0.0 or estimator.errorCovariance[1, 1] >= 0.25 - 1e-10,
    "GPS-seeded position covariance understates the supplied measurement noise");
  annotation(experiment(StartTime=0.0, StopTime=0.02,
    Tolerance=1.0e-8, Interval=0.005));
end StrapdownGpsSeedTests;
