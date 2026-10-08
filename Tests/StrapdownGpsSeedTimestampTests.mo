within Tests;

model StrapdownGpsSeedTimestampTests
  extends StrapdownEstimatorInterfaceTests.Harness(
    gpsEnabled=true, gpsInvalidTimestampUntil_s=0.01,
    estimator(samplePeriod=0.01));
  discrete Boolean validPacketAccepted(start=false, fixed=true);
equation
  assert(time <= 0.015 or validPacketAccepted,
    "An unusable seed timestamp blocked the subsequent valid GPS packet");
algorithm
  when sample(0.0, 0.01) then
    validPacketAccepted := pre(validPacketAccepted)
      or estimator.status.gpsPositionCorrectionAccepted;
  end when;
  annotation(experiment(StartTime=0.0, StopTime=0.02,
    Tolerance=1.0e-8, Interval=0.005));
end StrapdownGpsSeedTimestampTests;
