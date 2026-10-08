within Tests;

block PreintegrationNoiseReplay
  "Held-input covariance used by preintegrated prediction"
  input Real angularVelocity[3];
  input Real specificForce[3];
  input Real interval_s;
  input Real gyroscopeDensity;
  input Real accelerometerDensity;
  output Real covariance[15, 15](each start=0, each fixed=true);
algorithm
  when sample(0, 0.01) then
    covariance := Estimation.StrapdownINS.ESKF.discreteProcessCovariance(
      Estimation.StrapdownINS.ESKF.continuousTransition(
        angularVelocity, specificForce),
      Estimation.StrapdownINS.ESKF.noiseInputMatrix(),
      Estimation.StrapdownINS.ESKF.processNoiseMatrix(
        Estimation.StrapdownINS.ProcessNoise(
          gyroscope_rad2_s=gyroscopeDensity * identity(3),
          accelerometer_m2_s3=accelerometerDensity * identity(3),
          gyroscopeBias_rad2_s3=zeros(3, 3),
          accelerometerBias_m2_s5=zeros(3, 3))),
      interval_s);
  end when;
end PreintegrationNoiseReplay;
