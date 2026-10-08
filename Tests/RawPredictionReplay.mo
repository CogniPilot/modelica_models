within Tests;

block RawPredictionReplay
  input Real covariance[15, 15];
  input Real priorRoot[15, 15];
  input Real dt;
  input Boolean squareRoot;
  output Real predictedCovariance[15, 15];
  output Real nominal[16];
protected
  Estimation.StrapdownINS.ESKF.State predicted;
algorithm
  when sample(0.0, 0.01) then
    predicted := Estimation.StrapdownINS.ESKF.predict(
      Estimation.StrapdownINS.ESKF.State(
        barometerBiasCrossCovariance=zeros(15),
        positionWorldEnu_m=zeros(3), velocityWorldEnu_m_s={1.0, -2.0, 0.5},
        quaternionWorldBody={1.0, 0.0, 0.0, 0.0},
        gyroscopeBiasBodyFlu_rad_s=zeros(3),
        accelerometerBiasBodyFlu_m_s2=zeros(3), covariance=covariance,
        useSquareRootCovariance=squareRoot, covarianceRoot=priorRoot),
      zeros(3), zeros(3), zeros(3), dt,
      Estimation.StrapdownINS.ProcessNoise(zeros(3, 3), zeros(3, 3),
        zeros(3, 3), zeros(3, 3)));
    predictedCovariance := predicted.covariance;
    nominal := cat(1, predicted.positionWorldEnu_m, predicted.velocityWorldEnu_m_s,
      predicted.quaternionWorldBody, predicted.gyroscopeBiasBodyFlu_rad_s,
      predicted.accelerometerBiasBodyFlu_m_s2);
  end when;
end RawPredictionReplay;
