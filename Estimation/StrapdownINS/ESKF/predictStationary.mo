within Estimation.StrapdownINS.ESKF;

function predictStationary
  "Propagate a declared stationary vehicle with random-walk biases"
  input State previous;
  input Real dt(unit = "s");
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output State predicted;
protected
  Covariance transition;
  Covariance noise;
  Covariance noiseRoot;
  Boolean factorized;
algorithm
  transition := identity(TangentLength);
  transition[1:3, 4:6] := dt * identity(3);
  noise := zeros(TangentLength, TangentLength);
  noise[10:12, 10:12] := dt * processNoise.gyroscopeBias_rad2_s3;
  noise[13:15, 13:15] := dt * processNoise.accelerometerBias_m2_s5;
  predicted := State(
    positionWorldEnu_m=previous.positionWorldEnu_m
      + dt * previous.velocityWorldEnu_m_s,
    velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
    quaternionWorldBody=previous.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=previous.accelerometerBiasBodyFlu_m_s2,
    covariance=LinearAlgebra.symmetrize(
      LinearAlgebra.transformCovariance(transition, previous.covariance)
        + noise),
    covarianceRoot=previous.covarianceRoot,
    barometerBiasCrossCovariance=transition * previous.barometerBiasCrossCovariance,
    useSquareRootCovariance=previous.useSquareRootCovariance);
  if previous.useSquareRootCovariance then
    (noiseRoot, factorized) := LinearAlgebra.factorPSD(noise);
    assert(factorized, "Stationary process covariance is not positive semidefinite");
    predicted := withCovarianceRoot(predicted, LinearAlgebra.covarianceRoot(
      cat(2, transition * previous.covarianceRoot, noiseRoot)),
      predicted.barometerBiasCrossCovariance);
  end if;
end predictStationary;
