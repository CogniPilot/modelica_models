within Estimation.StrapdownINS.ESKF;

function predictCovariance "Propagate covariance in its selected representation"
  input State previous;
  input Covariance transition;
  input Covariance dynamics;
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  input Real dt;
  output State predicted;
protected
  Estimation.StrapdownINS.ProcessNoiseCovariance noise;
  Estimation.StrapdownINS.ProcessNoiseCovariance noiseRoot;
  Real drivenRoot[TangentLength, ProcessNoiseLength];
  Boolean factorized;
algorithm
  noise := processNoiseMatrix(processNoise);
  if previous.useSquareRootCovariance then
    (noiseRoot, factorized) := LinearAlgebra.factorPSD(noise);
    assert(factorized, "Process covariance is not positive semidefinite");
    drivenRoot := noiseInputMatrix() * noiseRoot;
    predicted := withCovarianceRoot(previous, LinearAlgebra.covarianceRoot(cat(2,
      transition * previous.covarianceRoot,
      sqrt(dt / 6.0) * drivenRoot,
      sqrt(2.0 * dt / 3.0) * discreteTransition(dynamics, 0.5 * dt) * drivenRoot,
      sqrt(dt / 6.0) * discreteTransition(dynamics, dt) * drivenRoot)),
      transition * previous.barometerBiasCrossCovariance);
  else
    predicted := withDenseCovariance(previous, LinearAlgebra.symmetrize(
      LinearAlgebra.transformCovariance(transition, previous.covariance)
        + discreteProcessCovariance(dynamics, noiseInputMatrix(), noise, dt)),
      transition * previous.barometerBiasCrossCovariance);
  end if;
end predictCovariance;
