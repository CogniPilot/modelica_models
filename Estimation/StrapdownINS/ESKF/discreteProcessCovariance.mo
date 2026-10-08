within Estimation.StrapdownINS.ESKF;

function discreteProcessCovariance
  "Simpson approximation of the exact held-input covariance integral"
  input Real A[TangentLength, TangentLength];
  input Real G[TangentLength, ProcessNoiseLength];
  input Estimation.StrapdownINS.ProcessNoiseCovariance continuousNoise;
  input Real dt(unit = "s");
  output Covariance covariance;
protected
  Real drivenNoise[TangentLength, TangentLength];
  Real halfTransition[TangentLength, TangentLength];
  Real fullTransition[TangentLength, TangentLength];
algorithm
  drivenNoise := LinearAlgebra.transformCovariance(G, continuousNoise);
  halfTransition := discreteTransition(A, 0.5 * dt);
  fullTransition := discreteTransition(A, dt);
  covariance := LinearAlgebra.symmetrize((dt / 6.0) * (
    drivenNoise
      + 4.0 * LinearAlgebra.transformCovariance(halfTransition, drivenNoise)
      + LinearAlgebra.transformCovariance(fullTransition, drivenNoise)));
end discreteProcessCovariance;
