within Estimation.StrapdownINS.ESKF;

function discreteProcessCovariance
  "Simpson approximation of the exact held-input covariance integral"
  input Real A[TangentLength, TangentLength];
  input Real G[TangentLength, ProcessNoiseLength];
  input Estimation.StrapdownINS.ProcessNoiseCovariance continuousNoise;
  input Real dt(unit = "s");
  output Estimation.StrapdownINS.ESKF.Covariance covariance;
protected
  Real drivenNoise[TangentLength, TangentLength];
  Real halfTransition[TangentLength, TangentLength];
  Real fullTransition[TangentLength, TangentLength];
algorithm
  drivenNoise := G * continuousNoise * transpose(G);
  halfTransition := discreteTransition(A, 0.5 * dt);
  fullTransition := discreteTransition(A, dt);
  covariance := LinearAlgebra.symmetrize((dt / 6.0) * (
    drivenNoise
      + 4.0 * halfTransition * drivenNoise * transpose(halfTransition)
      + fullTransition * drivenNoise * transpose(fullTransition)));
end discreteProcessCovariance;
