within SLAM.Inertial;
// Convenience propagation for callers that need only the current-state block.
// Joint-state callers use ES15TransitionNoise and propagate covariance once.
function ES15CovarianceStep
  import ES15TransitionNoise = SLAM.Inertial.ES15TransitionNoise;

  input Real F[15,15];
  input Real G[15,12];
  input Real P[15,15];
  input Real dt;
  input Real density[12];
  output Real Phi[15,15];
  output Real Q[15,15];
  output Real predicted[15,15];
protected
  Real propagated[15,15]; Real raw[15,15];
algorithm
  (Phi,Q) := ES15TransitionNoise(F,G,dt,density);
  propagated := Phi*P*transpose(Phi);
  raw := propagated+Q;
  predicted := 0.5*(raw+transpose(raw));
end ES15CovarianceStep;
