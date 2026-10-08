within SLAM.Inertial;
// Equation adapter preserving the existing model interface and input defaults.
model ES15CovariancePrediction
  import ES15CovarianceStep = SLAM.Inertial.ES15CovarianceStep;

  input Real F[15,15] = fill(0.0,15,15);
  input Real G[15,12] = fill(0.0,15,12);
  input Real P[15,15] = identity(15);
  input Real dt = 1.0/90.0;
  input Real density[12] = {0.06,0.06,0.06,0.006,0.006,0.006,
                           0.002,0.002,0.002,0.0002,0.0002,0.0002};
  output Real Phi[15,15];
  output Real Q[15,15];
  output Real predicted[15,15];
equation
  (Phi,Q,predicted) = ES15CovarianceStep(F,G,P,dt,density);
end ES15CovariancePrediction;
