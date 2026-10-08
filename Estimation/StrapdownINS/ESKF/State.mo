within Estimation.StrapdownINS.ESKF;

record State "Nominal state and full local-error tangent covariance"
  extends NominalState;
  Covariance covariance;
  Boolean useSquareRootCovariance;
  Covariance covarianceRoot;
  Real barometerBiasCrossCovariance[TangentLength];
  Real barometerBias_m;
  Real barometerBiasVariance_m2;
  Boolean useJointBarometerBias;
end State;
