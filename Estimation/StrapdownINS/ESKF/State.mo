within Estimation.StrapdownINS.ESKF;

record State "Nominal state and full local-error tangent covariance"
  extends NominalState;
  Covariance covariance;
  Boolean useSquareRootCovariance;
  Covariance covarianceRoot;
  Real barometerBiasCrossCovariance[TangentLength];
end State;
