within Estimation.MultiSensorInvariant;

record State "Nominal state and full local-error tangent covariance"
  extends Estimation.MultiSensorInvariant.NominalState;
  Estimation.MultiSensorInvariant.Covariance covariance;
end State;
