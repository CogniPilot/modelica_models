within Estimation.MultiSensorInvariant;

record State "Nominal state and full invariant tangent covariance"
  extends Estimation.MultiSensorInvariant.NominalState;
  Estimation.MultiSensorInvariant.Covariance covariance;
end State;
