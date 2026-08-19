within Estimation.MocapExternalOdometryErrorState;
record State "Complete geometric error-state EKF state"
  extends Estimation.MocapExternalOdometryErrorState.NominalState;
  Covariance covariance
    "Full tangent covariance ordered {attitude, velocity, position, angular velocity}";
end State;
