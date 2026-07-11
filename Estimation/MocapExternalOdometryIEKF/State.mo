within Estimation.MocapExternalOdometryIEKF;
record State "Complete IEKF state"
  extends Estimation.MocapExternalOdometryIEKF.NominalState;
  Covariance covariance
    "Full tangent covariance ordered {attitude, velocity, position, angular velocity}";
end State;
