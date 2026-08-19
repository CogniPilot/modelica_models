within Estimation.MocapExternalOdometryErrorState;
record InitialVariances "Diagonal variances used to initialize the geometric error-state EKF"
  Real attitude "Attitude error variance [rad2]";
  Real velocity "Velocity error variance [(m/s)2]";
  Real position "Position error variance [m2]";
  Real angularVelocity "Angular velocity error variance [(rad/s)2]";
end InitialVariances;
