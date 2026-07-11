within Estimation.MocapExternalOdometryIEKF;
record ProcessNoise "Continuous-time diagonal process-noise spectral densities"
  Real attitude "Attitude error spectral density [rad2/s]";
  Real velocity "Velocity error spectral density [(m/s)2/s]";
  Real position "Position error spectral density [m2/s]";
  Real angularVelocity "Angular velocity error spectral density [(rad/s)2/s]";
end ProcessNoise;
