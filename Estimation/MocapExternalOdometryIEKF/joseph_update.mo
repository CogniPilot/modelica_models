within Estimation.MocapExternalOdometryIEKF;
function joseph_update "Joseph-form covariance correction"
  input Real a[12, 12] "I - K H";
  input Real covariance[12, 12] "Predicted covariance";
  input Real gain[12, 6] "Kalman gain";
  input Real measurement_noise[6, 6] "Measurement covariance";
  output Real covariance_next[12, 12];
protected
  Real ap[12, 12];
  Real kr[12, 6];
algorithm
  ap := a * covariance;
  covariance_next := ap * transpose(a);
  kr := gain * measurement_noise;
  covariance_next := covariance_next + kr * transpose(gain);
end joseph_update;
