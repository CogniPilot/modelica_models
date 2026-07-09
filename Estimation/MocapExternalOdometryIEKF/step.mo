within Estimation.MocapExternalOdometryIEKF;
function step "One packet-driven discrete mocap IEKF step"
  input Real x_prev[157]
    "Flat state before this packet: attitude, velocity, position, angular velocity, covariance";
  input Real dt_s "Prediction interval from previous packet [s]";
  input Real measurement_valid
    "Greater than 0.5 when the pose measurement should be corrected";
  input Real measurement_position_enu_m[3] "Measured position [m]";
  input Real measurement_attitude_wxyz[4] "Measured attitude quaternion";
  input Real process_variance[4]
    "{attitude, velocity, position, angular velocity} spectral densities";
  input Real attitudeMeasurementVariance "Attitude measurement variance [rad2]";
  input Real positionMeasurementVariance "Position measurement variance [m2]";
  output Real x_next[157] "Flat state after prediction and optional correction";
  output Real correction_accepted
    "1 when a requested correction succeeded, 0 when skipped or singular";
protected
  Real x_pred[157];
algorithm
  x_pred := Estimation.MocapExternalOdometryIEKF.predict(
    x_prev,
    dt_s,
    process_variance);

  if measurement_valid > 0.5 then
    (x_next, correction_accepted) := Estimation.MocapExternalOdometryIEKF.correct(
      x_pred,
      measurement_position_enu_m,
      measurement_attitude_wxyz,
      attitudeMeasurementVariance,
      positionMeasurementVariance);
  else
    x_next := x_pred;
    correction_accepted := 0.0;
  end if;
end step;
