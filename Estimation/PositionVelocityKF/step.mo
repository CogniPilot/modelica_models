within Estimation.PositionVelocityKF;
function step
  "One IMU prediction and optional fresh Cartesian-position correction"
  input Estimation.PositionVelocityKF.State previous;
  input Real dt "Prediction interval [s]";
  input Real attitude_wb[4]
    "Scalar-first quaternion rotating body FLU vectors to world ENU";
  input Real specificForce_b_m_s2[3]
    "Accelerometer specific force in body FLU coordinates [m/s2]";
  input Boolean measurementFresh
    "True only on a tick containing a new valid position measurement";
  input Real measuredPosition[3]
    "GPS- or mocap-derived position in world ENU coordinates [m]";
  input Estimation.PositionVelocityKF.Gain gain;
  input Real gravity_m_s2 = 9.81 "Positive gravitational acceleration";
  output Estimation.PositionVelocityKF.State next;
  output Real acceleration_w_m_s2[3]
    "Gravity-compensated inertial acceleration in world ENU [m/s2]";
  output Real residual[3] "Position innovation, or zero without correction [m]";
  output Boolean correctionApplied;
protected
  Estimation.PositionVelocityKF.State predicted;
algorithm
  acceleration_w_m_s2 :=
    Estimation.PositionVelocityKF.bodySpecificForceToWorld(
      attitude_wb,
      specificForce_b_m_s2,
      gravity_m_s2);
  predicted := Estimation.PositionVelocityKF.predict(
    previous,
    acceleration_w_m_s2,
    dt);
  if measurementFresh then
    (next, residual) := Estimation.PositionVelocityKF.correct(
      predicted,
      measuredPosition,
      gain);
    correctionApplied := true;
  else
    next := predicted;
    residual := zeros(3);
    correctionApplied := false;
  end if;
end step;
