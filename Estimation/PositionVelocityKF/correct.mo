within Estimation.PositionVelocityKF;
function correct "Apply a fixed steady-state gain to a position measurement"
  input Estimation.PositionVelocityKF.State predicted;
  input Real measuredPosition[3]
    "GPS- or mocap-derived position in world ENU coordinates [m]";
  input Estimation.PositionVelocityKF.Gain gain;
  output Estimation.PositionVelocityKF.State corrected;
  output Real residual[3] "Measured minus predicted position [m]";
protected
  Real correction[6];
algorithm
  residual := measuredPosition - predicted.position;
  correction := gain * residual;
  corrected.position := predicted.position + correction[1:3];
  corrected.velocity := predicted.velocity + correction[4:6];
end correct;
