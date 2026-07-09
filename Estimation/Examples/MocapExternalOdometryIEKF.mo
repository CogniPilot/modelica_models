within Estimation.Examples;

// Packet-driven discrete wrapper for the mocap IEKF step used by
// synapse_qualisys_bridge. The flat state layout is:
//   1:4     attitude_wxyz
//   5:7     linear_velocity_enu_m_s
//   8:10    position_enu_m
//   11:13   angular_velocity_flu_rad_s
//   14:157  12x12 tangent covariance, column-major

model MocapExternalOdometryIEKF
  parameter Real attitudeProcessVariance = 1.0e-6
    "attitude error random-walk spectral density [rad2/s]";
  parameter Real velocityProcessVariance = 1.0e-4
    "linear velocity random-walk spectral density [(m/s)2/s]";
  parameter Real positionProcessVariance = 1.0e-8
    "position random-walk spectral density [m2/s]";
  parameter Real angularVelocityProcessVariance = 1.0e-4
    "angular velocity random-walk spectral density [(rad/s)2/s]";
  parameter Real attitudeMeasurementVariance = 0.010^2
    "attitude measurement variance [rad2]";
  parameter Real positionMeasurementVariance = 0.004^2
    "position measurement variance [m2]";

  input Real x_prev[157](start = zeros(157))
    "Previous flat IEKF state";
  input Real dt_s(start = 1.0 / 240.0)
    "Packet interval since the previous estimator update [s]";
  input Real measurement_valid(start = 1.0)
    "Greater than 0.5 when measurement_position_enu_m and measurement_attitude_wxyz are valid";
  input Real measurement_position_enu_m[3](start = {0.0, 0.0, 0.0})
    "Measured position [m]";
  input Real measurement_attitude_wxyz[4](start = {1.0, 0.0, 0.0, 0.0})
    "Measured attitude quaternion";

  output Real x_next[157]
    "Next flat IEKF state after prediction and optional correction";
  output Real correction_accepted
    "1 when a requested correction succeeded, 0 when skipped or singular";

equation
  (x_next, correction_accepted) = Estimation.MocapExternalOdometryIEKF.step(
    x_prev,
    dt_s,
    measurement_valid,
    measurement_position_enu_m,
    measurement_attitude_wxyz,
    {
      attitudeProcessVariance,
      velocityProcessVariance,
      positionProcessVariance,
      angularVelocityProcessVariance},
    attitudeMeasurementVariance,
    positionMeasurementVariance);

  annotation(Documentation(info="<html>
    <p>
      This is a discrete packet update, not a continuous ODE model. It is
      intended for bridges that call the estimator once per mocap packet with
      the previous flat state and the packet interval.
    </p>
    <p>
      The prediction uses the closed-form mixed exponential on SE_2(3),
      following Lin, Li-Yu, et al.,
      &quot;On Closed-Form Preintegration for a Class of Mixed-Invariant Systems
      in SEn(3),&quot; IEEE Control Systems Letters, 2025.
    </p>
  </html>"));
end MocapExternalOdometryIEKF;
