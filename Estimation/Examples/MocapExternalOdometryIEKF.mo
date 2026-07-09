within Estimation.Examples;

// Continuous-time prediction model for a 12D invariant/error-state EKF used to
// turn intermittent motion-capture poses into external odometry.
//
// Nominal state:
//   attitude_wxyz              world-to-body attitude quaternion
//   linear_velocity_enu_m_s    world-frame linear velocity
//   position_enu_m             world-frame position
//   angular_velocity_flu_rad_s body-frame angular velocity
//
// Tangent covariance order:
//   attitude error, linear velocity, position, angular velocity.
//
// Discrete pose correction, gating, dropout policy, covariance stabilization,
// and transport-specific scheduling are intentionally outside this model.

model MocapExternalOdometryIEKF
  parameter Real attitudeProcessVariance = 1.0e-6
    "attitude error random-walk spectral density [rad2/s]";
  parameter Real velocityProcessVariance = 1.0e-4
    "linear velocity random-walk spectral density [(m/s)2/s]";
  parameter Real positionProcessVariance = 1.0e-8
    "position random-walk spectral density [m2/s]";
  parameter Real angularVelocityProcessVariance = 1.0e-4
    "angular velocity random-walk spectral density [(rad/s)2/s]";

  Real attitude_wxyz[4](start = {1.0, 0.0, 0.0, 0.0})
    "world-to-body attitude quaternion as w,x,y,z";
  Real linear_velocity_enu_m_s[3](start = {0.0, 0.0, 0.0})
    "World-frame linear velocity [m/s]";
  Real position_enu_m[3](start = {0.0, 0.0, 0.0})
    "World-frame position [m]";
  Real angular_velocity_flu_rad_s[3](start = {0.0, 0.0, 0.0})
    "Body-frame angular velocity [rad/s]";
  Real covariance[12, 12](start = identity(12))
    "12D tangent covariance: attitude, velocity, position, angular velocity";

equation
  der(attitude_wxyz) = LieGroup.SO3.quaternionDerivative(
    attitude_wxyz,
    angular_velocity_flu_rad_s,
    0.0);

  der(linear_velocity_enu_m_s) = {0.0, 0.0, 0.0};
  der(position_enu_m) = linear_velocity_enu_m_s;
  der(angular_velocity_flu_rad_s) = {0.0, 0.0, 0.0};

  for i in 1:3 loop
    for j in 1:3 loop
      der(covariance[i, j]) =
        covariance[i + 9, j] + covariance[i, j + 9];
    end for;
    for j in 4:6 loop
      der(covariance[i, j]) = covariance[i + 9, j];
    end for;
    for j in 7:9 loop
      der(covariance[i, j]) = covariance[i + 9, j] + covariance[i, j - 3];
    end for;
    for j in 10:12 loop
      der(covariance[i, j]) = covariance[i + 9, j];
    end for;
  end for;

  for i in 4:6 loop
    for j in 1:3 loop
      der(covariance[i, j]) = covariance[i, j + 9];
    end for;
    for j in 4:6 loop
      der(covariance[i, j]) = 0.0;
    end for;
    for j in 7:9 loop
      der(covariance[i, j]) = covariance[i, j - 3];
    end for;
    for j in 10:12 loop
      der(covariance[i, j]) = 0.0;
    end for;
  end for;

  for i in 7:9 loop
    for j in 1:3 loop
      der(covariance[i, j]) = covariance[i - 3, j] + covariance[i, j + 9];
    end for;
    for j in 4:6 loop
      der(covariance[i, j]) = covariance[i - 3, j];
    end for;
    for j in 7:9 loop
      der(covariance[i, j]) =
        covariance[i - 3, j] + covariance[i, j - 3];
    end for;
    for j in 10:12 loop
      der(covariance[i, j]) = covariance[i - 3, j];
    end for;
  end for;

  for i in 10:12 loop
    for j in 1:3 loop
      der(covariance[i, j]) = covariance[i, j + 9];
    end for;
    for j in 4:6 loop
      der(covariance[i, j]) = 0.0;
    end for;
    for j in 7:9 loop
      der(covariance[i, j]) = covariance[i, j - 3];
    end for;
    for j in 10:12 loop
      der(covariance[i, j]) = 0.0;
    end for;
  end for;
end MocapExternalOdometryIEKF;
