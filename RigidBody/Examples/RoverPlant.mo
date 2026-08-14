within RigidBody.Examples;

model RoverPlant
  extends RigidBody.RigidBody6DOF(
    mass = 18.0,
    g = 9.8,
    ixx = 1.8,
    iyy = 2.4,
    izz = 3.2,
    p_start = {0, 0, 0},
    v_b_start = {0, 0, 0},
    qnorm_gain = 1.0
  );

  parameter Real wheel_radius = 0.15 "Wheel radius [m]";
  parameter Real max_speed = 8.0 "Maximum forward speed [m/s]";
  parameter Real max_yaw_rate = 1.2 "Maximum yaw rate [rad/s]";
  parameter Real speed_tau = 0.35 "Longitudinal speed response time [s]";
  parameter Real yaw_tau = 0.25 "Yaw-rate response time [s]";
  parameter Real side_tau = 0.12 "Lateral velocity damping time [s]";
  parameter Real vertical_tau = 0.05 "Vertical velocity damping time [s]";
  parameter Real height_tau = 0.08 "Ground-height correction time [s]";
  parameter Real mag_world_enu[3] = {0.0, 0.21, -0.45}
    "Magnetic field in world ENU axes [Gauss]";

  input Real steering(start = 0.0) "Normalized steering command";
  input Real throttle(start = 0.0) "Normalized throttle command";

  output Real wheel_omega[2](start = {0, 0}) "Left/right wheel angular rate [rad/s]";
  output Real accel[3] "Body FLU accelerometer [m/s^2] (specific force)";
  output Real gyro[3] "Body FLU gyroscope [rad/s]";
  output Real mag[3] "Body FLU magnetometer [Gauss]";

protected
  Real target_speed;
  Real target_yaw_rate;
  Real gravity_b_local[3];

equation
  target_speed = max_speed * throttle;
  target_yaw_rate = max_yaw_rate * steering * (0.2 + 0.8 * min(1.0, abs(v_b[1]) / max_speed));
  gravity_b_local = transpose(R) * {0, 0, -g};

  F_b = {
    -mass * gravity_b_local[1] + mass * (target_speed - v_b[1]) / speed_tau,
    -mass * gravity_b_local[2] - mass * v_b[2] / side_tau,
    -mass * gravity_b_local[3] - mass * v_b[3] / vertical_tau - mass * p[3] / height_tau
  };
  M_b = {
    -3.0 * omega[1],
    -4.0 * omega[2],
    izz * (target_yaw_rate - omega[3]) / yaw_tau
  };

  wheel_omega[1] = v_b[1] / wheel_radius;
  wheel_omega[2] = v_b[1] / wheel_radius;

  accel = a_b;
  gyro = omega;
  mag = transpose(R) * mag_world_enu;
end RoverPlant;
