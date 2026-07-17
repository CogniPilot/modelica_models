within Vehicles.Rdd2;
model AvionicsPlant "RDD2 plant boundary presented to flight firmware"
  parameter Real omegaMax(unit = "rad/s") = 1100.0;
  Plant vehicle;

  input Real motor0(start = 0.0);
  input Real motor1(start = 0.0);
  input Real motor2(start = 0.0);
  input Real motor3(start = 0.0);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real vx_m_s;
  output Real vy_m_s;
  output Real vz_m_s;
  output Real qw;
  output Real qx;
  output Real qy;
  output Real qz;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real gyro_x_rad_s;
  output Real gyro_y_rad_s;
  output Real gyro_z_rad_s;
  output Real accel_x_m_s2;
  output Real accel_y_m_s2;
  output Real accel_z_m_s2;
  output Real motor_omega0_rad_s;
  output Real motor_omega1_rad_s;
  output Real motor_omega2_rad_s;
  output Real motor_omega3_rad_s;

equation
  vehicle.omega_cmd = omegaMax * {
    MathUtilities.clip(motor0, 0.0, 1.0),
    MathUtilities.clip(motor1, 0.0, 1.0),
    MathUtilities.clip(motor2, 0.0, 1.0),
    MathUtilities.clip(motor3, 0.0, 1.0)
  };
  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  vx_m_s = vehicle.velocity[1];
  vy_m_s = vehicle.velocity[2];
  vz_m_s = vehicle.velocity[3];
  qw = vehicle.quat[1];
  qx = vehicle.quat[2];
  qy = vehicle.quat[3];
  qz = vehicle.quat[4];
  roll_rad = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  pitch_rad = asin(MathUtilities.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
  yaw_rad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  gyro_x_rad_s = vehicle.omega[1];
  gyro_y_rad_s = vehicle.omega[2];
  gyro_z_rad_s = vehicle.omega[3];
  accel_x_m_s2 = vehicle.a_b[1];
  accel_y_m_s2 = vehicle.a_b[2];
  accel_z_m_s2 = vehicle.a_b[3];
  motor_omega0_rad_s = vehicle.omega_m[1];
  motor_omega1_rad_s = vehicle.omega_m[2];
  motor_omega2_rad_s = vehicle.omega_m[3];
  motor_omega3_rad_s = vehicle.omega_m[4];
end AvionicsPlant;
