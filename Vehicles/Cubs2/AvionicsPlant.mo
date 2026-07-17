within Vehicles.Cubs2;

model AvionicsPlant "CUBS2 plant boundary presented to flight firmware"
  // Keep transitive CMM base packages visible to Rumoca source-root loading.
  import LieGroups;
  import RigidBody;

  // Start on the runway aligned with the first route leg from (0, 0) to
  // (-8, -8). Lateral controls remain neutral until the aircraft is airborne.
  Plant vehicle(
    q_start = {0.38268343236508984, 0.0, 0.0, -0.9238795325112867}
  );
  InnerLoop innerLoop;

  input Real pwm0_us(start = 1500.0);
  input Real pwm1_us(start = 1500.0);
  input Real pwm2_us(start = 1000.0);
  input Real pwm3_us(start = 1500.0);
  input Real pwm4_us(start = 1000.0);
  input Real pwm5_us(start = 0.0);
  input Real pwm6_us(start = 0.0);
  input Real pwm7_us(start = 0.0);
  input Real pwm8_us(start = 0.0);

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real vx_m_s;
  output Real vy_m_s;
  output Real vz_m_s;
  output Real airspeed_m_s;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real aileron_cmd;
  output Real elevator_cmd;
  output Real throttle_cmd;
  output Real rudder_cmd;
  output Real stick_roll_cmd;
  output Real stick_pitch_cmd;
  output Real stick_throttle_cmd;
  output Real stick_yaw_cmd;
  output Real armed_cmd;
  output Real current_waypoint;
  output Real desired_speed_m_s;
  output Real roll_command_rad;
  output Real course_error_rad;

  output Real odometry_timestamp_us;
  output Real odometry_x_m;
  output Real odometry_y_m;
  output Real odometry_z_m;
  output Real odometry_qw;
  output Real odometry_qx;
  output Real odometry_qy;
  output Real odometry_qz;
  output Real odometry_vx_m_s;
  output Real odometry_vy_m_s;
  output Real odometry_vz_m_s;
  output Real odometry_roll_rate_rad_s;
  output Real odometry_pitch_rate_rad_s;
  output Real odometry_yaw_rate_rad_s;
  output Real odometry_flags;
  output Real odometry_status;
  output Real odometry_source_id;
  output Real odometry_id;

equation
  stick_roll_cmd = -Vehicles.Interfaces.centeredPwmToUnit(pwm0_us);
  stick_pitch_cmd = -Vehicles.Interfaces.centeredPwmToUnit(pwm1_us);
  stick_throttle_cmd = Vehicles.Interfaces.throttlePwmToUnit(pwm2_us);
  stick_yaw_cmd = Vehicles.Interfaces.centeredPwmToUnit(pwm3_us);
  armed_cmd = noEvent(if pwm2_us > 1050.0 then 1.0 else 0.0);

  innerLoop.armed = armed_cmd;
  innerLoop.stick_roll = stick_roll_cmd;
  innerLoop.stick_pitch = stick_pitch_cmd;
  innerLoop.stick_yaw = stick_yaw_cmd;
  innerLoop.stick_throttle = stick_throttle_cmd;
  innerLoop.gyro = vehicle.gyro;
  innerLoop.up_body = vehicle.up_body;
  innerLoop.airspeed = vehicle.airspeed;

  vehicle.ail = innerLoop.ail;
  vehicle.elev = innerLoop.elev;
  vehicle.rud = innerLoop.rud;
  vehicle.thr = innerLoop.thr;

  aileron_cmd = innerLoop.ail;
  elevator_cmd = innerLoop.elev;
  throttle_cmd = innerLoop.thr;
  rudder_cmd = innerLoop.rud;

  time_s = time;
  x_m = vehicle.position[1];
  y_m = vehicle.position[2];
  z_m = vehicle.position[3];
  vx_m_s = vehicle.velocity[1];
  vy_m_s = vehicle.velocity[2];
  vz_m_s = vehicle.velocity[3];
  airspeed_m_s = vehicle.airspeed;
  roll_rad = atan2(
    2.0 * (vehicle.quat[1] * vehicle.quat[2] + vehicle.quat[3] * vehicle.quat[4]),
    1.0 - 2.0 * (vehicle.quat[2] * vehicle.quat[2]
      + vehicle.quat[3] * vehicle.quat[3])
  );
  pitch_rad = asin(MathUtilities.clip(
    2.0 * (vehicle.quat[1] * vehicle.quat[3] - vehicle.quat[4] * vehicle.quat[2]),
    -1.0,
    1.0
  ));
  yaw_rad = atan2(
    2.0 * (vehicle.quat[1] * vehicle.quat[4] + vehicle.quat[2] * vehicle.quat[3]),
    1.0 - 2.0 * (vehicle.quat[3] * vehicle.quat[3]
      + vehicle.quat[4] * vehicle.quat[4])
  );

  current_waypoint = pwm5_us;
  desired_speed_m_s = pwm6_us / 1000.0;
  roll_command_rad = pwm7_us / 1000.0;
  course_error_rad = pwm8_us / 1000.0;

  odometry_timestamp_us = 1000000.0 * time;
  odometry_x_m = vehicle.position[1];
  odometry_y_m = vehicle.position[2];
  odometry_z_m = vehicle.position[3];
  odometry_qw = vehicle.quat[1];
  odometry_qx = vehicle.quat[2];
  odometry_qy = vehicle.quat[3];
  odometry_qz = vehicle.quat[4];
  odometry_vx_m_s = vehicle.velocity[1];
  odometry_vy_m_s = vehicle.velocity[2];
  odometry_vz_m_s = vehicle.velocity[3];
  odometry_roll_rate_rad_s = vehicle.gyro[1];
  odometry_pitch_rate_rad_s = vehicle.gyro[2];
  odometry_yaw_rate_rad_s = vehicle.gyro[3];
  odometry_flags = 15.0;
  odometry_status = 1.0;
  odometry_source_id = 0.0;
  odometry_id = 1.0;
end AvionicsPlant;
