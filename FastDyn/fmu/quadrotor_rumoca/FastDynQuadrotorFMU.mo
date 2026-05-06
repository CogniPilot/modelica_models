model FastDynQuadrotorFMU
  QuadrotorSIL plant(ground_z = 0.0);

  parameter Real pwm_min = 1000.0 "Minimum motor PWM";
  parameter Real pwm_max = 2000.0 "Maximum motor PWM";
  parameter Real omega_min = 0.0 "Motor speed at minimum PWM [rad/s]";
  parameter Real omega_max = 900.0 "Motor speed at maximum PWM [rad/s]";
  parameter Real omega_hover = 757.0 "Approximate hover motor speed [rad/s]";
  parameter Real lat0 = 47.397742 "Reference latitude [deg]";
  parameter Real lon0 = 8.545594 "Reference longitude [deg]";
  parameter Real ground_alt_msl = 530.0 "Mean sea level altitude of the local ground collision plane [m]";
  parameter Real accel_bias[3] = {0, 0, 0} "Accelerometer bias [m/s^2]";
  parameter Real gyro_bias[3] = {0, 0, 0} "Gyroscope bias [rad/s]";
  parameter Real mag_bias[3] = {0, 0, 0} "Magnetometer bias [Gauss]";
  parameter Real gps_bias[3] = {0, 0, 0} "GPS bias N/E/altitude [m]";
  parameter Real baro_alt_bias = 0.0 "Barometer relative altitude bias [m]";
  parameter Real pi = 3.141592653589793;
  parameter Real meters_per_deg_lat = 111111.0;
  parameter Real meters_per_deg_lon = 111111.0 * cos(lat0 * pi / 180.0);

  input Real pwm[4](start = {1000, 1000, 1000, 1000}) "Motor PWM commands";

  output Real accel[3] "Body FRD accelerometer [m/s^2]";
  output Real gyro[3] "Body FRD gyroscope [rad/s]";
  output Real mag[3] "Body FRD magnetometer [Gauss]";
  output Real gps[3] "GPS latitude, longitude, altitude";
  output Real vel_ned[3] "GPS velocity NED [m/s]";
  output Real yaw_deg "Yaw [deg]";
  output Real baro[4] "Barometer altitude, pressure, temperature, climb rate";
  output Real motor_pos[2](start = {0, 0}) "Motor 1 and 3 integrated shaft positions [rad]";
  output Real motor_cmd[4] "Motor commands after PWM scaling [rad/s]";

protected
  Real pwm_span;
  Real pwm_norm[4];
  Real yaw_rad;

equation
  pwm_span = pwm_max - pwm_min;

  for i in 1:4 loop
    pwm_norm[i] = min(1.0, max(0.0, (pwm[i] - pwm_min) / pwm_span));
    motor_cmd[i] = if pwm_norm[i] <= 0.5
                   then omega_min + (omega_hover - omega_min) * pwm_norm[i] / 0.5
                   else omega_hover + (omega_max - omega_hover) * (pwm_norm[i] - 0.5) / 0.5;
    plant.omega_cmd[i] = motor_cmd[i];
  end for;

  accel = plant.accel + accel_bias;
  gyro = plant.gyro + gyro_bias;
  mag = plant.mag + mag_bias;

  gps[1] = lat0 + (plant.p[1] + gps_bias[1]) / meters_per_deg_lat;
  gps[2] = lon0 + (-plant.p[2] + gps_bias[2]) / meters_per_deg_lon;
  gps[3] = ground_alt_msl + plant.p[3] + gps_bias[3];

  vel_ned[1] = plant.v_w[1];
  vel_ned[2] = -plant.v_w[2];
  vel_ned[3] = -plant.v_w[3];

  yaw_rad = atan2(2.0 * (plant.q[1] * plant.q[4] + plant.q[2] * plant.q[3]),
                  1.0 - 2.0 * (plant.q[3] * plant.q[3] + plant.q[4] * plant.q[4]));
  yaw_deg = -yaw_rad * 180.0 / pi;

  baro[1] = plant.p[3] + baro_alt_bias;
  baro[3] = 15.0 - 0.0065 * (ground_alt_msl + baro[1]);
  baro[2] = 101325.0 * (1.0 - 2.25577e-5 * (ground_alt_msl + baro[1])) ^ 5.25588;
  baro[4] = plant.v_w[3];

  der(motor_pos[1]) = plant.omega_m[1];
  der(motor_pos[2]) = plant.omega_m[3];
end FastDynQuadrotorFMU;
