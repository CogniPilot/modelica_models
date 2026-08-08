within Vehicles.Cubs2;

model AvionicsPlant "CUBS2 PWM and odometry boundary around the physical plant"
  // Keep transitive CMM base packages visible to Rumoca source-root loading.
  import LieGroups;
  import RigidBody;

  Plant vehicle(
    q_start = {0.38268343236508984, 0.0, 0.0, -0.9238795325112867});
  OnboardStabilizerSurrogate onboardStabilizer;

  input Real pwm_us[9](start = {
    1500.0,
    1500.0,
    1000.0,
    1500.0,
    1000.0,
    0.0,
    0.0,
    0.0,
    0.0});

  output Real time_s;
  output Real position_m[3];
  output Real velocity_m_s[3];
  output Real airspeed_m_s;
  output Real euler_rad[3] "{roll, pitch, yaw}";
  output Real actuatorCommand[4] "{aileron, elevator, rudder, throttle}";
  output Real stickCommand[4] "{roll, pitch, yaw, throttle}";
  output Real armCommand "continuous normalized arm command [0, 1]";
  output Vehicles.Cubs2.Interfaces.AutopilotDebug autopilotDebug;
  output Vehicles.Cubs2.Interfaces.Odometry odometry;

protected
  Real eulerB321_rad[3] "{yaw, pitch, roll}";

equation
  stickCommand = {
    -Vehicles.Interfaces.centeredPwmToUnit(pwm_us[1]),
    -Vehicles.Interfaces.centeredPwmToUnit(pwm_us[2]),
    Vehicles.Interfaces.centeredPwmToUnit(pwm_us[4]),
    Vehicles.Interfaces.throttlePwmToUnit(pwm_us[3])};
  armCommand = MathUtilities.clip((pwm_us[3] - 1000.0) / 50.0, 0.0, 1.0);
  onboardStabilizer.armed = armCommand;
  onboardStabilizer.pilotCommand = stickCommand;
  onboardStabilizer.gyro = vehicle.gyro;
  onboardStabilizer.up_body = vehicle.up_body;
  onboardStabilizer.airspeed = vehicle.airspeed;

  actuatorCommand = onboardStabilizer.surfaceCommand;
  {vehicle.ail, vehicle.elev, vehicle.rud, vehicle.thr} = actuatorCommand;

  time_s = time;
  position_m = vehicle.position;
  velocity_m_s = vehicle.velocity;
  airspeed_m_s = vehicle.airspeed;
  eulerB321_rad = LieGroups.SO3.EulerB321.from_Quat(vehicle.quat);
  euler_rad = {eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]};

  autopilotDebug.currentWaypoint = pwm_us[6];
  autopilotDebug.desiredSpeed_m_s = pwm_us[7] / 1000.0;
  autopilotDebug.rollCommand_rad = pwm_us[8] / 1000.0;
  autopilotDebug.courseError_rad = pwm_us[9] / 1000.0;

  odometry.timestamp_us = 1000000.0 * time;
  odometry.position_m = vehicle.position;
  odometry.quaternion = vehicle.quat;
  odometry.velocity_m_s = vehicle.velocity;
  odometry.angularVelocity_rad_s = vehicle.gyro;
  odometry.flags = 15.0;
  odometry.status = 1.0;
  odometry.sourceId = 0.0;
  odometry.id = 1.0;
end AvionicsPlant;
