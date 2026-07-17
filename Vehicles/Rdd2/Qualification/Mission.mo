within Vehicles.Rdd2.Qualification;
model Mission
  Vehicles.Rdd2.AvionicsPlant plant;
  Estimation.ComplementaryAttitude attitudeEstimate(samplePeriod=0.005);
  Vehicles.Rdd2.CommandMapping rcMapping(samplePeriod=0.000625);
  Vehicles.Rdd2.PidAxis attitudeRoll(
    samplePeriod=0.005,
    kp=4.0,
    ki=0.0,
    kd=0.0,
    i_limit=0.0,
    output_limit=6.0
  );
  Vehicles.Rdd2.PidAxis attitudePitch(
    samplePeriod=0.005,
    kp=4.0,
    ki=0.0,
    kd=0.0,
    i_limit=0.0,
    output_limit=6.0
  );
  Vehicles.Rdd2.PidAxis rateRoll(samplePeriod=0.000625);
  Vehicles.Rdd2.PidAxis ratePitch(samplePeriod=0.000625);
  Vehicles.Rdd2.PidAxis rateYaw(
    samplePeriod=0.000625,
    kp=0.20,
    ki=0.20,
    kd=0.0
  );
  Vehicles.Rdd2.Mixer mixer(samplePeriod=0.000625);

  Real desiredAltitude_m;
  Real throttleNorm;
  Real rcRollUs;
  Real rcPitchUs;
  Real rcThrottleUs;
  Real rcYawUs;
  Real rcArmUs;
  Boolean armed;
  Real rollRateDesired;
  Real pitchRateDesired;
  Real yawRateDesired;
  Real throttleCommand;

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real vz_m_s;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real motor0;
  output Real motor1;
  output Real motor2;
  output Real motor3;

equation
  desiredAltitude_m = if time < 2.0 then 0.0
    else if time < 5.0 then (time - 2.0) * (2.0 / 3.0)
    else if time < 14.0 then 2.0
    else if time < 18.0 then 2.0 - (time - 14.0) * 0.5
    else 0.0;
  armed = time >= 1.0 and time < 19.0;
  throttleNorm = if time < 1.25 or time >= 19.0 then 0.0
    else MathUtilities.clip(
      0.688 + 0.10 * (desiredAltitude_m - plant.z_m)
        - 0.075 * plant.vz_m_s,
      0.38,
      0.82
    );
  rcRollUs = if time >= 8.0 and time < 9.0 then 1625.0 else 1500.0;
  rcPitchUs = if time >= 11.0 and time < 12.0 then 1375.0 else 1500.0;
  rcThrottleUs = 1000.0 + 1000.0 * throttleNorm;
  rcYawUs = 1500.0;
  rcArmUs = if armed then 2000.0 else 1000.0;

  attitudeEstimate.gyro_rad_s = {
    plant.gyro_x_rad_s,
    plant.gyro_y_rad_s,
    plant.gyro_z_rad_s
  };
  attitudeEstimate.accel_m_s2 = {
    plant.accel_x_m_s2,
    plant.accel_y_m_s2,
    plant.accel_z_m_s2
  };
  attitudeEstimate.reset = not armed;

  rcMapping.rcRollUs = rcRollUs;
  rcMapping.rcPitchUs = rcPitchUs;
  rcMapping.rcThrottleUs = rcThrottleUs;
  rcMapping.rcYawUs = rcYawUs;
  rcMapping.rcArmUs = rcArmUs;
  rcMapping.attitudeRoll = attitudeEstimate.euler_rad[1];
  rcMapping.attitudePitch = attitudeEstimate.euler_rad[2];
  rcMapping.attitudeYaw = attitudeEstimate.euler_rad[3];
  rcMapping.throttleInputForCommand = throttleNorm;
  rcMapping.armed = armed;

  attitudeRoll.setpoint = rcMapping.attitudeDesiredRoll;
  attitudeRoll.measurement = attitudeEstimate.euler_rad[1];
  attitudeRoll.integrate = 0.0;
  attitudePitch.setpoint = rcMapping.attitudeDesiredPitch;
  attitudePitch.measurement = attitudeEstimate.euler_rad[2];
  attitudePitch.integrate = 0.0;

  rollRateDesired = attitudeRoll.pidOutput;
  pitchRateDesired = attitudePitch.pidOutput;
  yawRateDesired = rcMapping.yawRateDesired;
  throttleCommand = rcMapping.throttleCommand;

  rateRoll.setpoint = rollRateDesired;
  rateRoll.measurement = plant.gyro_x_rad_s;
  rateRoll.integrate = if rcMapping.ratePidIntegrate then 1.0 else 0.0;
  ratePitch.setpoint = pitchRateDesired;
  ratePitch.measurement = plant.gyro_y_rad_s;
  ratePitch.integrate = if rcMapping.ratePidIntegrate then 1.0 else 0.0;
  rateYaw.setpoint = yawRateDesired;
  rateYaw.measurement = plant.gyro_z_rad_s;
  rateYaw.integrate = if rcMapping.ratePidIntegrate then 1.0 else 0.0;

  mixer.throttle = throttleCommand;
  mixer.rateCmdRoll = rateRoll.pidOutput;
  mixer.rateCmdPitch = ratePitch.pidOutput;
  mixer.rateCmdYaw = rateYaw.pidOutput;

  plant.motor0 = mixer.motor0;
  plant.motor1 = mixer.motor1;
  plant.motor2 = mixer.motor2;
  plant.motor3 = mixer.motor3;

  mixer.armed = armed;

  time_s = time;
  x_m = plant.x_m;
  y_m = plant.y_m;
  z_m = plant.z_m;
  vz_m_s = plant.vz_m_s;
  roll_rad = plant.roll_rad;
  pitch_rad = plant.pitch_rad;
  yaw_rad = plant.yaw_rad;
  motor0 = mixer.motor0;
  motor1 = mixer.motor1;
  motor2 = mixer.motor2;
  motor3 = mixer.motor3;
end Mission;
