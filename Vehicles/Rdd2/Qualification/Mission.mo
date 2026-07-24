within Vehicles.Rdd2.Qualification;
model Mission
  parameter Real boxSide_m = 4.0;
  parameter Real horizontalPositionKp = 0.08;
  parameter Real horizontalVelocityKd = 0.18;
  parameter Real horizontalCommandLimit = 0.22;
  parameter Real boxLegDuration_s = 7.0;
  parameter Real maximumTilt_rad = 0.61086524;
  parameter Real inertia[3, 3] = diagonal({
    0.02166666666666667,
    0.02166666666666667,
    0.04000000000000001});
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
  Real desiredX_m;
  Real desiredY_m;
  Real segmentStartX_m;
  Real segmentStartY_m;
  Real segmentEndX_m;
  Real segmentEndY_m;
  Real segmentTime_s;
  Real positionStartDerivative[3, 4];
  Real positionEndDerivative[3, 4];
  Real positionControlPoint[3, 8];
  Real yawControlPoint[1, 4];
  Planning.Bezier.MultirotorTrajectory flatTrajectory;
  Planning.Bezier.FlatReference flatReference;
  Real feedforwardRoll_rad;
  Real feedforwardPitch_rad;
  Real throttleNorm;
  Real rollCommandNorm;
  Real pitchCommandNorm;
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
  output Real estimated_roll_rad;
  output Real estimated_pitch_rad;
  output Real estimated_yaw_rad;
  output Real motor0;
  output Real motor1;
  output Real motor2;
  output Real motor3;
  output Real target_x_m;
  output Real target_y_m;
  output Real target_vx_m_s;
  output Real target_vy_m_s;
  output Real target_ax_m_s2;
  output Real target_ay_m_s2;
  output Real mission_phase;

equation
  desiredAltitude_m = if time < 2.0 then 0.0
    else if time < 5.0 then (time - 2.0) * (2.0 / 3.0)
    else if time < 38.0 then 2.0
    else if time < 43.0 then 2.0 - (time - 38.0) * 0.4
    else 0.0;
  segmentStartX_m = if time < 13.0 then 0.0
    else if time < 27.0 then boxSide_m
    else 0.0;
  segmentStartY_m = if time < 20.0 then 0.0
    else if time < 34.0 then boxSide_m
    else 0.0;
  segmentEndX_m = if time < 6.0 then 0.0
    else if time < 20.0 then boxSide_m
    else 0.0;
  segmentEndY_m = if time < 13.0 then 0.0
    else if time < 27.0 then boxSide_m
    else 0.0;
  segmentTime_s = if time < 6.0 then 0.0
    else if time < 13.0 then time - 6.0
    else if time < 20.0 then time - 13.0
    else if time < 27.0 then time - 20.0
    else if time < 34.0 then time - 27.0
    else 0.0;
  positionStartDerivative = [
    segmentStartX_m, 0.0, 0.0, 0.0;
    segmentStartY_m, 0.0, 0.0, 0.0;
    2.0, 0.0, 0.0, 0.0];
  positionEndDerivative = [
    segmentEndX_m, 0.0, 0.0, 0.0;
    segmentEndY_m, 0.0, 0.0, 0.0;
    2.0, 0.0, 0.0, 0.0];
  positionControlPoint = Planning.Bezier.septicControlPoints(
    positionStartDerivative, positionEndDerivative, boxLegDuration_s);
  yawControlPoint = Planning.Bezier.cubicControlPoints(
    [0.0, 0.0], [0.0, 0.0], boxLegDuration_s);
  flatTrajectory = Planning.Bezier.evaluateMultirotor(
    positionControlPoint, yawControlPoint, boxLegDuration_s, segmentTime_s);
  flatReference = Planning.Bezier.flatReference(
    flatTrajectory, 2.0, 9.80665, inertia);
  desiredX_m = flatTrajectory.position[1];
  desiredY_m = flatTrajectory.position[2];
  feedforwardRoll_rad = atan2(
    flatReference.bodyToWorld[3, 2], flatReference.bodyToWorld[3, 3]);
  feedforwardPitch_rad = asin(MathUtilities.clip(
    -flatReference.bodyToWorld[3, 1], -1.0, 1.0));
  armed = time >= 1.0 and time < 44.0;
  throttleNorm = if time < 1.25 or time >= 44.0 then 0.0
    else MathUtilities.clip(
      0.688 + 0.10 * (desiredAltitude_m - plant.z_m)
        - 0.075 * plant.vz_m_s,
      0.38,
      0.82
    );
  rollCommandNorm = if time < 5.0 or time >= 43.0 then 0.0
    else MathUtilities.clip(
      feedforwardRoll_rad / maximumTilt_rad
        - horizontalPositionKp * (desiredY_m - plant.y_m)
        - horizontalVelocityKd
          * (flatTrajectory.velocity[2] - plant.vy_m_s),
      -horizontalCommandLimit,
      horizontalCommandLimit
    );
  pitchCommandNorm = if time < 5.0 or time >= 43.0 then 0.0
    else MathUtilities.clip(
      feedforwardPitch_rad / maximumTilt_rad
        + horizontalPositionKp * (desiredX_m - plant.x_m)
        + horizontalVelocityKd
          * (flatTrajectory.velocity[1] - plant.vx_m_s),
      -horizontalCommandLimit,
      horizontalCommandLimit
    );
  rcRollUs = 1500.0 + 500.0 * rollCommandNorm;
  rcPitchUs = 1500.0 + 500.0 * pitchCommandNorm;
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
  estimated_roll_rad = attitudeEstimate.euler_rad[1];
  estimated_pitch_rad = attitudeEstimate.euler_rad[2];
  estimated_yaw_rad = attitudeEstimate.euler_rad[3];
  motor0 = mixer.motor0;
  motor1 = mixer.motor1;
  motor2 = mixer.motor2;
  motor3 = mixer.motor3;
  target_x_m = desiredX_m;
  target_y_m = desiredY_m;
  target_vx_m_s = flatTrajectory.velocity[1];
  target_vy_m_s = flatTrajectory.velocity[2];
  target_ax_m_s2 = flatTrajectory.acceleration[1];
  target_ay_m_s2 = flatTrajectory.acceleration[2];
  mission_phase = if time < 5.0 then 0.0
    else if time < 6.0 then 1.0
    else if time < 13.0 then 2.0
    else if time < 20.0 then 3.0
    else if time < 27.0 then 4.0
    else if time < 38.0 then 5.0
    else 6.0;
end Mission;
