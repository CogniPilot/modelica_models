within Vehicles.Rdd2;
model Controller "RDD2 sampled flight-control algorithms"
  parameter Real samplePeriod(unit="s") = 0.001;
  constant Real unwindTau = 0.001;
  constant Real rcUsCenter = 1500.0;
  constant Real rcUsMin = 1000.0;
  constant Real rcUsMax = 2000.0;
  constant Real armSwitchThresholdUs = 1600.0;
  constant Real maxRollPitchRateRadS = 6.0;
  constant Real maxYawRateRadS = 3.5;
  constant Real maxAutoLevelTiltRad = 0.61086524;
  constant Real motorIdleThrottle = 0.0;
  constant Real pidIntegrateThrottleMin = 0.02;
  parameter Real kp = 0.12;
  parameter Real ki = 0.35;
  parameter Real kd = 0.0015;
  parameter Real i_limit = 0.2;
  parameter Real output_limit = 0.35;
  parameter Real f_cut = 25.0;
  input Real setpoint(start=0.0);
  input Real measurement(start=0.0);
  input Real integrate(start=0.0);
  input Real rcRollUs(start=1500.0);
  input Real rcPitchUs(start=1500.0);
  input Real rcThrottleUs(start=1000.0);
  input Real rcYawUs(start=1500.0);
  input Real rcArmUs(start=1000.0);
  input Real attitudeRoll(start=0.0);
  input Real attitudePitch(start=0.0);
  input Real attitudeYaw(start=0.0);
  input Real throttle(start=0.0);
  input Real throttleInputForCommand(start=0.0);
  input Real rateCmdRoll(start=0.0);
  input Real rateCmdPitch(start=0.0);
  input Real rateCmdYaw(start=0.0);
  input Boolean armed(start=false);
  discrete Real e_int(start=0.0);
  discrete Real meas_filt(start=0.0);
  discrete Boolean meas_filt_valid(start=false);
  discrete Real derivative(start=0.0);
  discrete Real rollNorm(start=0.0);
  discrete Real pitchNorm(start=0.0);
  discrete Real throttleNorm(start=0.0);
  discrete Real yawNorm(start=0.0);
  discrete Real correction0(start=0.0);
  discrete Real correction1(start=0.0);
  discrete Real correction2(start=0.0);
  discrete Real correction3(start=0.0);
  discrete Real maxUp(start=0.0);
  discrete Real maxDown(start=0.0);
  discrete Real scaleUp(start=1.0);
  discrete Real scale(start=1.0);
  discrete output Real pidError(start=0.0);
  discrete output Real pidOutput(start=0.0);
  discrete output Boolean armSwitchHigh(start=false);
  discrete output Boolean ratePidIntegrate(start=false);
  discrete output Real throttleInput(start=0.0);
  discrete output Real throttleCommand(start=0.0);
  discrete output Real yawRateDesired(start=0.0);
  discrete output Real acroRateDesiredRoll(start=0.0);
  discrete output Real acroRateDesiredPitch(start=0.0);
  discrete output Real acroRateDesiredYaw(start=0.0);
  discrete output Real attitudeDesiredRoll(start=0.0);
  discrete output Real attitudeDesiredPitch(start=0.0);
  discrete output Real attitudeDesiredYaw(start=0.0);
  discrete output Real motor0(start=0.0);
  discrete output Real motor1(start=0.0);
  discrete output Real motor2(start=0.0);
  discrete output Real motor3(start=0.0);
equation
  when sample(0.0, samplePeriod) then
    pidError = setpoint - measurement;
    derivative = if pre(meas_filt_valid) and f_cut > 0.0 then
        (measurement - pre(meas_filt)) / (1.0 / (6.283185307179586 * f_cut))
      else
        0.0;
    meas_filt = if f_cut > 0.0 then
        if pre(meas_filt_valid) then
          pre(meas_filt) + samplePeriod * derivative
        else
          measurement
      else
        measurement;
    meas_filt_valid = true;
    e_int = if integrate > 0.5 then
        MathUtilities.clip(
          pre(e_int) + (ki * pidError * samplePeriod), -i_limit, i_limit)
      else
        pre(e_int) * max(0.0, 1.0 - (samplePeriod / unwindTau));
    pidOutput = MathUtilities.clip(
      (kp * pidError) + e_int - (kd * derivative), -output_limit, output_limit);

    rollNorm = MathUtilities.clip((rcRollUs - rcUsCenter) / 500.0, -1.0, 1.0);
    pitchNorm = MathUtilities.clip((rcPitchUs - rcUsCenter) / 500.0, -1.0, 1.0);
    yawNorm = MathUtilities.clip((rcYawUs - rcUsCenter) / 500.0, -1.0, 1.0);
    throttleNorm = MathUtilities.clip(
      (rcThrottleUs - rcUsMin) / (rcUsMax - rcUsMin), 0.0, 1.0);

    armSwitchHigh = rcArmUs > armSwitchThresholdUs;
    throttleInput = throttleNorm;
    throttleCommand = if armed then
        motorIdleThrottle + throttleInputForCommand * (1.0 - motorIdleThrottle)
      else
        0.0;
    ratePidIntegrate = armed and throttleInputForCommand > pidIntegrateThrottleMin;

    yawRateDesired = -yawNorm * maxYawRateRadS;
    acroRateDesiredRoll = rollNorm * maxRollPitchRateRadS;
    acroRateDesiredPitch = pitchNorm * maxRollPitchRateRadS;
    acroRateDesiredYaw = yawRateDesired;

    attitudeDesiredRoll = rollNorm * maxAutoLevelTiltRad;
    attitudeDesiredPitch = pitchNorm * maxAutoLevelTiltRad;
    attitudeDesiredYaw = attitudeYaw;

    correction0 = -rateCmdRoll - rateCmdPitch - rateCmdYaw;
    correction1 = -rateCmdRoll + rateCmdPitch + rateCmdYaw;
    correction2 = rateCmdRoll + rateCmdPitch - rateCmdYaw;
    correction3 = rateCmdRoll - rateCmdPitch + rateCmdYaw;
    maxUp = max(0.0, max(correction0, max(correction1, max(correction2, correction3))));
    maxDown = max(0.0, max(-correction0, max(-correction1, max(-correction2, -correction3))));
    scaleUp = if maxUp > (1.0 - throttle) and maxUp > 0.0 then
        (1.0 - throttle) / maxUp
      else
        1.0;
    scale = if maxDown > throttle and maxDown > 0.0 then
        min(scaleUp, throttle / maxDown)
      else
        scaleUp;
    motor0 = MathUtilities.clip(throttle + correction0 * scale, 0.0, 1.0);
    motor1 = MathUtilities.clip(throttle + correction1 * scale, 0.0, 1.0);
    motor2 = MathUtilities.clip(throttle + correction2 * scale, 0.0, 1.0);
    motor3 = MathUtilities.clip(throttle + correction3 * scale, 0.0, 1.0);
  end when;
end Controller;
