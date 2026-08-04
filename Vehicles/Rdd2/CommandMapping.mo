within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block CommandMapping "Map pilot RC inputs into rate, attitude, and throttle demands"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit="s") = 0.001;
  parameter Real rcUsCenter = 1500.0;
  parameter Real rcUsMin = 1000.0;
  parameter Real rcUsMax = 2000.0;
  parameter Real armSwitchThresholdUs = 1600.0;
  parameter Real maxRollPitchRateRadS(unit="rad/s") = 6.0;
  parameter Real maxYawRateRadS(unit="rad/s") = 3.5;
  parameter Real maxAutoLevelTiltRad(unit="rad") = 0.61086524;
  parameter Real motorIdleThrottle = 0.0;

  Interfaces.PilotInput pilot;
  Interfaces.PilotCommands commands;

protected
  discrete Real normalizedRoll(start=0.0);
  discrete Real normalizedPitch(start=0.0);
  discrete Real normalizedYaw(start=0.0);

algorithm
  when sample(0.0, samplePeriod) then
    normalizedRoll := MathUtilities.clip(
      (pilot.rcUs[1] - rcUsCenter) / 500.0, -1.0, 1.0);
    normalizedPitch := MathUtilities.clip(
      (pilot.rcUs[2] - rcUsCenter) / 500.0, -1.0, 1.0);
    normalizedYaw := MathUtilities.clip(
      (pilot.rcUs[4] - rcUsCenter) / 500.0, -1.0, 1.0);
    commands.throttleInput := MathUtilities.clip(
      (pilot.rcUs[3] - rcUsMin) / (rcUsMax - rcUsMin), 0.0, 1.0);

    commands.armSwitchHigh := pilot.rcUs[5] > armSwitchThresholdUs;
    if pilot.armed then
      commands.throttleCommand := motorIdleThrottle
        + pilot.throttleForCommand * (1.0 - motorIdleThrottle);
    else
      commands.throttleCommand := 0.0;
    end if;
    commands.acroRateDesired_rad_s[1] :=
      normalizedRoll * maxRollPitchRateRadS;
    commands.acroRateDesired_rad_s[2] :=
      normalizedPitch * maxRollPitchRateRadS;
    commands.acroRateDesired_rad_s[3] := -normalizedYaw * maxYawRateRadS;

    commands.attitudeDesired_rad[1] := normalizedRoll * maxAutoLevelTiltRad;
    commands.attitudeDesired_rad[2] := normalizedPitch * maxAutoLevelTiltRad;
    commands.attitudeDesired_rad[3] := pilot.attitude_rad[3];
  end when;
end CommandMapping;
