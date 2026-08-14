within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block CommandMapping "Map pilot input into rate, attitude, and thrust demands"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit="s") = 0.001;
  parameter Real maxRollPitchRateRadS(unit="rad/s") = 6.0;
  parameter Real maxYawRateRadS(unit="rad/s") = 3.5;
  parameter Real maxAutoLevelTiltRad(unit="rad") = 0.61086524;

  Interfaces.PilotInput pilot;
  Interfaces.PilotCommands commands;

protected
  discrete Real normalizedStick[3](each start = 0.0)
    "Normalized {roll, pitch, yaw} stick position";

algorithm
  when sample(0.0, samplePeriod) then
    normalizedStick := MathUtilities.clipVector(pilot.stick, -1.0, 1.0);
    commands.throttleInput := MathUtilities.clip(pilot.throttle, 0.0, 1.0);

    commands.acroRateDesired_rad_s :=
      {maxRollPitchRateRadS, maxRollPitchRateRadS, maxYawRateRadS}
      .* normalizedStick;
    commands.attitudeTiltDesired_rad :=
      maxAutoLevelTiltRad * normalizedStick[1:2];
  end when;
end CommandMapping;
