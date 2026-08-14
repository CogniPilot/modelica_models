within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

package ControllerInterfaces
  "Typed signal boundaries shared by the RDD2 controller blocks"

connector RealOutput = output Real(start = 0.0, fixed = true);

connector PilotInput
  "Normalized ManualControlData values consumed by guidance"
  input Real stick[3] "Normalized {roll, pitch, yaw} command";
  input Real throttle "Normalized collective command";
end PilotInput;

connector PilotCommands
  output Real throttleInput;
  output Real acroRateDesired_rad_s[3] "[roll, pitch, yaw]";
  output Real attitudeTiltDesired_rad[2] "[roll, pitch]";
end PilotCommands;

record MotorCommand
  Real motor[4] "normalized motor commands";
end MotorCommand;

connector MotorCommands = output MotorCommand;
connector MotorCommandInput = input MotorCommand;

connector TrajectoryReferenceInput
  "Local trajectory sample consumed by position guidance"
  input Real positionWorld_m[3];
  input Real velocityWorld_m_s[3];
  input Real accelerationWorld_m_s2[3];
  input Real yaw_rad;
end TrajectoryReferenceInput;

connector GuidanceStateInput
  "Minimal navigation state consumed by the guidance task"
  input Real positionWorldEnu_m[3];
  input Real velocityWorldEnu_m_s[3];
  input Real quaternionWorldBody[4]
    "Scalar-first Hamilton quaternion, body FLU to world ENU";
end GuidanceStateInput;

connector RateControlInput
  "Outer-loop command and measured rate consumed by the rate task"
  input Boolean armed;
  input Real thrust_N;
  input Real angularVelocityCommandFlu_rad_s[3];
  input Real angularVelocityMeasuredFlu_rad_s[3];
end RateControlInput;

connector RateCommand
  "Body-rate and collective-thrust message produced by guidance"
  output Real thrust_N;
  output Real angularVelocityCommandFlu_rad_s[3];
end RateCommand;

connector RateCommandInput
  "Body-rate and collective-thrust message consumed by rate control"
  input Real thrust_N;
  input Real angularVelocityCommandFlu_rad_s[3];
end RateCommandInput;

connector TelemetrySource
  "Causal controller telemetry connected to an optional expandable bus"
  RealOutput thrust_N;
  RealOutput angularVelocitySetpointFlu_rad_s[3];
  RealOutput motor[4];
end TelemetrySource;

expandable connector TelemetryBus
  "Optional, non-control-path extension point for qualification telemetry"
  Real thrust_N;
  Real angularVelocitySetpointFlu_rad_s[3];
  Real motor[4];
end TelemetryBus;

end ControllerInterfaces;
