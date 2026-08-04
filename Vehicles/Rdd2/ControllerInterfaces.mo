within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

package ControllerInterfaces
  "Typed signal boundaries shared by the RDD2 controller blocks"

connector RealOutput = output Real(start = 0.0, fixed = true);

connector FeedbackInput
  input Real setpoint;
  input Real measurement;
end FeedbackInput;

connector FeedbackOutput
  output Real error;
  output Real command;
end FeedbackOutput;

connector PilotInput
  input Real rcUs[5] "{roll, pitch, throttle, yaw, arm} receiver pulse widths";
  input Real attitude_rad[3] "{roll, pitch, yaw} attitude estimate";
  input Real throttleForCommand;
  input Boolean armed;
end PilotInput;

connector PilotCommands
  output Boolean armSwitchHigh;
  output Real throttleInput;
  output Real throttleCommand;
  output Real acroRateDesired_rad_s[3] "[roll, pitch, yaw]";
  output Real attitudeDesired_rad[3] "[roll, pitch, yaw]";
end PilotCommands;

connector MixerInput
  input Real throttle;
  input Real rateCorrection[3] "{roll, pitch, yaw} normalized corrections";
end MixerInput;

connector MotorCommands
  output Real motor[4] "normalized motor commands";
end MotorCommands;

connector NavigationEstimateInput
  "Navigation and inertial estimate consumed by the flight-control tasks"
  input Real positionWorld_m[3];
  input Real velocityWorld_m_s[3];
  input Real quaternionWorldBody[4] "body-to-world quaternion {w,x,y,z}";
  input Real angularVelocityBodyFrd_rad_s[3]
    "measured body rate in firmware Forward-Right-Down axes";
end NavigationEstimateInput;

connector RateControlInput
  "Outer-loop command and measured rate consumed by the rate task"
  input Boolean armed;
  input Real thrust_N;
  input Real angularVelocitySetpointFlu_rad_s[3];
  input Real angularVelocityCorrectionFlu_rad_s[3];
  input Real angularVelocityMeasuredFrd_rad_s[3];
end RateControlInput;

connector TelemetrySource
  "Causal telemetry bundle connected to an optional expandable bus"
  RealOutput pidError;
  RealOutput pidCommand;
  RealOutput throttleCommand;
  RealOutput motor[4];
end TelemetrySource;

expandable connector TelemetryBus
  "Optional, non-control-path extension point for qualification telemetry"
  Real pidError;
  Real pidCommand;
  Real throttleCommand;
  Real motor[4];
end TelemetryBus;

end ControllerInterfaces;
