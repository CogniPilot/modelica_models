within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0
//
// Fixed-wing outer-loop autopilot for the HobbyZone Sport Cub S2.
// This fixed-period sampled model is the source for Rumoca eFMI Production Code.

block OuterLoop
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Real dt(unit="s") = 0.02
    "50 Hz outer loop (lockstep: 2 plant steps of 0.01 per packet)";
  parameter Components.VehicleParameters vehicle = Components.VehicleParameters();
  parameter Components.RouteParameters route = Components.RouteParameters();
  parameter Components.TecsParameters tecsParams = Components.TecsParameters();
  parameter Components.AttitudeParameters attitudeParams = Components.AttitudeParameters();
  parameter Real filterCutoffHz(unit="Hz") = 10.0;

  Components.StateEstimator estimator(dt=dt, filterCutoffHz=filterCutoffHz);
  replaceable Components.CrossTrackGuidance guidance(dt=dt, route=route)
    constrainedby Components.RouteGuidanceInterface
    "Replace with a Bry/Dubins polynomial strategy without changing routing"
    annotation(choicesAllMatching = true);
  Components.TECSController tecs(dt=dt, vehicle=vehicle, tecs=tecsParams);
  Components.AttitudeController attitude(dt=dt, vehicle=vehicle, params=attitudeParams);

  input Real position_m[3](each unit="m") "current sample [x, y, z] [m]";
  input Real euler_rad[3](each unit="rad") "current sample [roll, pitch, yaw] [rad]";
  input Real velocity_m_s[3](each unit="m/s") "current velocity sample [x, y, z] [m/s]";
  input Real eulerRate_rad_s[3](each unit="rad/s") "current body-rate sample [roll, pitch, yaw] [rad/s]";

  output Real command[4]
    "normalized {roll, pitch, yaw, throttle} inputs to the onboard stabilizer";
  Components.GuidanceSetpointsOutput setpoints;
  Components.TecsCommandsOutput tecsCommands;
  Components.FlightStateOutput estimate;
  output Real stabilizer "onboard stabilizer PWM [us]";
  output Integer currentWaypoint;
  output Real rollCommand;
  output Real rollCommandPreview;
  output Real pitchCommandPreview;
  output Real courseError;
equation
  estimator.position_m = position_m;
  estimator.euler_rad = euler_rad;
  estimator.velocity_m_s = velocity_m_s;
  estimator.eulerRate_rad_s = eulerRate_rad_s;

  connect(estimator.estimate, guidance.estimate);
  connect(estimator.estimate, tecs.estimate);
  connect(estimator.estimate, attitude.estimate);
  connect(guidance.setpoints, tecs.setpoints);
  connect(guidance.setpoints, attitude.setpoints);
  connect(tecs.commands, attitude.tecsCommands);
  connect(guidance.setpoints, setpoints);
  connect(tecs.commands, tecsCommands);
  connect(estimator.estimate, estimate);

  command = attitude.commands.normalized;
  stabilizer = attitudeParams.stabilizerCommand;
  currentWaypoint = guidance.currentWaypoint;
  rollCommand = attitude.rollCommand;
  rollCommandPreview = attitude.rollCommandPreview;
  pitchCommandPreview = tecs.commands.pitchPreview;
  courseError = attitude.courseError;
end OuterLoop;
