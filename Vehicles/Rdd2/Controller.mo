within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block Controller
  "Composed RDD2 controller used by simulation and qualification"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.000625;
  parameter Real guidancePeriod(unit = "s") = 0.005;

  input Integer mode(min = 0, max = 3);
  input Boolean armed;
  Interfaces.PilotInput pilot;
  Avionics.NavigationEstimateInput navigation;
  Interfaces.TrajectoryReferenceInput reference;

  Interfaces.PilotCommands pilotCommands;
  Interfaces.MotorCommands motorCommands;
  output Real thrust_N;
  output Real angularVelocitySetpointFlu_rad_s[3];
  output Real angularVelocityCommandFlu_rad_s[3];
  Interfaces.TelemetrySource telemetry;

protected
  GuidanceController guidanceTask(samplePeriod = guidancePeriod);
  RateControlAllocator rateTask(samplePeriod = samplePeriod);

equation
  guidanceTask.mode = mode;
  guidanceTask.armed = armed;
  guidanceTask.pilot.stick = pilot.stick;
  guidanceTask.pilot.throttle = pilot.throttle;
  guidanceTask.navigation.positionWorldEnu_m = navigation.positionWorldEnu_m;
  guidanceTask.navigation.velocityWorldEnu_m_s = navigation.velocityWorldEnu_m_s;
  guidanceTask.navigation.quaternionWorldBody = navigation.quaternionWorldBody;
  guidanceTask.reference.positionWorld_m = reference.positionWorld_m;
  guidanceTask.reference.velocityWorld_m_s = reference.velocityWorld_m_s;
  guidanceTask.reference.accelerationWorld_m_s2 =
    reference.accelerationWorld_m_s2;
  guidanceTask.reference.yaw_rad = reference.yaw_rad;
  pilotCommands.throttleInput =
    guidanceTask.pilotCommands.throttleInput;
  pilotCommands.acroRateDesired_rad_s =
    guidanceTask.pilotCommands.acroRateDesired_rad_s;
  pilotCommands.attitudeTiltDesired_rad =
    guidanceTask.pilotCommands.attitudeTiltDesired_rad;

  rateTask.inputSignal.armed = armed;
  rateTask.inputSignal.thrust_N = guidanceTask.rateCommand.thrust_N;
  rateTask.inputSignal.angularVelocityCommandFlu_rad_s =
    guidanceTask.rateCommand.angularVelocityCommandFlu_rad_s;
  rateTask.inputSignal.angularVelocityMeasuredFlu_rad_s =
    navigation.angularVelocityBodyFlu_rad_s;
  motorCommands.motor = rateTask.commands.motor;

  thrust_N = guidanceTask.rateCommand.thrust_N;
  angularVelocitySetpointFlu_rad_s =
    guidanceTask.angularVelocitySetpointFlu_rad_s;
  angularVelocityCommandFlu_rad_s =
    guidanceTask.rateCommand.angularVelocityCommandFlu_rad_s;
  telemetry.thrust_N = thrust_N;
  telemetry.angularVelocitySetpointFlu_rad_s =
    angularVelocitySetpointFlu_rad_s;
  telemetry.motor = motorCommands.motor;

  annotation(Documentation(info = "<html>
    <p>The simulation controller is intentionally only a composition: a
    guidance eFMU produces one rate-command message and a rate/allocation eFMU
    consumes it. Firmware deploys the same two blocks as separate RTOS tasks.</p>
  </html>"));
end Controller;
