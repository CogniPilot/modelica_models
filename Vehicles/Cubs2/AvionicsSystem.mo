within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0

block AvionicsSystem
  "CUBS2 outer-loop runtime composed with a simulation-only stabilizer"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Real outerLoopPeriod(unit = "s") = 0.02;
  parameter Real stabilizerPeriod(unit = "s") = 0.005
    "Assumed update period of the simulation surrogate";
  parameter Components.RouteParameters route = Components.RouteParameters();
  parameter Components.VehicleParameters vehicle = Components.VehicleParameters();
  parameter Components.TecsParameters tecs = Components.TecsParameters();
  parameter Components.AttitudeParameters attitude = Components.AttitudeParameters();

  input Real position_m[3];
  input Real euler_rad[3] "{roll, pitch, yaw}";
  input Real velocity_m_s[3];
  input Real angularVelocityBody_rad_s[3];
  input Real upBody[3];
  input Real airspeed_m_s;
  input Boolean engaged;
  input Boolean armed;
  input Boolean stickOverrideActive;
  input Real stickOverride[4] "manual {roll, pitch, yaw, throttle} command";

  discrete output Real actuatorCommand[4](
    start = {0.0, 0.0, 0.0, 0.0})
    "held {aileron, elevator, rudder, throttle} command";
  output Real stickCommand[4] "{roll, pitch, yaw, throttle}";
  output Real attitudeCommand_rad[2] "{roll, pitch}";
  Components.FlightStateOutput estimate;
  Components.GuidanceSetpointsOutput setpoints;
  Components.TecsCommandsOutput tecsCommands;
  output Integer currentWaypoint;
  output Real stabilizerCommand_us;
  output Real courseError_rad;
  output Real rollCommand_rad;

protected
  Vehicles.Cubs2.OuterLoop outerLoopTask(
    dt = outerLoopPeriod,
    route = route,
    vehicle = vehicle,
    tecsParams = tecs,
    attitudeParams = attitude);
  Vehicles.Cubs2.OnboardStabilizerSurrogate onboardStabilizer;

equation
  outerLoopTask.position_m = position_m;
  outerLoopTask.euler_rad = euler_rad;
  outerLoopTask.velocity_m_s = velocity_m_s;
  outerLoopTask.eulerRate_rad_s = angularVelocityBody_rad_s;
  if stickOverrideActive then
    stickCommand = stickOverride;
  elseif engaged then
    stickCommand = outerLoopTask.command;
  else
    stickCommand = zeros(4);
  end if;
  onboardStabilizer.pilotCommand = stickCommand;
  if armed then
    onboardStabilizer.armed = 1.0;
  else
    onboardStabilizer.armed = 0.0;
  end if;
  onboardStabilizer.gyro = angularVelocityBody_rad_s;
  onboardStabilizer.up_body = upBody;
  onboardStabilizer.airspeed = airspeed_m_s;

  connect(outerLoopTask.estimate, estimate);
  connect(outerLoopTask.setpoints, setpoints);
  connect(outerLoopTask.tecsCommands, tecsCommands);
  currentWaypoint = outerLoopTask.currentWaypoint;
  stabilizerCommand_us = outerLoopTask.stabilizer;
  courseError_rad = outerLoopTask.courseError;
  rollCommand_rad = outerLoopTask.rollCommand;
  attitudeCommand_rad = onboardStabilizer.attitudeCommand_rad;

algorithm
  when sample(0.0, stabilizerPeriod) then
    actuatorCommand := onboardStabilizer.surfaceCommand;
  end when;

  annotation(Documentation(info = "<html>
    <p>The navigation message feeds the deployable 50 Hz outer-loop eFMU. Its
    tensor pilot command crosses the real system boundary into the proprietary
    onboard stabilizer.</p>
    <p>For closed-loop simulation only, this model connects that boundary to
    <code>OnboardStabilizerSurrogate</code>. The RTOS holds its continuous
    surrogate response at <code>stabilizerPeriod</code>; that schedule and zero
    transport delay are test-fixture choices, not claims about the proprietary
    implementation.</p>
  </html>"));
end AvionicsSystem;
