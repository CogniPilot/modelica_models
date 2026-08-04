within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0

block FlightControlSystem
  "Ideal RTOS composition of the CUBS2 outer and inner flight-control tasks"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Real outerLoopPeriod(unit = "s") = 0.02;
  parameter Real innerLoopPeriod(unit = "s") = 0.005;
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
  Vehicles.Cubs2.InnerLoop innerLoopTask(samplePeriod = innerLoopPeriod);

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
  innerLoopTask.stick = stickCommand;
  if armed then
    innerLoopTask.armed = 1.0;
  else
    innerLoopTask.armed = 0.0;
  end if;
  innerLoopTask.gyro = angularVelocityBody_rad_s;
  innerLoopTask.up_body = upBody;
  innerLoopTask.airspeed = airspeed_m_s;

  connect(outerLoopTask.estimate, estimate);
  connect(outerLoopTask.setpoints, setpoints);
  connect(outerLoopTask.tecsCommands, tecsCommands);
  currentWaypoint = outerLoopTask.currentWaypoint;
  stabilizerCommand_us = outerLoopTask.stabilizer;
  courseError_rad = outerLoopTask.courseError;
  rollCommand_rad = outerLoopTask.rollCommand;
  attitudeCommand_rad = innerLoopTask.attitudeCommand_rad;

algorithm
  when sample(0.0, innerLoopPeriod) then
    actuatorCommand := innerLoopTask.actuator;
  end when;

  annotation(Documentation(info = "<html>
    <p>This model is the ideal deterministic RTOS routing for the CUBS2 eFMU
    candidates. The navigation message feeds the 50 Hz outer-loop task. Its
    tensor command feeds the faster inner-loop task, whose actuator publication
    is sampled and held at <code>innerLoopPeriod</code>.</p>
    <p>Both releases start at phase zero and transport has zero delay. Hardware
    RTOS integration can be compared against this model without changing either
    control component.</p>
  </html>"));
end FlightControlSystem;
