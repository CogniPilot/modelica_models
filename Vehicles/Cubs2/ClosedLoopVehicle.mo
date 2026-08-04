within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0

model ClosedLoopVehicle
  "Closed-loop CUBS2 plant driven by the ideal RTOS flight-control system"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Components.RouteParameters route = Components.RouteParameters();
  parameter Real initialPosition_m[3] = {0.0, 0.0, 0.0};
  parameter Real initialVelocityBody_m_s[3] = {0.0, 0.0, 0.0};
  parameter Real initialQuaternion[4] = {1.0, 0.0, 0.0, 0.0};

  input Boolean engaged;
  input Boolean armed;
  input Boolean stickOverrideActive;
  input Real stickOverride[4];

  Vehicles.Cubs2.Plant plant(
    p_start = initialPosition_m,
    v_b_start = initialVelocityBody_m_s,
    q_start = initialQuaternion);
  Vehicles.Cubs2.FlightControlSystem flightControl(route = route);

  output Real time_s;
  output Real position_m[3];
  output Real velocity_m_s[3];
  output Real euler_rad[3];
  output Real airspeed_m_s;
  output Real actuatorCommand[4];
  output Real stickCommand[4];
  output Real attitudeCommand_rad[2];
  output Integer currentWaypoint;
  Components.GuidanceSetpointsOutput setpoints;
  Components.TecsCommandsOutput tecsCommands;
  output Real courseError_rad;
  output Real rollCommand_rad;

equation
  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(plant.quat);
  flightControl.position_m = plant.position;
  flightControl.euler_rad = euler_rad;
  flightControl.velocity_m_s = plant.velocity;
  flightControl.angularVelocityBody_rad_s = plant.gyro;
  flightControl.upBody = plant.up_body;
  flightControl.airspeed_m_s = plant.airspeed;
  flightControl.engaged = engaged;
  flightControl.armed = armed;
  flightControl.stickOverrideActive = stickOverrideActive;
  flightControl.stickOverride = stickOverride;

  actuatorCommand = flightControl.actuatorCommand;
  stickCommand = flightControl.stickCommand;
  attitudeCommand_rad = flightControl.attitudeCommand_rad;
  {plant.ail, plant.elev, plant.rud, plant.thr} = actuatorCommand;

  time_s = time;
  position_m = plant.position;
  velocity_m_s = plant.velocity;
  airspeed_m_s = plant.airspeed;
  currentWaypoint = flightControl.currentWaypoint;
  connect(flightControl.setpoints, setpoints);
  connect(flightControl.tecsCommands, tecsCommands);
  courseError_rad = flightControl.courseError_rad;
  rollCommand_rad = flightControl.rollCommand_rad;
end ClosedLoopVehicle;
