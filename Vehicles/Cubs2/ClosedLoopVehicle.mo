within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0

model ClosedLoopVehicle
  "Closed-loop CUBS2 plant using the onboard-stabilizer surrogate"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

  parameter Components.RouteParameters route = Components.RouteParameters();
  parameter Real initialPosition_m[3] = {0.0, 0.0, 0.1}
    "Body origin; 0.1 m places the main wheels on the ground plane";
  parameter Real initialVelocityBody_m_s[3] = {0.0, 0.0, 0.0};
  parameter Real initialQuaternion[4] = {1.0, 0.0, 0.0, 0.0};

  // This is a closed test composition, not an exported input boundary.
  // Ordinary bound variables permit scenarios to supply runtime mode logic
  // (for example landing/disarm) without treating it as an input default that
  // must be evaluated as a compile-time coordinate.
  Boolean engaged = false;
  Boolean armed = false;
  Boolean stickOverrideActive = false;
  Real stickOverride[4] = zeros(4);

  Vehicles.Cubs2.Plant plant(
    p_start = initialPosition_m,
    v_b_start = initialVelocityBody_m_s,
    q_start = initialQuaternion);
  Vehicles.Cubs2.AvionicsSystem avionics(route = route);

  output Real time_s;
  output Real position_m[3];
  output Real velocity_m_s[3];
  output Real euler_rad[3];
  output Real airspeed_m_s;
  output Boolean airborne
    "Qualification takeoff transition: clear of gear height and flying speed";
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
  avionics.position_m = plant.position;
  avionics.euler_rad = euler_rad;
  avionics.velocity_m_s = plant.velocity;
  avionics.angularVelocityBody_rad_s = plant.gyro;
  avionics.upBody = plant.up_body;
  avionics.airspeed_m_s = plant.airspeed;
  avionics.engaged = engaged;
  avionics.armed = armed;
  avionics.stickOverrideActive = stickOverrideActive;
  avionics.stickOverride = stickOverride;

  actuatorCommand = avionics.actuatorCommand;
  stickCommand = avionics.stickCommand;
  attitudeCommand_rad = avionics.attitudeCommand_rad;
  {plant.ail, plant.elev, plant.rud, plant.thr} = actuatorCommand;

  time_s = time;
  position_m = plant.position;
  velocity_m_s = plant.velocity;
  airspeed_m_s = plant.airspeed;
  airborne = position_m[3] > 0.25 and airspeed_m_s > 3.0;
  currentWaypoint = avionics.currentWaypoint;
  connect(avionics.setpoints, setpoints);
  connect(avionics.tecsCommands, tecsCommands);
  courseError_rad = avionics.courseError_rad;
  rollCommand_rad = avionics.rollCommand_rad;
end ClosedLoopVehicle;
