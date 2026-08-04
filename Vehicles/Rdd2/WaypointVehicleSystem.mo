within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

model WaypointVehicleSystem
  "Closed-loop plant and ideal RTOS composition driven by a waypoint plan"

  parameter Integer maxWaypoints(min = 2) = 8;
  parameter Integer waypointCount(min = 2, max = maxWaypoints) = 8;
  parameter Boolean useGlobalWaypoints = false;
  parameter Geodesy.GeodeticOrigin origin = Geodesy.GeodeticOrigin(
    latitude_deg = 40.4237,
    longitude_deg = -86.9212,
    altitude_m = 180.0);
  parameter Real localRoute[maxWaypoints, 3];
  parameter Real globalRoute[maxWaypoints, 3] =
    Geodesy.routeLocalEnuToGeodetic(origin, localRoute);
  parameter Real waypointVelocityEnu[maxWaypoints, 3] = zeros(maxWaypoints, 3);
  parameter Real waypointYaw[maxWaypoints] = zeros(maxWaypoints);
  parameter Real nominalSpeed(unit = "m/s") = 1.0;
  parameter Real minSegmentDuration(unit = "s") = 1.0;
  parameter Real armTime_s = 1.0;
  parameter Real disarmDelay_s = 3.0;
  parameter Real planningPeriod(unit = "s") = 0.02;
  parameter Real guidancePeriod(unit = "s") = 0.005;
  parameter Real ratePeriod(unit = "s") = 0.001;

  parameter Real segmentDuration[maxWaypoints - 1] =
    Planning.Bezier.waypointDurations(
      localRoute, nominalSpeed, minSegmentDuration);
  parameter Real trajectoryDuration(unit = "s") = sum(segmentDuration);
  parameter Real disarmTime_s = armTime_s + trajectoryDuration + disarmDelay_s;

  Vehicles.Rdd2.PlantAdapter plant;
  Vehicles.Rdd2.FlightControlSystem flightControl(
    maxWaypoints = maxWaypoints,
    planningPeriod = planningPeriod,
    guidancePeriod = guidancePeriod,
    ratePeriod = ratePeriod);

  output Boolean armed;
  output Real time_s;
  output Real position_m[3];
  output Real velocity_m_s[3];
  output Real euler_rad[3];
  output Real motorCommand[4];
  output Real thrust_N;
  output Real geodetic[3]
    "{latitude_deg, longitude_deg, altitude_m}";
  output Real missionPhase;

equation
  flightControl.plan.valid = time >= armTime_s;
  flightControl.plan.sequence = 0;
  flightControl.plan.waypointCount = waypointCount;
  flightControl.plan.globalFrame = useGlobalWaypoints;
  flightControl.plan.originGeodetic = {
    origin.latitude_deg,
    origin.longitude_deg,
    origin.altitude_m};
  if useGlobalWaypoints then
    flightControl.plan.waypoint = globalRoute;
  else
    flightControl.plan.waypoint = localRoute;
  end if;
  flightControl.plan.velocityEnu = waypointVelocityEnu;
  flightControl.plan.yaw = waypointYaw;
  flightControl.plan.nominalSpeed = nominalSpeed;
  flightControl.plan.minSegmentDuration = minSegmentDuration;

  armed = flightControl.reference.valid and time < disarmTime_s;
  flightControl.armed = armed;
  flightControl.navigation.positionWorld_m = plant.position_m;
  flightControl.navigation.velocityWorld_m_s = plant.velocity_m_s;
  flightControl.navigation.quaternionWorldBody = plant.quaternion;
  flightControl.navigation.angularVelocityBodyFrd_rad_s = plant.gyro_rad_s;
  plant.motorCommand = flightControl.motorCommands.motor;

  time_s = time;
  position_m = plant.position_m;
  velocity_m_s = plant.velocity_m_s;
  euler_rad = plant.euler_rad;
  motorCommand = flightControl.motorCommands.motor;
  thrust_N = flightControl.thrust_N;
  geodetic = Geodesy.localEnuToGeodetic(
    origin, position_m[1], position_m[2], position_m[3]);
  if not armed then
    missionPhase = 0.0;
  elseif flightControl.reference.complete then
    missionPhase = 2.0;
  else
    missionPhase = 1.0;
  end if;

  annotation(Documentation(info = "<html>
    <p>Reusable closed-loop integration of the RDD2 physical plant and
    <code>FlightControlSystem</code>. It supplies a fixed-capacity waypoint
    message and exposes flight observations; qualification models only need to
    select the route and experiment limits.</p>
  </html>"));
end WaypointVehicleSystem;
