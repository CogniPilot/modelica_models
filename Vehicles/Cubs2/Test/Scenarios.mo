within Vehicles.Cubs2.Test;

package Scenarios "Executable CUBS2 test missions"
  import Components = Vehicles.Cubs2.OuterLoopComponents;

model TakeoffOpenLoop
  Vehicles.Cubs2.Plant plant;
  Vehicles.Cubs2.OnboardStabilizerSurrogate onboardStabilizer;

  output Real time_s;
  output Real position_m[3];
  output Real euler_rad[3];
  output Real airspeed_m_s;
  output Real stickCommand[4];
  output Real actuatorCommand[4];

equation
  if time < 0.8 then
    stickCommand = {0.0, 0.0, 0.0, 1.0};
  else
    stickCommand = {0.0, 0.55, 0.0, 1.0};
  end if;
  onboardStabilizer.armed = 1.0;
  onboardStabilizer.pilotCommand = stickCommand;
  onboardStabilizer.gyro = plant.gyro;
  onboardStabilizer.up_body = plant.up_body;
  onboardStabilizer.airspeed = plant.airspeed;
  actuatorCommand = onboardStabilizer.surfaceCommand;
  {plant.ail, plant.elev, plant.rud, plant.thr} = actuatorCommand;
  time_s = time;
  position_m = plant.position;
  euler_rad = Vehicles.Interfaces.eulerFromQuaternion(plant.quat);
  airspeed_m_s = plant.airspeed;
end TakeoffOpenLoop;

model Takeoff
  parameter Components.RouteParameters takeoffRoute = Components.RouteParameters(
    cruiseSpeed = 4.5,
    waypoints = [
      0.0, 0.0, 0.0;
      30.0, 0.0, 3.0;
      60.0, 0.0, 3.0;
      90.0, 0.0, 3.0;
      120.0, 0.0, 3.0;
      150.0, 0.0, 3.0;
      180.0, 0.0, 3.0]);
  extends Vehicles.Cubs2.ClosedLoopVehicle(
    route = takeoffRoute,
    engaged = true,
    armed = true,
    stickOverrideActive = false,
    stickOverride = zeros(4));
end Takeoff;

model AltitudeHold
  parameter Real targetAltitude_m = 3.0;
  parameter Components.RouteParameters straightRoute = Components.RouteParameters(
    cruiseSpeed = 4.0,
    waypointSwitchingDistance = 3.0,
    waypoints = [
      0.0, 0.0, 3.0;
      30.0, 0.0, 3.0;
      60.0, 0.0, 3.0;
      90.0, 0.0, 3.0;
      120.0, 0.0, 3.0;
      150.0, 0.0, 3.0;
      180.0, 0.0, 3.0]);
  extends Vehicles.Cubs2.ClosedLoopVehicle(
    route = straightRoute,
    initialPosition_m = {0.0, 0.0, targetAltitude_m},
    initialVelocityBody_m_s = {4.0, 0.0, 0.0},
    engaged = true,
    armed = true,
    stickOverrideActive = false,
    stickOverride = zeros(4));
  output Real altitudeError_m;
equation
  altitudeError_m = targetAltitude_m - position_m[3];
end AltitudeHold;

model HeadingHold
  parameter Real targetAltitude_m = 3.0;
  parameter Components.RouteParameters straightRoute = Components.RouteParameters(
    cruiseSpeed = 4.0,
    waypointSwitchingDistance = 3.0,
    waypoints = [
      0.0, 0.0, targetAltitude_m;
      30.0, 0.0, targetAltitude_m;
      60.0, 0.0, targetAltitude_m;
      90.0, 0.0, targetAltitude_m;
      120.0, 0.0, targetAltitude_m;
      150.0, 0.0, targetAltitude_m;
      180.0, 0.0, targetAltitude_m]);
  extends Vehicles.Cubs2.ClosedLoopVehicle(
    route = straightRoute,
    initialPosition_m = {0.0, 0.0, targetAltitude_m},
    initialVelocityBody_m_s = {4.0, 0.0, 0.0},
    initialQuaternion = {0.9689124217106447, 0.0, 0.0, -0.24740395925452294},
    engaged = true,
    armed = true,
    stickOverrideActive = false,
    stickOverride = zeros(4));
  output Real headingError_rad;
equation
  headingError_rad = MathUtilities.wrapAngle(setpoints.heading - euler_rad[3]);
end HeadingHold;

model PatternMission
  parameter Components.RouteParameters patternRoute = Components.RouteParameters(
    cruiseSpeed = 4.0,
    waypointSwitchingDistance = 3.0,
    waypoints = [
      0.0, 0.0, 3.0;
      12.0, 0.0, 3.0;
      30.0, 0.0, 3.0;
      30.0, 20.0, 3.0;
      0.0, 20.0, 3.0;
      0.0, 0.0, 3.0;
      12.0, 0.0, 3.0]);
  extends Vehicles.Cubs2.ClosedLoopVehicle(
    route = patternRoute,
    initialPosition_m = {0.0, 0.0, 3.0},
    initialVelocityBody_m_s = {4.0, 0.0, 0.0},
    engaged = true,
    armed = not (landing and position_m[3] < 0.4),
    stickOverrideActive = landing,
    stickOverride = {0.0, -0.25, 0.0, 0.12});
  discrete output Integer lapCount(start = 0, fixed = true);
  discrete output Boolean landing(start = false, fixed = true);
  output Integer missionPhase;
protected
  discrete Integer previousWaypoint(start = 1, fixed = true);
algorithm
  when sample(0.0, 0.02) then
    if not pre(landing) and position_m[3] > 2.0
        and pre(previousWaypoint) == 6 and currentWaypoint == 1 then
      lapCount := pre(lapCount) + 1;
    else
      lapCount := pre(lapCount);
    end if;
    previousWaypoint := currentWaypoint;
    landing := pre(landing) or lapCount >= 2;
  end when;
equation
  if landing then
    missionPhase = 3;
  elseif position_m[3] > 0.4 then
    missionPhase = 2;
  else
    missionPhase = 1;
  end if;
end PatternMission;

model Mission
  extends Vehicles.Cubs2.ClosedLoopVehicle(
    initialQuaternion =
      {0.38268343236508984, 0.0, 0.0, -0.9238795325112867},
    engaged = true,
    armed = true,
    stickOverrideActive = false,
    stickOverride = zeros(4));
  discrete output Integer lapCount(start = 0, fixed = true);
protected
  discrete Integer previousWaypoint(start = 1, fixed = true);
algorithm
  when sample(0.0, 0.02) then
    if pre(previousWaypoint) == route.nSegments and currentWaypoint == 1 then
      lapCount := pre(lapCount) + 1;
    else
      lapCount := pre(lapCount);
    end if;
    previousWaypoint := currentWaypoint;
  end when;
end Mission;

end Scenarios;
