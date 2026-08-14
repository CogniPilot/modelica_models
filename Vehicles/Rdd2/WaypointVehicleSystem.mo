within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

model WaypointVehicleSystem
  "Closed-loop plant and ideal RTOS composition driven by a waypoint plan"

  parameter Integer maxWaypoints(min = 2) = 8;
  parameter Integer waypointCount(min = 2, max = maxWaypoints) = 8;
  parameter Boolean useGlobalWaypoints = false;
  parameter Integer navigationSource(min = 0, max = 2) = 0
    "Guidance navigation: 0 truth bypass, 1 GPS-aided INS, 2 optical-flow-aided INS";
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
  parameter Real gravity_m_s2 = 9.80665;
  parameter Real gpsPositionCovarianceWorld_m2[3, 3] = identity(3) * 0.25;
  parameter Real gpsVelocityCovarianceWorld_m2_s2[3, 3] = identity(3) * 0.01;
  parameter Real opticalFlowVelocityCovarianceBody_m2_s2[2, 2] =
    identity(2) * 0.01;

  parameter Real segmentDuration[maxWaypoints - 1] =
    Planning.Bezier.waypointDurations(
      localRoute, nominalSpeed, minSegmentDuration);
  parameter Real trajectoryDuration(unit = "s") = sum(segmentDuration);
  parameter Real disarmTime_s = armTime_s + trajectoryDuration + disarmDelay_s;

  Vehicles.Rdd2.Plant plant;
  Vehicles.Rdd2.AvionicsSystem avionics(
    maxWaypoints = maxWaypoints,
    planningPeriod = planningPeriod,
    guidancePeriod = guidancePeriod,
    ratePeriod = ratePeriod);
  Vehicles.Rdd2.NavigationEstimator estimator(
    samplePeriod = guidancePeriod,
    gravityWorldEnu_m_s2 = {0.0, 0.0, -gravity_m_s2});

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
  output Real navigationError_m
    "Distance between the guidance navigation position and truth";

protected
  Real velocityBodyFlu_m_s[3]
    "Truth velocity resolved in body axes for the flow sensor model";

equation
  avionics.plan.valid = time >= armTime_s;
  avionics.plan.sequence = 0;
  avionics.plan.waypointCount = waypointCount;
  avionics.plan.globalFrame = useGlobalWaypoints;
  avionics.plan.originGeodetic = {
    origin.latitude_deg,
    origin.longitude_deg,
    origin.altitude_m};
  if useGlobalWaypoints then
    avionics.plan.waypoint = globalRoute;
  else
    avionics.plan.waypoint = localRoute;
  end if;
  avionics.plan.velocityEnu = waypointVelocityEnu;
  avionics.plan.yaw = waypointYaw;
  avionics.plan.nominalSpeed = nominalSpeed;
  avionics.plan.minSegmentDuration = minSegmentDuration;

  armed = avionics.reference.valid and time < disarmTime_s;
  avionics.armed = armed;
  avionics.mode = 2;
  avionics.pilot.stick = zeros(3);
  avionics.pilot.throttle = 0.0;
  connect(avionics.motorCommands, plant.commands);

  // Onboard sensing: the estimator always predicts on the plant IMU and is
  // aided by exactly one ideal (noise-free) sensor model built from truth.
  estimator.reset = false;
  connect(plant.imu, estimator.imu);

  estimator.mocap.valid = false;
  estimator.mocap.fresh = false;
  estimator.mocap.timestamp_s = time;
  estimator.mocap.positionWorldEnu_m = zeros(3);
  estimator.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  estimator.mocap.positionCovarianceWorld_m2 = identity(3);
  estimator.mocap.attitudeCovarianceBody_rad2 = identity(3);

  estimator.gps.valid = navigationSource == 1;
  estimator.gps.fresh = navigationSource == 1;
  estimator.gps.positionValid = navigationSource == 1;
  estimator.gps.velocityValid = navigationSource == 1;
  estimator.gps.timestamp_s = time;
  estimator.gps.geodetic_deg_m = geodetic;
  estimator.gps.positionWorldEnu_m = plant.truth.positionWorldEnu_m;
  estimator.gps.velocityWorldEnu_m_s = plant.truth.velocityWorldEnu_m_s;
  estimator.gps.positionCovarianceWorld_m2 = gpsPositionCovarianceWorld_m2;
  estimator.gps.velocityCovarianceWorld_m2_s2 =
    gpsVelocityCovarianceWorld_m2_s2;

  velocityBodyFlu_m_s = transpose(plant.truth.rotationWorldBody)
    * plant.truth.velocityWorldEnu_m_s;
  estimator.opticalFlow.valid = navigationSource == 2;
  estimator.opticalFlow.fresh = navigationSource == 2;
  estimator.opticalFlow.timestamp_s = time;
  estimator.opticalFlow.velocityBodyFlu_m_s = velocityBodyFlu_m_s[1:2];
  estimator.opticalFlow.velocityCovarianceBody_m2_s2 =
    opticalFlowVelocityCovarianceBody_m2_s2;
  estimator.opticalFlow.integratedLineOfSight_rad = zeros(2);
  estimator.opticalFlow.integrationTime_s = guidancePeriod;
  estimator.opticalFlow.groundDistance_m =
    max(plant.truth.positionWorldEnu_m[3], 0.0);
  estimator.opticalFlow.quality = 1.0;

  // Guidance flies on the selected navigation solution; truth bypass remains
  // available for isolating control-stack regressions from estimation.
  if navigationSource == 0 then
    avionics.navigation.valid = plant.truth.valid;
    avionics.navigation.timestamp_s = plant.truth.timestamp_s;
    avionics.navigation.positionWorldEnu_m = plant.truth.positionWorldEnu_m;
    avionics.navigation.velocityWorldEnu_m_s =
      plant.truth.velocityWorldEnu_m_s;
    avionics.navigation.accelerationWorldEnu_m_s2 =
      plant.truth.accelerationWorldEnu_m_s2;
    avionics.navigation.quaternionWorldBody =
      plant.truth.quaternionWorldBody;
    avionics.navigation.rotationWorldBody = plant.truth.rotationWorldBody;
    avionics.navigation.eulerRpy_rad = plant.truth.eulerRpy_rad;
    avionics.navigation.angularVelocityBodyFlu_rad_s =
      plant.truth.angularVelocityBodyFlu_rad_s;
    avionics.navigation.angularVelocityWorldEnu_rad_s =
      plant.truth.angularVelocityWorldEnu_rad_s;
  else
    avionics.navigation.valid = estimator.estimate.valid;
    avionics.navigation.timestamp_s = estimator.estimate.timestamp_s;
    avionics.navigation.positionWorldEnu_m =
      estimator.estimate.positionWorldEnu_m;
    avionics.navigation.velocityWorldEnu_m_s =
      estimator.estimate.velocityWorldEnu_m_s;
    avionics.navigation.accelerationWorldEnu_m_s2 =
      estimator.estimate.accelerationWorldEnu_m_s2;
    avionics.navigation.quaternionWorldBody =
      estimator.estimate.quaternionWorldBody;
    avionics.navigation.rotationWorldBody =
      estimator.estimate.rotationWorldBody;
    avionics.navigation.eulerRpy_rad = estimator.estimate.eulerRpy_rad;
    avionics.navigation.angularVelocityBodyFlu_rad_s =
      estimator.estimate.angularVelocityBodyFlu_rad_s;
    avionics.navigation.angularVelocityWorldEnu_rad_s =
      estimator.estimate.angularVelocityWorldEnu_rad_s;
  end if;

  time_s = time;
  position_m = plant.truth.positionWorldEnu_m;
  velocity_m_s = plant.truth.velocityWorldEnu_m_s;
  euler_rad = plant.truth.eulerRpy_rad;
  motorCommand = avionics.motorCommands.motor;
  thrust_N = avionics.thrust_N;
  geodetic = Geodesy.localEnuToGeodetic(
    origin, position_m[1], position_m[2], position_m[3]);
  navigationError_m = MathUtilities.norm3(
    avionics.navigation.positionWorldEnu_m
      - plant.truth.positionWorldEnu_m);
  if not armed then
    missionPhase = 0.0;
  elseif avionics.reference.complete then
    missionPhase = 2.0;
  else
    missionPhase = 1.0;
  end if;

  annotation(Documentation(info = "<html>
    <p>Reusable closed-loop integration of the RDD2 physical plant and
    <code>AvionicsSystem</code>. It supplies a fixed-capacity waypoint
    message and exposes flight observations; qualification models only need to
    select the route, the navigation source, and experiment limits.</p>
    <p><code>navigationSource</code> selects what guidance flies on: the truth
    bypass, a GPS-aided inertial solution, or an optical-flow-aided inertial
    solution. The aiding sensors are ideal (noise-free) maps of plant truth,
    so a passing mission qualifies the estimator-in-the-loop signal routing
    and observability rather than sensor-noise robustness.</p>
  </html>"));
end WaypointVehicleSystem;
