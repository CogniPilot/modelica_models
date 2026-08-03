within Vehicles.Rdd2.Qualification;
model WaypointMission
  "RDD2 takeoff, box, and landing on a differentially flat waypoint trajectory"

  // Mission mode. The local route is the source of truth; the global route is
  // the same box lifted to latitude/longitude/altitude through the mission
  // origin, so both modes fly an identical local trajectory and the geodetic
  // round-trip is exercised end to end.
  parameter Boolean useGlobalWaypoints = false
    "Interpret the route as global lat/lon/alt projected through the origin";
  parameter Geodesy.GeodeticOrigin origin = Geodesy.GeodeticOrigin(
    latitude_deg = 40.4237,
    longitude_deg = -86.9212,
    altitude_m = 180.0) "Fixed local-frame reference point";

  parameter Real cruiseAltitude_m = 2.0 "Box altitude above the origin";
  parameter Real boxSide_m = 4.0 "Box edge length";
  parameter Real localRoute[8, 3] = [
    0.0,       0.0,       0.0;
    0.0,       0.0,       cruiseAltitude_m;
    boxSide_m, 0.0,       cruiseAltitude_m;
    boxSide_m, boxSide_m, cruiseAltitude_m;
    0.0,       boxSide_m, cruiseAltitude_m;
    0.0,       0.0,       cruiseAltitude_m;
    0.0,       0.0,       0.3;
    0.0,       0.0,       0.1] "Waypoints [east, north, up] [m]";
  // In global mode the local box is lifted to lat/lon/alt through the origin
  // and projected back, so the followed route is identical to local mode and
  // the geodetic round-trip is exercised. The geodetic functions are only
  // evaluated when this branch is selected.
  parameter Real activeRoute[8, 3] = if useGlobalWaypoints then
      Geodesy.projectRouteToLocalEnu(
        origin, Geodesy.routeLocalEnuToGeodetic(origin, localRoute))
    else
      localRoute "Local route the trajectory passes through [m]";

  // Rest-to-rest trajectory: zero velocity, acceleration, and jerk at every
  // waypoint. Segment durations hold a nominal average speed.
  parameter Real nominalSpeed = 1.0 "Average speed along each segment [m/s]";
  parameter Real minSegmentDuration = 1.0 "Lower bound on a segment [s]";
  parameter Real velocities[8, 3] = zeros(8, 3)
    "Commanded velocity at each waypoint";
  parameter Real yawWaypoints[8] = zeros(8) "Heading at each waypoint [rad]";
  parameter Real durations[7] = Planning.Bezier.waypointDurations(
    activeRoute, nominalSpeed, minSegmentDuration) "Segment durations [s]";
  parameter Real trajectoryDuration = sum(durations) "Total flight time [s]";

  parameter Real armTime_s = 1.0;
  parameter Real disarmTime_s = armTime_s + trajectoryDuration + 3.0;

  // RDD2 physical constants (see Vehicles.Rdd2.Plant / QuadrotorPlant).
  parameter Real inertia[3] = {
    0.02166666666666667,
    0.02166666666666667,
    0.04000000000000001};
  parameter Real thrustCoefficient = 8.54858e-6 "Ct [N/(rad/s)^2]";
  parameter Real maxMotorSpeed = 1100.0 "AvionicsPlant omegaMax [rad/s]";
  parameter Real rateGain[3] = {20.0, 20.0, 10.0} "Body-rate bandwidth [1/s]";
  // wrenchToThrust = inverse([ones(1,4); RDD2 motor_moment_map]); the RDD2 map
  // is orthogonal, so the inverse is the scaled transpose. Columns act on
  // {thrust, Mx, My, Mz}. 1/(4d) = 1.4142135623730951 with d = 0.25/sqrt(2);
  // 1/(4*Cm) = 15.625 with Cm = 0.016.
  parameter Real wrenchToThrust[4, 4] = [
    0.25, -1.4142135623730951, -1.4142135623730951, -15.625;
    0.25, -1.4142135623730951,  1.4142135623730951,  15.625;
    0.25,  1.4142135623730951,  1.4142135623730951, -15.625;
    0.25,  1.4142135623730951, -1.4142135623730951,  15.625];

  Vehicles.Rdd2.AvionicsPlant plant;
  Vehicles.Rdd2.LogLinearController controller(samplePeriod = 0.005);

  Planning.Bezier.MultirotorTrajectory flatTrajectory;
  Real trajectoryTime_s;
  Boolean armed;
  Real angularVelocityBody[3];
  Real angularVelocityCommand[3];
  Real momentBody[3];
  Real motorCommand[4];
  Real headingQuaternionReference[4];
  Real geodetic[3];

  output Real time_s;
  output Real x_m;
  output Real y_m;
  output Real z_m;
  output Real vz_m_s;
  output Real roll_rad;
  output Real pitch_rad;
  output Real yaw_rad;
  output Real motor0;
  output Real motor1;
  output Real motor2;
  output Real motor3;
  output Real thrust_N;
  output Real target_x_m;
  output Real target_y_m;
  output Real target_z_m;
  output Real trajectory_time_s;
  output Real latitude_deg;
  output Real longitude_deg;
  output Real altitude_m;
  output Real mission_phase;

equation
  armed = time >= armTime_s and time < disarmTime_s;

  // Time-parameterized flat trajectory. Before arming the reference holds the
  // first waypoint; after the last segment it holds the final waypoint.
  trajectoryTime_s = time - armTime_s;
  flatTrajectory = Planning.Bezier.waypointTrajectory(
    activeRoute, velocities, yawWaypoints, durations, trajectoryTime_s);

  // Feedback in the local world frame (x East, y North, z Up), body frame FLU.
  headingQuaternionReference =
    LieGroups.SO3.EulerB321.to_Quat({flatTrajectory.yaw, 0.0, 0.0});

  controller.positionWorld = {plant.x_m, plant.y_m, plant.z_m};
  controller.velocityWorld = {plant.vx_m_s, plant.vy_m_s, plant.vz_m_s};
  controller.quaternionWorldBody = {plant.qw, plant.qx, plant.qy, plant.qz};
  controller.positionReferenceWorld = flatTrajectory.position;
  controller.velocityReferenceWorld = flatTrajectory.velocity;
  controller.accelerationReferenceWorld = flatTrajectory.acceleration;
  controller.headingQuaternionReference = headingQuaternionReference;
  controller.resetIntegral = not armed;

  // AvionicsPlant reports the gyro in FRD; the log-linear controller works in
  // the plant's FLU body frame, so map the rates back with diag(1, -1, -1).
  angularVelocityBody = {
    plant.gyro_x_rad_s,
    -plant.gyro_y_rad_s,
    -plant.gyro_z_rad_s};
  angularVelocityCommand =
    controller.angularVelocitySetpoint + controller.angularVelocityCorrection;
  momentBody = Control.Multirotor.RateLoop.bodyMoment(
    angularVelocityCommand,
    angularVelocityBody,
    inertia,
    rateGain);
  motorCommand = Control.Multirotor.Allocation.motorCommands(
    controller.thrust,
    momentBody,
    wrenchToThrust,
    thrustCoefficient,
    maxMotorSpeed);

  plant.motor0 = if armed then motorCommand[1] else 0.0;
  plant.motor1 = if armed then motorCommand[2] else 0.0;
  plant.motor2 = if armed then motorCommand[3] else 0.0;
  plant.motor3 = if armed then motorCommand[4] else 0.0;

  geodetic = Geodesy.localEnuToGeodetic(
    origin, plant.x_m, plant.y_m, plant.z_m);

  time_s = time;
  x_m = plant.x_m;
  y_m = plant.y_m;
  z_m = plant.z_m;
  vz_m_s = plant.vz_m_s;
  roll_rad = plant.roll_rad;
  pitch_rad = plant.pitch_rad;
  yaw_rad = plant.yaw_rad;
  motor0 = plant.motor0;
  motor1 = plant.motor1;
  motor2 = plant.motor2;
  motor3 = plant.motor3;
  thrust_N = controller.thrust;
  target_x_m = flatTrajectory.position[1];
  target_y_m = flatTrajectory.position[2];
  target_z_m = flatTrajectory.position[3];
  trajectory_time_s = trajectoryTime_s;
  latitude_deg = geodetic[1];
  longitude_deg = geodetic[2];
  altitude_m = geodetic[3];
  mission_phase = if not armed then 0.0
    elseif trajectoryTime_s < trajectoryDuration then 1.0
    else 2.0;

  annotation(
    experiment(StartTime = 0.0, StopTime = 45.0, Tolerance = 1.0e-8,
      Interval = 0.005),
    Documentation(info="<html>
      <p>Flies the RDD2 quadrotor through a takeoff, a box, and a landing on a
      differentially flat waypoint trajectory. A piecewise-septic
      <code>Planning.Bezier.waypointTrajectory</code> passes through each
      waypoint at rest and feeds its world position, velocity, and acceleration
      to the log-linear controller
      (<code>Vehicles.Rdd2.LogLinearController</code>); the collective thrust and
      body-rate command pass through the reusable body-rate loop and control
      allocation onto the four rotors.</p>
      <p>With <code>useGlobalWaypoints = false</code> the trajectory follows the
      local box directly. With <code>useGlobalWaypoints = true</code> the same
      box is lifted to latitude/longitude/altitude through <code>origin</code>
      and projected back, so the flown trajectory is identical and the geodetic
      round-trip is validated. All control runs in the local East-North-Up
      frame; the geodetic layer only converts the fixed mission route once.</p>
    </html>"));
end WaypointMission;
