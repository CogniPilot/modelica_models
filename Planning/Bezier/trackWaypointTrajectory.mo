within Planning.Bezier;
function trackWaypointTrajectory
  "Evaluate the populated prefix of a fixed-capacity waypoint plan"
  input Real localWaypoint[:, 3](each unit = "m");
  input Real velocityEnu[size(localWaypoint, 1), 3](each unit = "m/s");
  input Real yaw[size(localWaypoint, 1)](each unit = "rad");
  input Real segmentDuration[size(localWaypoint, 1) - 1](each unit = "s");
  input Integer waypointCount(min = 2, max = size(localWaypoint, 1));
  input Real trajectoryTime(unit = "s");
  output Real position[3](each unit = "m");
  output Real velocity[3](each unit = "m/s");
  output Real acceleration[3](each unit = "m/s2");
  output Real jerk[3](each unit = "m/s3");
  output Real snap[3](each unit = "m/s4");
  output Real trajectoryYaw(unit = "rad");
  output Real yawRate(unit = "rad/s");
  output Real yawAcceleration(unit = "rad/s2");
protected
  Integer segment;
  Real totalDuration(unit = "s");
  Real segmentStart(unit = "s");
  Real localTime(unit = "s");
  Real startDerivative[3, 4];
  Real endDerivative[3, 4];
  Real positionControlPoint[3, 8];
  Real yawControlPoint[1, 4];
  Planning.Bezier.MultirotorTrajectory trajectory;
algorithm
  totalDuration := 0.0;
  for segmentIndex in 1:size(segmentDuration, 1) loop
    if segmentIndex < waypointCount then
      totalDuration := totalDuration + segmentDuration[segmentIndex];
    end if;
  end for;
  segment := Planning.Bezier.activeWaypointSegment(
    segmentDuration,
    waypointCount,
    min(max(trajectoryTime, 0.0), totalDuration));
  segmentStart := 0.0;
  for segmentIndex in 1:size(segmentDuration, 1) loop
    if segmentIndex < segment then
      segmentStart := segmentStart + segmentDuration[segmentIndex];
    end if;
  end for;
  localTime := max(0.0, min(
    trajectoryTime - segmentStart,
    segmentDuration[segment]));

  startDerivative := [
    localWaypoint[segment, 1], velocityEnu[segment, 1], 0.0, 0.0;
    localWaypoint[segment, 2], velocityEnu[segment, 2], 0.0, 0.0;
    localWaypoint[segment, 3], velocityEnu[segment, 3], 0.0, 0.0];
  endDerivative := [
    localWaypoint[segment + 1, 1], velocityEnu[segment + 1, 1], 0.0, 0.0;
    localWaypoint[segment + 1, 2], velocityEnu[segment + 1, 2], 0.0, 0.0;
    localWaypoint[segment + 1, 3], velocityEnu[segment + 1, 3], 0.0, 0.0];
  positionControlPoint := Planning.Bezier.septicControlPoints(
    startDerivative, endDerivative, segmentDuration[segment]);
  yawControlPoint := Planning.Bezier.cubicControlPoints(
    [yaw[segment], 0.0],
    [yaw[segment + 1], 0.0],
    segmentDuration[segment]);
  trajectory := Planning.Bezier.evaluateMultirotor(
    positionControlPoint,
    yawControlPoint,
    segmentDuration[segment],
    localTime);
  position := trajectory.position;
  velocity := trajectory.velocity;
  acceleration := trajectory.acceleration;
  jerk := trajectory.jerk;
  snap := trajectory.snap;
  trajectoryYaw := trajectory.yaw;
  yawRate := trajectory.yawRate;
  yawAcceleration := trajectory.yawAcceleration;
end trackWaypointTrajectory;
