within Planning.Bezier;

function expandWaypointSegment
  "Expand one segment of a waypoint plan into its derivative ladder"
  input Real localWaypoint[:, 3](each unit = "m");
  input Real velocityEnu[size(localWaypoint, 1), 3](each unit = "m/s");
  input Real yaw[size(localWaypoint, 1)](each unit = "rad");
  input Real segmentDuration[size(localWaypoint, 1) - 1](each unit = "s");
  input Integer segment(min = 1);
  output Real position[3, 8] "Septic position control points";
  output Real velocity[3, 7] "First position derivative control points";
  output Real acceleration[3, 6] "Second";
  output Real jerk[3, 5] "Third";
  output Real snap[3, 4] "Fourth";
  output Real yawPoint[1, 4] "Cubic yaw control points";
  output Real yawRatePoint[1, 3] "First yaw derivative";
  output Real yawAccelerationPoint[1, 2] "Second yaw derivative";
  output Real duration(unit = "s") "This segment's duration";
protected
  Real startDerivative[3, 4];
  Real endDerivative[3, 4];
algorithm
  duration := segmentDuration[segment];
  startDerivative := [
    localWaypoint[segment, 1], velocityEnu[segment, 1], 0.0, 0.0;
    localWaypoint[segment, 2], velocityEnu[segment, 2], 0.0, 0.0;
    localWaypoint[segment, 3], velocityEnu[segment, 3], 0.0, 0.0];
  endDerivative := [
    localWaypoint[segment + 1, 1], velocityEnu[segment + 1, 1], 0.0, 0.0;
    localWaypoint[segment + 1, 2], velocityEnu[segment + 1, 2], 0.0, 0.0;
    localWaypoint[segment + 1, 3], velocityEnu[segment + 1, 3], 0.0, 0.0];
  (position, velocity, acceleration, jerk, snap, yawPoint, yawRatePoint,
   yawAccelerationPoint) := Planning.Bezier.expandMultirotorSegment(
    Planning.Bezier.septicControlPoints(
      startDerivative, endDerivative, segmentDuration[segment]),
    Planning.Bezier.cubicControlPoints(
      [yaw[segment], 0.0],
      [yaw[segment + 1], 0.0],
      segmentDuration[segment]),
    segmentDuration[segment]);
  annotation(Documentation(info = "<html>
    <p>Everything a segment change costs, and nothing a sample costs. Called
    when the active segment changes, not on every evaluation.</p>
  </html>"));
end expandWaypointSegment;
