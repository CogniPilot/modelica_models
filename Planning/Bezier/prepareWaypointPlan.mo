within Planning.Bezier;
function prepareWaypointPlan
  "Validate and convert the fixed-capacity payload of one waypoint message"
  input Real waypoint[:, 3]
    "Local ENU metres or global latitude/longitude/altitude rows";
  input Real velocityEnu[size(waypoint, 1), 3]
    "World-frame velocity at each waypoint [m/s]";
  input Real yaw[size(waypoint, 1)](each unit = "rad");
  input Integer waypointCount(min = 2, max = size(waypoint, 1));
  input Boolean globalFrame;
  input Real originGeodetic[3]
    "Local origin [latitude_deg, longitude_deg, altitude_m]";
  input Real nominalSpeed(unit = "m/s");
  input Real minSegmentDuration(unit = "s");
  output Real localWaypoint[size(waypoint, 1), 3]
    "Fixed-capacity local East-North-Up rows [m]";
  output Real acceptedVelocity[size(waypoint, 1), 3]
    "Fixed-capacity local velocity rows [m/s]";
  output Real acceptedYaw[size(waypoint, 1)](each unit = "rad");
  output Real segmentDuration[size(waypoint, 1) - 1](each unit = "s");
  output Real totalDuration(unit = "s");
algorithm
  assert(waypointCount >= 2 and waypointCount <= size(waypoint, 1),
    "Waypoint count must fit the fixed-capacity plan");
  assert(nominalSpeed > 0.0, "Nominal speed must be positive");
  assert(minSegmentDuration > 0.0, "Minimum segment duration must be positive");

  for waypointIndex in 1:size(waypoint, 1) loop
    if waypointIndex <= waypointCount then
      if globalFrame then
        localWaypoint[waypointIndex, :] := Geodesy.geodeticToLocalEnu(
          Geodesy.GeodeticOrigin(
            originGeodetic[1], originGeodetic[2], originGeodetic[3]),
          waypoint[waypointIndex, 1],
          waypoint[waypointIndex, 2],
          waypoint[waypointIndex, 3]);
      else
        localWaypoint[waypointIndex, :] := waypoint[waypointIndex, :];
      end if;
      acceptedVelocity[waypointIndex, :] := velocityEnu[waypointIndex, :];
      acceptedYaw[waypointIndex] := yaw[waypointIndex];
    else
      localWaypoint[waypointIndex, :] := zeros(3);
      acceptedVelocity[waypointIndex, :] := zeros(3);
      acceptedYaw[waypointIndex] := 0.0;
    end if;
  end for;

  for segmentIndex in 1:(size(waypoint, 1) - 1) loop
    if segmentIndex < waypointCount then
      segmentDuration[segmentIndex] := max(
        minSegmentDuration,
        sqrt(sum(
          (localWaypoint[segmentIndex + 1, :]
            - localWaypoint[segmentIndex, :]) .^ 2)) / nominalSpeed);
    else
      segmentDuration[segmentIndex] := 0.0;
    end if;
  end for;
  totalDuration := sum(segmentDuration);
end prepareWaypointPlan;
