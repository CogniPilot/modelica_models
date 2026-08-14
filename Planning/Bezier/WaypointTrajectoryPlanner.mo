within Planning.Bezier;
block WaypointTrajectoryPlanner
  "Accept local or global waypoint plans and emit a tracked Bezier reference"

  parameter Integer maxWaypoints(min = 2) = 16
    "Maximum waypoint rows accepted from the transport";
  parameter Real samplePeriod(unit = "s") = 0.02
    "Period at which new mission messages are accepted";

  Planning.Interfaces.WaypointPlanInput plan(capacity = maxWaypoints);
  Planning.Interfaces.TrajectoryReferenceOutput reference;

protected
  discrete Integer waypointCount(start = 0, fixed = true);
  discrete Integer sequence(start = -1, fixed = true);
  discrete Real localWaypoint[maxWaypoints, 3](each start = 0.0, each fixed = true);
  discrete Real velocityEnu[maxWaypoints, 3](each start = 0.0, each fixed = true);
  discrete Real yaw[maxWaypoints](each start = 0.0, each fixed = true);
  discrete Real segmentDuration[maxWaypoints - 1](
    each unit = "s", each start = 1.0, each fixed = true);
  discrete Real totalDuration(unit = "s", start = 0.0, fixed = true);
  discrete Real trajectoryTime(unit = "s", start = 0.0, fixed = true);
  discrete Integer activeSegment(start = 1, fixed = true);
  discrete Real referencePosition[3](each start = 0.0, each fixed = true);
  discrete Real referenceVelocity[3](each start = 0.0, each fixed = true);
  discrete Real referenceAcceleration[3](each start = 0.0, each fixed = true);
  discrete Real referenceJerk[3](each start = 0.0, each fixed = true);
  discrete Real referenceSnap[3](each start = 0.0, each fixed = true);
  discrete Real referenceYaw(start = 0.0, fixed = true);
  discrete Real referenceYawRate(start = 0.0, fixed = true);
  discrete Real referenceYawAcceleration(start = 0.0, fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    if plan.valid and plan.sequence <> pre(sequence)
        and plan.waypointCount >= 2 and plan.waypointCount <= maxWaypoints
        and plan.nominalSpeed > 0.0 and plan.minSegmentDuration > 0.0 then
      waypointCount := plan.waypointCount;
      sequence := plan.sequence;
      trajectoryTime := 0.0;
      (localWaypoint, velocityEnu, yaw, segmentDuration, totalDuration) :=
        Planning.Bezier.prepareWaypointPlan(
          plan.waypoint,
          plan.velocityEnu,
          plan.yaw,
          plan.waypointCount,
          plan.globalFrame,
          plan.originGeodetic,
          plan.nominalSpeed,
          plan.minSegmentDuration);
    elseif pre(waypointCount) >= 2 then
      trajectoryTime := min(
        pre(trajectoryTime) + samplePeriod,
        pre(totalDuration));
    end if;

    reference.valid := waypointCount >= 2;
    reference.sequence := sequence;
    reference.trajectoryTime := trajectoryTime;
    reference.totalDuration := totalDuration;
    reference.complete := reference.valid and trajectoryTime >= totalDuration;
    if waypointCount >= 2 then
      activeSegment := Planning.Bezier.activeWaypointSegment(
        segmentDuration,
        waypointCount,
        trajectoryTime);
      (referencePosition,
       referenceVelocity,
       referenceAcceleration,
       referenceJerk,
       referenceSnap,
       referenceYaw,
       referenceYawRate,
       referenceYawAcceleration) := Planning.Bezier.trackWaypointTrajectory(
        localWaypoint,
        velocityEnu,
        yaw,
        segmentDuration,
        waypointCount,
        trajectoryTime);
    else
      activeSegment := pre(activeSegment);
      referencePosition := pre(referencePosition);
      referenceVelocity := pre(referenceVelocity);
      referenceAcceleration := pre(referenceAcceleration);
      referenceJerk := pre(referenceJerk);
      referenceSnap := pre(referenceSnap);
      referenceYaw := pre(referenceYaw);
      referenceYawRate := pre(referenceYawRate);
      referenceYawAcceleration := pre(referenceYawAcceleration);
    end if;
    reference.activeSegment := activeSegment;
    reference.position := referencePosition;
    reference.velocity := referenceVelocity;
    reference.acceleration := referenceAcceleration;
    reference.jerk := referenceJerk;
    reference.snap := referenceSnap;
    reference.yaw := referenceYaw;
    reference.yawRate := referenceYawRate;
    reference.yawAcceleration := referenceYawAcceleration;
  end when;

  annotation(Documentation(info = "<html>
    <p>This block is the deployable mission-to-control boundary. A transport
    adapter writes one fixed-capacity waypoint message and increments its
    sequence number. The planner validates and latches the populated prefix,
    converts global geodetic rows into the local ENU frame once, and advances
    the corresponding septic-position and cubic-yaw trajectory on its fixed
    sample clock.</p>
    <p>It is independent of vehicle dynamics and feedback control so it can be
    exported as its own eFMU. A controller eFMU consumes the tensor reference
    connector.</p>
  </html>"));
end WaypointTrajectoryPlanner;
