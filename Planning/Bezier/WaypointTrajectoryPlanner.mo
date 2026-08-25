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
  discrete Integer trajectoryTick(start = 0, fixed = true);
  discrete Real trajectoryTime(unit = "s", start = 0.0, fixed = true);
  discrete Real segmentStart[maxWaypoints - 1](
    each unit = "s", each start = 0.0, each fixed = true);
  discrete Real ladderPosition[3, 8](each start = 0.0, each fixed = true);
  discrete Real ladderVelocity[3, 7](each start = 0.0, each fixed = true);
  discrete Real ladderAcceleration[3, 6](each start = 0.0, each fixed = true);
  discrete Real ladderJerk[3, 5](each start = 0.0, each fixed = true);
  discrete Real ladderSnap[3, 4](each start = 0.0, each fixed = true);
  discrete Real ladderYaw[1, 4](each start = 0.0, each fixed = true);
  discrete Real ladderYawRate[1, 3](each start = 0.0, each fixed = true);
  discrete Real ladderYawAcceleration[1, 2](each start = 0.0, each fixed = true);
  discrete Real ladderDuration(unit = "s", start = 1.0, fixed = true);
  discrete Integer activeSegment(start = 1, fixed = true);
  Planning.Bezier.MultirotorTrajectory trajectory;
  Integer selectedSegment;
  Real localTime(unit = "s");
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
      trajectoryTick := 0;
      trajectoryTime := 0.0;
      (localWaypoint, velocityEnu, yaw, segmentDuration, segmentStart,
       totalDuration) :=
        Planning.Bezier.prepareWaypointPlan(
          plan.waypoint,
          plan.velocityEnu,
          plan.yaw,
          plan.waypointCount,
          plan.globalFrame,
          plan.originGeodetic,
          plan.nominalSpeed,
          plan.minSegmentDuration);
      // Force the ladder to be built for whichever segment comes first.
      activeSegment := 0;
    elseif pre(waypointCount) >= 2 then
      // TIME FROM A TICK COUNT, not an accumulator: the trajectory clock
      // must not depend on how often it is read, and a running sum in
      // single precision drifts by more over a mission than the sampling
      // error it would replace.
      trajectoryTick := pre(trajectoryTick) + 1;
      trajectoryTime := min(trajectoryTick * samplePeriod, pre(totalDuration));
    end if;

    reference.valid := waypointCount >= 2;
    reference.sequence := sequence;
    reference.trajectoryTime := trajectoryTime;
    reference.totalDuration := totalDuration;
    reference.complete := reference.valid and trajectoryTime >= totalDuration;
    if waypointCount >= 2 then
      (selectedSegment, localTime) :=
        Planning.Bezier.waypointSegmentPlacement(
          segmentDuration,
          segmentStart,
          waypointCount,
          trajectoryTime,
          totalDuration);
      // THE LADDER IS REBUILT ONLY WHEN THE SEGMENT CHANGES. Its control
      // points are a function of the segment alone, so rebuilding them per
      // sample would rediscover, every tick, a fact that changes a handful
      // of times in a mission.
      if selectedSegment <> pre(activeSegment) then
        (ladderPosition, ladderVelocity, ladderAcceleration, ladderJerk,
         ladderSnap, ladderYaw, ladderYawRate, ladderYawAcceleration,
         ladderDuration) :=
          Planning.Bezier.expandWaypointSegment(
            localWaypoint, velocityEnu, yaw, segmentDuration, selectedSegment);
      else
        ladderPosition := pre(ladderPosition);
        ladderVelocity := pre(ladderVelocity);
        ladderAcceleration := pre(ladderAcceleration);
        ladderJerk := pre(ladderJerk);
        ladderSnap := pre(ladderSnap);
        ladderYaw := pre(ladderYaw);
        ladderYawRate := pre(ladderYawRate);
        ladderYawAcceleration := pre(ladderYawAcceleration);
        ladderDuration := pre(ladderDuration);
      end if;
      activeSegment := selectedSegment;
      trajectory := Planning.Bezier.evaluateMultirotorSegment(
        ladderPosition, ladderVelocity, ladderAcceleration, ladderJerk,
        ladderSnap, ladderYaw, ladderYawRate, ladderYawAcceleration,
        ladderDuration, localTime);
      referencePosition := trajectory.position;
      referenceVelocity := trajectory.velocity;
      referenceAcceleration := trajectory.acceleration;
      referenceJerk := trajectory.jerk;
      referenceSnap := trajectory.snap;
      referenceYaw := trajectory.yaw;
      referenceYawRate := trajectory.yawRate;
      referenceYawAcceleration := trajectory.yawAcceleration;
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
