within Tests;
model BezierTests "Bezier endpoint and multirotor flatness tests"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 2.0e-9;
    Real startDerivative[1, 4];
    Real endDerivative[1, 4];
    Real controlPoint[1, 8];
    Real positionStart[3, 4];
    Real positionEnd[3, 4];
    Real positionControlPoint[3, 8];
    Real yawControlPoint[1, 4];
    Real evaluatedStart[1];
    Real evaluatedEnd[1];
    Planning.Bezier.MultirotorTrajectory trajectory;
    Planning.Bezier.MultirotorTrajectory headingSingularityTrajectory;
    Planning.Bezier.FlatReference reference;
    Planning.Bezier.FlatReference headingSingularityReference;
    Real waypoints[3, 3];
    Real waypointVelocities[3, 3];
    Real waypointYaw[3];
    Real segmentDurations[2];
    Planning.Bezier.MultirotorTrajectory waypointStart;
    Planning.Bezier.MultirotorTrajectory waypointKnot;
    Planning.Bezier.MultirotorTrajectory waypointEnd;
    Planning.Bezier.MultirotorTrajectory waypointHold;
    Real ladderPosition[3, 8];
    Real ladderVelocity[3, 7];
    Real ladderAcceleration[3, 6];
    Real ladderJerk[3, 5];
    Real ladderSnap[3, 4];
    Real ladderYaw[1, 4];
    Real ladderYawRate[1, 3];
    Real ladderYawAcceleration[1, 2];
    Planning.Bezier.MultirotorTrajectory ladderTrajectory;
    Planning.Bezier.MultirotorTrajectory perCallTrajectory;
    Real ladderError;
  algorithm
    startDerivative := [1.0, -0.5, 0.25, -0.125];
    endDerivative := [2.0, 0.75, -0.1, 0.2];
    controlPoint := Planning.Bezier.septicControlPoints(
      startDerivative, endDerivative, 1.7);
    for derivativeOrder in 0:3 loop
      evaluatedStart := Planning.Bezier.evaluateDerivative(
        controlPoint, 1.7, 0.0, derivativeOrder);
      evaluatedEnd := Planning.Bezier.evaluateDerivative(
        controlPoint, 1.7, 1.7, derivativeOrder);
      assert(abs(evaluatedStart[1]
          - startDerivative[1, derivativeOrder + 1]) < tolerance,
        "Septic Bezier start derivative was not satisfied");
      assert(abs(evaluatedEnd[1]
          - endDerivative[1, derivativeOrder + 1]) < tolerance,
        "Septic Bezier end derivative was not satisfied");
    end for;

    positionStart := [
      0.0, 0.0, 0.0, 0.0;
      0.0, 0.0, 0.0, 0.0;
      2.0, 0.0, 0.0, 0.0];
    positionEnd := [
      4.0, 0.0, 0.0, 0.0;
      0.0, 0.0, 0.0, 0.0;
      2.0, 0.0, 0.0, 0.0];
    positionControlPoint := Planning.Bezier.septicControlPoints(
      positionStart, positionEnd, 7.0);
    yawControlPoint := Planning.Bezier.cubicControlPoints(
      [0.0, 0.0], [0.0, 0.0], 7.0);
    trajectory := Planning.Bezier.evaluateMultirotor(
      positionControlPoint, yawControlPoint, 7.0, 3.5);
    // THE LADDER MUST AGREE WITH THE PER-CALL DERIVATIVES IT REPLACES.
    // expandMultirotorSegment forms each derivative control polygon by one
    // hodograph pass over the rung above it; the path it replaced asked
    // derivativeControlPoints for orders one through four independently,
    // repeating every lower pass inside each higher one. The two are the
    // same relation applied the same number of times, so they must agree to
    // rounding, and this fails if a rung is ever built from the wrong
    // parent or scaled by the wrong degree.
    (ladderPosition, ladderVelocity, ladderAcceleration, ladderJerk,
     ladderSnap, ladderYaw, ladderYawRate, ladderYawAcceleration) :=
      Planning.Bezier.expandMultirotorSegment(
        positionControlPoint, yawControlPoint, 7.0);
    ladderError := 0.0;
    for sampleIndex in 0:14 loop
      ladderTrajectory := Planning.Bezier.evaluateMultirotorSegment(
        ladderPosition, ladderVelocity, ladderAcceleration, ladderJerk,
        ladderSnap, ladderYaw, ladderYawRate, ladderYawAcceleration,
        7.0, 0.5 * sampleIndex);
      perCallTrajectory := Planning.Bezier.evaluateMultirotor(
        positionControlPoint, yawControlPoint, 7.0, 0.5 * sampleIndex);
      ladderError := max(ladderError, max({
        Tests.Assertions.maxAbsVector(
          ladderTrajectory.position - perCallTrajectory.position),
        Tests.Assertions.maxAbsVector(
          ladderTrajectory.velocity - perCallTrajectory.velocity),
        Tests.Assertions.maxAbsVector(
          ladderTrajectory.acceleration - perCallTrajectory.acceleration),
        Tests.Assertions.maxAbsVector(
          ladderTrajectory.jerk - perCallTrajectory.jerk),
        Tests.Assertions.maxAbsVector(
          ladderTrajectory.snap - perCallTrajectory.snap),
        abs(ladderTrajectory.yaw - perCallTrajectory.yaw),
        abs(ladderTrajectory.yawRate - perCallTrajectory.yawRate),
        abs(ladderTrajectory.yawAcceleration
          - perCallTrajectory.yawAcceleration)}));
    end for;
    assert(ladderError < tolerance,
      "Expanded segment ladder disagreed with per-call derivative evaluation");

    reference := Planning.Bezier.flatReference(
      trajectory,
      2.0,
      9.80665,
      diagonal({0.02166666666666667, 0.02166666666666667, 0.04}));
    assert(abs(trajectory.position[1] - 2.0) < tolerance and
        trajectory.velocity[1] > 0.0 and
        Tests.Assertions.isFiniteVector(trajectory.snap),
      "Multirotor Bezier evaluation did not produce the expected midpoint");
    assert(abs(reference.quaternion * reference.quaternion - 1.0) < tolerance and
        Tests.Assertions.maxAbsMatrix(
          transpose(reference.bodyToWorld) * reference.bodyToWorld - identity(3))
          < tolerance and
        reference.thrust > 0.0 and
        Tests.Assertions.isFiniteVector(reference.momentBody),
      "Differential-flatness reconstruction was invalid");

    headingSingularityTrajectory := Planning.Bezier.MultirotorTrajectory(
      position=zeros(3),
      velocity=zeros(3),
      acceleration={1.0, 0.0, -9.80665},
      jerk=zeros(3),
      snap=zeros(3),
      yaw=0.0,
      yawRate=0.0,
      yawAcceleration=0.0);
    headingSingularityReference := Planning.Bezier.flatReference(
      headingSingularityTrajectory,
      2.0,
      9.80665,
      diagonal({0.02166666666666667, 0.02166666666666667, 0.04}));
    assert(Tests.Assertions.maxAbsMatrix(
        transpose(headingSingularityReference.bodyToWorld)
          * headingSingularityReference.bodyToWorld - identity(3))
          < tolerance and
        Tests.Assertions.isFiniteVector(
          headingSingularityReference.angularVelocityBody),
      "Flatness heading-singularity fallback was not orthonormal");
    // A rest-to-rest waypoint trajectory passes through every waypoint at rest,
    // holds the first waypoint before the start, and the last one after the end.
    waypoints := [
      0.0, 0.0, 0.0;
      2.0, 0.0, 1.0;
      2.0, 3.0, 1.0];
    waypointVelocities := zeros(3, 3);
    waypointYaw := {0.0, 0.0, 0.0};
    segmentDurations := Planning.Bezier.waypointDurations(waypoints, 1.0, 1.0);
    waypointStart := Planning.Bezier.waypointTrajectory(
      waypoints, waypointVelocities, waypointYaw, segmentDurations, 0.0);
    waypointKnot := Planning.Bezier.waypointTrajectory(
      waypoints, waypointVelocities, waypointYaw, segmentDurations,
      segmentDurations[1]);
    waypointEnd := Planning.Bezier.waypointTrajectory(
      waypoints, waypointVelocities, waypointYaw, segmentDurations,
      segmentDurations[1] + segmentDurations[2]);
    waypointHold := Planning.Bezier.waypointTrajectory(
      waypoints, waypointVelocities, waypointYaw, segmentDurations, -1.0);
    assert(Tests.Assertions.maxAbsVector(
        waypointStart.position - waypoints[1, :]) < 1.0e-9
      and Tests.Assertions.maxAbsVector(waypointStart.velocity) < 1.0e-9,
      "Waypoint trajectory did not start at the first waypoint at rest");
    assert(Tests.Assertions.maxAbsVector(
        waypointKnot.position - waypoints[2, :]) < 1.0e-9
      and Tests.Assertions.maxAbsVector(waypointKnot.velocity) < 1.0e-9,
      "Waypoint trajectory did not pass through the interior waypoint at rest");
    assert(Tests.Assertions.maxAbsVector(
        waypointEnd.position - waypoints[3, :]) < 1.0e-9
      and Tests.Assertions.maxAbsVector(waypointEnd.velocity) < 1.0e-9,
      "Waypoint trajectory did not end at the last waypoint at rest");
    assert(Tests.Assertions.maxAbsVector(
        waypointHold.position - waypoints[1, :]) < 1.0e-9,
      "Waypoint trajectory did not hold the first waypoint before the start");

    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Bezier assertions did not complete");
end BezierTests;
