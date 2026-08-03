within Planning;
package Bezier
  "Bezier curves and differentially flat multirotor references"

  record MultirotorTrajectory
    "Flat outputs and the derivatives required by a multirotor"
    Real position[3](each unit = "m") "World-frame position";
    Real velocity[3](each unit = "m/s") "World-frame velocity";
    Real acceleration[3](each unit = "m/s2") "World-frame acceleration";
    Real jerk[3](each unit = "m/s3") "World-frame jerk";
    Real snap[3](each unit = "m/s4") "World-frame snap";
    Real yaw(unit = "rad") "Heading angle";
    Real yawRate(unit = "rad/s") "Heading rate";
    Real yawAcceleration(unit = "rad/s2") "Heading acceleration";
  end MultirotorTrajectory;

  record FlatReference
    "Multirotor state and wrench reconstructed from flat outputs"
    Real velocityBody[3](each unit = "m/s") "Body-frame velocity";
    Real quaternion[4](each unit = "1")
      "Body-to-world quaternion {w,x,y,z}";
    Real bodyToWorld[3, 3](each unit = "1") "Direction cosine matrix";
    Real angularVelocityBody[3](each unit = "rad/s")
      "Body-frame angular velocity";
    Real angularAccelerationBody[3](each unit = "rad/s2")
      "Body-frame angular acceleration";
    Real momentBody[3](each unit = "N.m") "Body-frame control moment";
    Real thrust(unit = "N") "Collective thrust magnitude";
  end FlatReference;

  function evaluate
    "Evaluate a dimension-generic Bezier curve using De Casteljau's algorithm"
    input Real controlPoint[:, :] "One control point per column";
    input Real duration(unit = "s") "Curve duration";
    input Real t(unit = "s") "Time relative to the segment start";
    output Real value[size(controlPoint, 1)] "Curve value at t";
  protected
    Real stage[size(controlPoint, 1), size(controlPoint, 2)];
    Real beta(unit = "1") "Normalized curve parameter";
  algorithm
    assert(duration > 0.0, "Bezier duration must be positive");
    assert(size(controlPoint, 2) >= 1,
      "A Bezier curve requires at least one control point");
    beta := t / duration;
    stage := controlPoint;
    for level in 1:size(controlPoint, 2) - 1 loop
      for pointIndex in 1:size(controlPoint, 2) - level loop
        stage[:, pointIndex] := (1.0 - beta) * stage[:, pointIndex]
          + beta * stage[:, pointIndex + 1];
      end for;
    end for;
    value := stage[:, 1];
    annotation(Documentation(info="<html>
      <p>Evaluates a Bezier curve without forming Bernstein polynomials.
      Rows are independent signal dimensions and columns are control points.</p>
      <p><code>t</code> is intentionally not clipped; values outside
      <code>[0, duration]</code> extrapolate the polynomial.</p>
    </html>"));
  end evaluate;

  function derivativeControlPoints
    "Construct control points for a physical-time derivative"
    input Real controlPoint[:, :] "One control point per column";
    input Real duration(unit = "s") "Curve duration";
    input Integer derivativeOrder(min = 0) = 1 "Derivative order";
    output Real derivativePoint[
      size(controlPoint, 1), size(controlPoint, 2) - derivativeOrder];
  protected
    Real stage[size(controlPoint, 1), size(controlPoint, 2)];
    Integer degree;
  algorithm
    assert(duration > 0.0, "Bezier duration must be positive");
    assert(derivativeOrder < size(controlPoint, 2),
      "Bezier derivative order must not exceed the curve degree");
    stage := controlPoint;
    degree := size(controlPoint, 2) - 1;
    for derivativeIndex in 1:derivativeOrder loop
      for pointIndex in 1:size(controlPoint, 2) - derivativeIndex loop
        stage[:, pointIndex] := (degree - derivativeIndex + 1)
          * (stage[:, pointIndex + 1] - stage[:, pointIndex]) / duration;
      end for;
    end for;
    derivativePoint := stage[:, 1:size(controlPoint, 2) - derivativeOrder];
    annotation(Documentation(info="<html>
      <p>Repeatedly applies the Bezier hodograph relation. Division by
      <code>duration</code> makes the result a derivative with respect to
      physical time rather than normalized curve parameter.</p>
    </html>"));
  end derivativeControlPoints;

  function evaluateDerivative
    "Evaluate a physical-time derivative of a Bezier curve"
    input Real controlPoint[:, :] "One control point per column";
    input Real duration(unit = "s") "Curve duration";
    input Real t(unit = "s") "Time relative to the segment start";
    input Integer derivativeOrder(min = 0) = 1 "Derivative order";
    output Real value[size(controlPoint, 1)] "Derivative value at t";
  protected
    Real derivativePoint[
      size(controlPoint, 1), size(controlPoint, 2) - derivativeOrder];
  algorithm
    derivativePoint := Planning.Bezier.derivativeControlPoints(
      controlPoint, duration, derivativeOrder);
    value := Planning.Bezier.evaluate(derivativePoint, duration, t);
    annotation(Documentation(info="<html>
      <p>Forms the derivative control polygon and evaluates it with the same
      De Casteljau implementation used for the original curve.</p>
    </html>"));
  end evaluateDerivative;

  function septicControlPoints
    "Construct a degree-seven curve from endpoint derivatives through jerk"
    input Real startDerivative[:, 4]
      "Columns are position, velocity, acceleration, and jerk at t=0";
    input Real endDerivative[size(startDerivative, 1), 4]
      "Columns are position, velocity, acceleration, and jerk at t=duration";
    input Real duration(unit = "s") "Curve duration";
    output Real controlPoint[size(startDerivative, 1), 8]
      "Degree-seven Bezier control points";
  algorithm
    assert(duration > 0.0, "Bezier duration must be positive");
    controlPoint[:, 1] := startDerivative[:, 1];
    controlPoint[:, 2] := controlPoint[:, 1]
      + duration * startDerivative[:, 2] / 7.0;
    controlPoint[:, 3] := duration^2 * startDerivative[:, 3] / 42.0
      + 2.0 * controlPoint[:, 2] - controlPoint[:, 1];
    controlPoint[:, 4] := duration^3 * startDerivative[:, 4] / 210.0
      + 3.0 * controlPoint[:, 3] - 3.0 * controlPoint[:, 2]
      + controlPoint[:, 1];
    controlPoint[:, 8] := endDerivative[:, 1];
    controlPoint[:, 7] := controlPoint[:, 8]
      - duration * endDerivative[:, 2] / 7.0;
    controlPoint[:, 6] := duration^2 * endDerivative[:, 3] / 42.0
      + 2.0 * controlPoint[:, 7] - controlPoint[:, 8];
    controlPoint[:, 5] := controlPoint[:, 8] - 3.0 * controlPoint[:, 7]
      + 3.0 * controlPoint[:, 6]
      - duration^3 * endDerivative[:, 4] / 210.0;
    annotation(Documentation(info="<html>
      <p>The eight endpoint constraints uniquely determine the eight control
      points. The resulting segment is suitable for position trajectories
      whose snap must also be evaluated.</p>
    </html>"));
  end septicControlPoints;

  function cubicControlPoints
    "Construct a degree-three curve from endpoint value and rate"
    input Real startDerivative[:, 2] "Value and rate at t=0";
    input Real endDerivative[size(startDerivative, 1), 2]
      "Value and rate at t=duration";
    input Real duration(unit = "s") "Curve duration";
    output Real controlPoint[size(startDerivative, 1), 4]
      "Degree-three Bezier control points";
  algorithm
    assert(duration > 0.0, "Bezier duration must be positive");
    controlPoint[:, 1] := startDerivative[:, 1];
    controlPoint[:, 2] := controlPoint[:, 1]
      + duration * startDerivative[:, 2] / 3.0;
    controlPoint[:, 4] := endDerivative[:, 1];
    controlPoint[:, 3] := controlPoint[:, 4]
      - duration * endDerivative[:, 2] / 3.0;
    annotation(Documentation(info="<html>
      <p>The four endpoint constraints uniquely determine a cubic curve.
      This package uses it for yaw because flatness reconstruction requires
      yaw through its second derivative.</p>
    </html>"));
  end cubicControlPoints;

  function evaluateMultirotor
    "Evaluate position through snap and yaw through angular acceleration"
    input Real positionControlPoint[3, 8]
      "Septic control points for world-frame position";
    input Real yawControlPoint[1, 4] "Cubic control points for yaw";
    input Real duration(unit = "s") "Segment duration";
    input Real t(unit = "s") "Time relative to the segment start";
    output Planning.Bezier.MultirotorTrajectory trajectory;
  protected
    Real yawValue[1];
    Real yawRateValue[1];
    Real yawAccelerationValue[1];
  algorithm
    trajectory.position := Planning.Bezier.evaluate(
      positionControlPoint, duration, t);
    trajectory.velocity := Planning.Bezier.evaluateDerivative(
      positionControlPoint, duration, t, 1);
    trajectory.acceleration := Planning.Bezier.evaluateDerivative(
      positionControlPoint, duration, t, 2);
    trajectory.jerk := Planning.Bezier.evaluateDerivative(
      positionControlPoint, duration, t, 3);
    trajectory.snap := Planning.Bezier.evaluateDerivative(
      positionControlPoint, duration, t, 4);
    yawValue := Planning.Bezier.evaluate(yawControlPoint, duration, t);
    yawRateValue := Planning.Bezier.evaluateDerivative(
      yawControlPoint, duration, t, 1);
    yawAccelerationValue := Planning.Bezier.evaluateDerivative(
      yawControlPoint, duration, t, 2);
    trajectory.yaw := yawValue[1];
    trajectory.yawRate := yawRateValue[1];
    trajectory.yawAcceleration := yawAccelerationValue[1];
    annotation(Documentation(info="<html>
      <p>Combines the translational and heading curves into the conventional
      flat-output derivative set for a quadrotor.</p>
    </html>"));
  end evaluateMultirotor;

  function flatReference
    "Recover multirotor attitude, rates, moment, and thrust from flat outputs"
    input Planning.Bezier.MultirotorTrajectory trajectory;
    input Real mass(unit = "kg") "Vehicle mass";
    input Real gravity(unit = "m/s2") "Positive gravitational acceleration";
    input Real inertia[3, 3](each unit = "kg.m2") "Body inertia matrix";
    input Real tolerance(min = 0.0) = 1.0e-6 "Singularity tolerance";
    output Planning.Bezier.FlatReference reference;
  protected
    constant Real worldX[3] = {1.0, 0.0, 0.0};
    constant Real worldY[3] = {0.0, 1.0, 0.0};
    constant Real worldZ[3] = {0.0, 0.0, 1.0};
    Real thrustAxis[3](each unit = "m/s2");
    Real thrustAxisNorm(unit = "m/s2");
    Real thrustAxisNormRate(unit = "m/s3");
    Real thrustAxisNormAcceleration(unit = "m/s4");
    Real bodyZ[3];
    Real bodyZRate[3](each unit = "1/s");
    Real bodyZAcceleration[3](each unit = "1/s2");
    Real headingX[3];
    Real headingXRate[3](each unit = "1/s");
    Real headingXAcceleration[3](each unit = "1/s2");
    Real headingBasis[3];
    Real headingBasisRate[3](each unit = "1/s");
    Real headingBasisAcceleration[3](each unit = "1/s2");
    Real bodyYRaw[3];
    Real bodyYRawRate[3](each unit = "1/s");
    Real bodyYRawAcceleration[3](each unit = "1/s2");
    Real bodyYRawNorm;
    Real bodyYRawNormRate(unit = "1/s");
    Real bodyYRawNormAcceleration(unit = "1/s2");
    Real bodyY[3];
    Real bodyYRate[3](each unit = "1/s");
    Real bodyYAcceleration[3](each unit = "1/s2");
    Real bodyX[3];
    Real bodyXRate[3](each unit = "1/s");
    Real bodyXAcceleration[3](each unit = "1/s2");
    Real rotationRate[3, 3](each unit = "1/s");
    Real rotationAcceleration[3, 3](each unit = "1/s2");
    Real angularVelocitySkew[3, 3](each unit = "1/s");
    Real angularAccelerationSkew[3, 3](each unit = "1/s2");
  algorithm
    assert(mass > 0.0, "Multirotor mass must be positive");
    assert(gravity > 0.0, "Gravity must be positive");
    thrustAxis := gravity * worldZ + trajectory.acceleration;
    thrustAxisNorm := sqrt(thrustAxis * thrustAxis);
    assert(thrustAxisNorm > tolerance,
      "Multirotor flat reference has a zero thrust direction");
    bodyZ := thrustAxis / thrustAxisNorm;
    thrustAxisNormRate := bodyZ * trajectory.jerk;
    bodyZRate := (trajectory.jerk - bodyZ * thrustAxisNormRate)
      / thrustAxisNorm;
    thrustAxisNormAcceleration := bodyZRate * trajectory.jerk
      + bodyZ * trajectory.snap;
    bodyZAcceleration := (trajectory.snap
      - bodyZ * thrustAxisNormAcceleration
      - 2.0 * bodyZRate * thrustAxisNormRate) / thrustAxisNorm;

    headingX := {cos(trajectory.yaw), sin(trajectory.yaw), 0.0};
    headingXRate := trajectory.yawRate
      * {-sin(trajectory.yaw), cos(trajectory.yaw), 0.0};
    headingXAcceleration := trajectory.yawAcceleration
        * {-sin(trajectory.yaw), cos(trajectory.yaw), 0.0}
      - trajectory.yawRate^2
        * {cos(trajectory.yaw), sin(trajectory.yaw), 0.0};
    bodyYRaw := cross(bodyZ, headingX);
    bodyYRawNorm := sqrt(bodyYRaw * bodyYRaw);
    if bodyYRawNorm > tolerance then
      headingBasis := headingX;
      headingBasisRate := headingXRate;
      headingBasisAcceleration := headingXAcceleration;
    else
      headingBasis := if abs(bodyZ[1]) <= abs(bodyZ[2])
          and abs(bodyZ[1]) <= abs(bodyZ[3]) then
          worldX
        else if abs(bodyZ[2]) <= abs(bodyZ[3]) then
          worldY
        else
          worldZ;
      headingBasisRate := zeros(3);
      headingBasisAcceleration := zeros(3);
    end if;
    bodyYRaw := cross(bodyZ, headingBasis);
    bodyYRawRate := cross(bodyZRate, headingBasis)
      + cross(bodyZ, headingBasisRate);
    bodyYRawAcceleration := cross(bodyZAcceleration, headingBasis)
      + 2.0 * cross(bodyZRate, headingBasisRate)
      + cross(bodyZ, headingBasisAcceleration);
    bodyYRawNorm := sqrt(bodyYRaw * bodyYRaw);
    assert(bodyYRawNorm > tolerance,
      "Unable to construct a body frame from thrust and heading");
    bodyY := bodyYRaw / bodyYRawNorm;
    bodyYRawNormRate := bodyY * bodyYRawRate;
    bodyYRate := (bodyYRawRate - bodyY * bodyYRawNormRate)
      / bodyYRawNorm;
    bodyYRawNormAcceleration := bodyYRate * bodyYRawRate
      + bodyY * bodyYRawAcceleration;
    bodyYAcceleration := (bodyYRawAcceleration
      - bodyY * bodyYRawNormAcceleration
      - 2.0 * bodyYRate * bodyYRawNormRate) / bodyYRawNorm;

    bodyX := cross(bodyY, bodyZ);
    bodyXRate := cross(bodyYRate, bodyZ) + cross(bodyY, bodyZRate);
    bodyXAcceleration := cross(bodyYAcceleration, bodyZ)
      + 2.0 * cross(bodyYRate, bodyZRate)
      + cross(bodyY, bodyZAcceleration);
    reference.bodyToWorld[:, 1] := bodyX;
    reference.bodyToWorld[:, 2] := bodyY;
    reference.bodyToWorld[:, 3] := bodyZ;
    rotationRate[:, 1] := bodyXRate;
    rotationRate[:, 2] := bodyYRate;
    rotationRate[:, 3] := bodyZRate;
    rotationAcceleration[:, 1] := bodyXAcceleration;
    rotationAcceleration[:, 2] := bodyYAcceleration;
    rotationAcceleration[:, 3] := bodyZAcceleration;

    angularVelocitySkew := transpose(reference.bodyToWorld) * rotationRate;
    reference.angularVelocityBody := {
      angularVelocitySkew[3, 2],
      angularVelocitySkew[1, 3],
      angularVelocitySkew[2, 1]};
    angularAccelerationSkew := transpose(reference.bodyToWorld)
      * (rotationAcceleration - rotationRate * angularVelocitySkew);
    reference.angularAccelerationBody := {
      angularAccelerationSkew[3, 2],
      angularAccelerationSkew[1, 3],
      angularAccelerationSkew[2, 1]};
    reference.velocityBody := transpose(reference.bodyToWorld)
      * trajectory.velocity;
    reference.quaternion := LieGroups.SO3.Quat.from_DCM(
      reference.bodyToWorld);
    reference.momentBody := inertia * reference.angularAccelerationBody
      + cross(reference.angularVelocityBody,
          inertia * reference.angularVelocityBody);
    reference.thrust := mass * thrustAxisNorm;
    annotation(Documentation(info="<html>
      <p>Uses differential flatness to reconstruct a body frame whose
      z-axis is aligned with required thrust and whose x-axis follows the yaw
      heading. Analytic derivatives provide body rate and angular acceleration;
      rigid-body dynamics then provide moment and collective thrust.</p>
      <p>The thrust direction is singular when gravity plus commanded
      acceleration is zero. Heading is singular when it is parallel to the
      thrust direction; the latter case uses the least-aligned Cartesian axis
      and differentiates the resulting orthonormal frame consistently.</p>
    </html>"));
  end flatReference;

  function waypointDurations
    "Per-segment durations that hold a nominal average speed between waypoints"
    input Real waypoints[:, 3] "World-frame waypoints, one [x, y, z] per row";
    input Real nominalSpeed(unit = "m/s") "Average speed along each segment";
    input Real minDuration(unit = "s") = 1.0 "Lower bound on any segment";
    output Real durations[size(waypoints, 1) - 1] "Segment durations [s]";
  algorithm
    assert(nominalSpeed > 0.0, "Nominal speed must be positive");
    assert(minDuration > 0.0, "Minimum duration must be positive");
    // Inline the chord so the loop's only target is the output array, which the
    // eFMI lowering proves totally defined element by element.
    for i in 1:(size(waypoints, 1) - 1) loop
      durations[i] := max(minDuration, sqrt(
        (waypoints[i + 1, 1] - waypoints[i, 1])^2
        + (waypoints[i + 1, 2] - waypoints[i, 2])^2
        + (waypoints[i + 1, 3] - waypoints[i, 3])^2) / nominalSpeed);
    end for;
    annotation(Documentation(info="<html>
      <p>Assigns each segment a duration equal to its chord length divided by a
      nominal average speed, floored at <code>minDuration</code>. For a
      rest-to-rest septic segment the peak speed is roughly 2.2 times this
      average. Use it to size a waypoint trajectory from a single cruise
      parameter.</p>
    </html>"));
  end waypointDurations;

  function waypointTrajectory
    "Differentially flat trajectory through waypoints with endpoint velocities"
    input Real waypoints[:, 3] "World-frame waypoints, one [x, y, z] per row";
    input Real velocities[size(waypoints, 1), 3]
      "World-frame velocity at each waypoint (zeros give rest-to-rest)";
    input Real yawWaypoints[size(waypoints, 1)](each unit = "rad")
      "Heading at each waypoint";
    input Real durations[size(waypoints, 1) - 1](each unit = "s")
      "Segment durations";
    input Real t(unit = "s") "Time since the trajectory start";
    output Planning.Bezier.MultirotorTrajectory trajectory;
  protected
    Integer n;
    Integer segment;
    Real totalDuration(unit = "s");
    Real segmentStart(unit = "s");
    Real elapsed(unit = "s");
    Real clampedTime(unit = "s");
    Real localTime(unit = "s");
    Real startDerivative[3, 4];
    Real endDerivative[3, 4];
    Real positionControlPoint[3, 8];
    Real yawControlPoint[1, 4];
  algorithm
    n := size(waypoints, 1);
    assert(n >= 2, "A waypoint trajectory needs at least two waypoints");

    totalDuration := 0.0;
    for i in 1:(n - 1) loop
      totalDuration := totalDuration + durations[i];
    end for;
    // Before the start hold the first waypoint; after the end hold the last.
    clampedTime := max(0.0, min(t, totalDuration));

    // Locate the active segment and its start time without dynamic model
    // indexing (this runs inside a function, so the loop index is procedural).
    segment := 1;
    segmentStart := 0.0;
    elapsed := 0.0;
    for i in 1:(n - 1) loop
      if clampedTime >= elapsed then
        segment := i;
        segmentStart := elapsed;
      end if;
      elapsed := elapsed + durations[i];
    end for;
    localTime := max(0.0, min(clampedTime - segmentStart, durations[segment]));

    // Each segment is a septic constrained by endpoint position and velocity
    // with zero acceleration and jerk at the waypoints. Assign the boundary
    // matrices whole so no partial-row write leaves a loop-carried target.
    startDerivative := [
      waypoints[segment, 1], velocities[segment, 1], 0.0, 0.0;
      waypoints[segment, 2], velocities[segment, 2], 0.0, 0.0;
      waypoints[segment, 3], velocities[segment, 3], 0.0, 0.0];
    endDerivative := [
      waypoints[segment + 1, 1], velocities[segment + 1, 1], 0.0, 0.0;
      waypoints[segment + 1, 2], velocities[segment + 1, 2], 0.0, 0.0;
      waypoints[segment + 1, 3], velocities[segment + 1, 3], 0.0, 0.0];
    positionControlPoint := Planning.Bezier.septicControlPoints(
      startDerivative, endDerivative, durations[segment]);
    yawControlPoint := Planning.Bezier.cubicControlPoints(
      [yawWaypoints[segment], 0.0],
      [yawWaypoints[segment + 1], 0.0],
      durations[segment]);
    trajectory := Planning.Bezier.evaluateMultirotor(
      positionControlPoint, yawControlPoint, durations[segment], localTime);
    annotation(Documentation(info="<html>
      <p>Builds a piecewise-septic differentially flat trajectory that passes
      through each waypoint at its commanded velocity (zero for rest-to-rest),
      with zero acceleration and jerk at the knots. Given the elapsed trajectory
      time it selects the active segment, constructs that segment's control
      points, and returns position through snap and yaw through angular
      acceleration. Time is clamped to the trajectory span, so the vehicle holds
      the first waypoint before the start and the last waypoint after the end.</p>
      <p>Connect <code>position</code>, <code>velocity</code>, and
      <code>acceleration</code> to a multirotor controller's world references and
      build a heading quaternion from <code>yaw</code>.</p>
    </html>"));
  end waypointTrajectory;

  annotation(Documentation(info="<html>
    <h4>Overview</h4>
    <p>This package provides dimension-generic Bezier evaluation, endpoint
    construction for cubic and septic segments, a multirotor
    differential-flatness reconstruction, and a piecewise-septic waypoint
    trajectory generator.</p>
    <h4>Conventions</h4>
    <ul>
      <li>Control points are matrix columns; rows are independent signals.</li>
      <li>Time derivatives are with respect to seconds, not normalized time.</li>
      <li>World z points upward and quaternions are scalar-first body-to-world.</li>
      <li>The septic position segment constrains position through jerk at both
      endpoints; the cubic yaw segment constrains yaw and yaw rate.</li>
    </ul>
    <h4>Example</h4>
    <pre>
positionControlPoint := Planning.Bezier.septicControlPoints(
  startDerivative, endDerivative, 7.0);
trajectory := Planning.Bezier.evaluateMultirotor(
  positionControlPoint, yawControlPoint, 7.0, segmentTime);
reference := Planning.Bezier.flatReference(
  trajectory, mass, gravity, inertia);
    </pre>
    <p>The implementation follows the algorithms in
    <a href='https://github.com/CogniPilot/cyecca/blob/main/cyecca/models/bezier.py'>
    cyecca/models/bezier.py</a>.</p>
  </html>"));
end Bezier;
