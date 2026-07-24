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
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Bezier assertions did not complete");
end BezierTests;
