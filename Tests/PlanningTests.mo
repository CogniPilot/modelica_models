within Tests;
model PlanningTests "Dubins family, endpoint, optimality, invariance, and forward-motion tests"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 2.0e-9;
    constant Real pi = 2.0 * asin(1.0);
    Planning.Dubins.PathType types[6];
    Planning.Dubins.Candidate candidateResult;
    Planning.Dubins.Candidate candidateLsl;
    Planning.Dubins.Candidate candidateRsr;
    Planning.Dubins.Candidate candidateLsr;
    Planning.Dubins.Candidate candidateRsl;
    Planning.Dubins.Candidate candidateRlr;
    Planning.Dubins.Candidate candidateLrl;
    Planning.Dubins.Path candidatePath;
    Planning.Dubins.Path straightPath;
    Planning.Dubins.Path plannedPath;
    Planning.Dubins.Path transformedPath;
    Planning.Dubins.Path scaledPath;
    Planning.Dubins.Pose pose;
    Planning.Dubins.Pose previousPose;
    Planning.Dubins.Pose currentPose;
    Planning.DubinsPolynomial.State smoothState;
    Planning.DubinsPolynomial.State smoothStart;
    Planning.DubinsPolynomial.State smoothGoal;
    Real offsetCoefficient[3, 8];
    Real optimizedCoefficient[3, 8];
    Real hermiteCoefficient[6];
    Boolean offsetAccepted;
    Boolean optimizedAccepted;
    Boolean hermiteAccepted;
    Real continuityResidual;
    Real smoothingCost;
    Real seedCost;
    Real optimizedCost;
    Real rollAngle;
    Real rollRate;
    Real minimumLength;
    Real displacement[2];
    Real gamma;
    Real rotation[2, 2];
    Real translation[2];
    Real startPosition[2];
    Real goalPosition[2];
    Real startHeading;
    Real goalHeading;
    Real radius;
  algorithm
    types := {
      Planning.Dubins.PathType.LSL,
      Planning.Dubins.PathType.RSR,
      Planning.Dubins.PathType.LSR,
      Planning.Dubins.PathType.RSL,
      Planning.Dubins.PathType.RLR,
      Planning.Dubins.PathType.LRL};

    straightPath := Planning.Dubins.plan(
      {0.0, 0.0}, 0.0, {10.0, 0.0}, 0.0, 1.0);
    pose := Planning.Dubins.evaluate(straightPath, 1.0);
    assert(straightPath.feasible and abs(straightPath.length - 10.0) < tolerance,
      "Straight Dubins path had incorrect length");
    assert(Tests.Assertions.maxAbsVector(pose.position - {10.0, 0.0}) < tolerance and
        abs(Planning.Dubins.wrapAngle(pose.heading)) < tolerance,
      "Straight Dubins path did not reach its terminal pose");
    (offsetCoefficient, offsetAccepted) :=
      Planning.DubinsPolynomial.smoothOffsets(
        straightPath, 0.0, 0.0, 0.0, 0.0);
    assert(offsetAccepted and
        Planning.DubinsPolynomial.junctionContinuityResidual(
          straightPath, offsetCoefficient) < tolerance,
      "Dubins-polynomial smoothing rejected a path with zero-length turns");
    for sampleIndex in 0:10 loop
      smoothState := Planning.DubinsPolynomial.evaluate(
        straightPath, offsetCoefficient, sampleIndex / 10.0);
      assert(Tests.Assertions.maxAbsVector(
          smoothState.position - {sampleIndex, 0.0}) < tolerance and
          abs(smoothState.curvature) < tolerance and
          abs(smoothState.curvatureDerivative) < tolerance,
        "Smoothed straight Dubins path was not geometrically exact");
    end for;

    startPosition := {0.0, 0.0};
    startHeading := 0.0;
    goalPosition := {2.0, 1.0};
    goalHeading := 1.2;
    radius := 1.0;
    for familyIndex in 1:6 loop
      candidateResult := Planning.Dubins.candidate(
        startPosition, startHeading, goalPosition, goalHeading,
        radius, types[familyIndex]);
      assert(candidateResult.feasible,
        "Expected all six Dubins families to be feasible for the coverage case");
      candidatePath.startPosition := startPosition;
      candidatePath.startHeading := startHeading;
      candidatePath.goalPosition := goalPosition;
      candidatePath.goalHeading := goalHeading;
      candidatePath.turnRadius := radius;
      candidatePath.pathType := candidateResult.pathType;
      candidatePath.normalizedSegmentLength :=
        candidateResult.normalizedSegmentLength;
      candidatePath.length := candidateResult.length;
      candidatePath.feasible := candidateResult.feasible;
      pose := Planning.Dubins.evaluate(candidatePath, 1.0);
      assert(Tests.Assertions.maxAbsVector(pose.position - goalPosition) < 2.0e-8 and
          abs(Planning.Dubins.wrapAngle(pose.heading - goalHeading)) < 2.0e-8,
        "A Dubins family did not satisfy the terminal boundary conditions");
    end for;

    plannedPath := Planning.Dubins.plan(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7);
    // Six separate record locals rather than a Candidate[6] array: rumoca
    // rejects assigning a function result into a record-array element
    // (EX002), which would make this suite unrunnable on the toolchain that
    // generates flight code. Same reason as Planning.Dubins.plan itself.
    candidateLsl := Planning.Dubins.candidate(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7, types[1]);
    candidateRsr := Planning.Dubins.candidate(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7, types[2]);
    candidateLsr := Planning.Dubins.candidate(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7, types[3]);
    candidateRsl := Planning.Dubins.candidate(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7, types[4]);
    candidateRlr := Planning.Dubins.candidate(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7, types[5]);
    candidateLrl := Planning.Dubins.candidate(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7, types[6]);
    minimumLength := candidateLsl.length;
    if candidateRsr.feasible then
      minimumLength := min(minimumLength, candidateRsr.length);
    end if;
    if candidateLsr.feasible then
      minimumLength := min(minimumLength, candidateLsr.length);
    end if;
    if candidateRsl.feasible then
      minimumLength := min(minimumLength, candidateRsl.length);
    end if;
    if candidateRlr.feasible then
      minimumLength := min(minimumLength, candidateRlr.length);
    end if;
    if candidateLrl.feasible then
      minimumLength := min(minimumLength, candidateLrl.length);
    end if;
    assert(abs(plannedPath.length - minimumLength) < tolerance,
      "Dubins planner did not select the shortest feasible family");
    candidatePath := Planning.Dubins.plan(
      {0.0, 0.0}, -0.4, {10.0, 7.0}, 1.3, 1.7,
      allowThreeTurnPaths=false);
    assert(candidatePath.pathType <> Planning.Dubins.PathType.RLR and
        candidatePath.pathType <> Planning.Dubins.PathType.LRL,
      "Dubins planner selected a disabled three-turn family");
    pose := Planning.Dubins.evaluate(plannedPath, 1.0);
    assert(Tests.Assertions.maxAbsVector(
        pose.position - plannedPath.goalPosition) < 2.0e-8 and
        abs(Planning.Dubins.wrapAngle(
          pose.heading - plannedPath.goalHeading)) < 2.0e-8,
      "Selected Dubins path did not reach its goal");

    previousPose := Planning.Dubins.evaluate(plannedPath, 0.0);
    for sampleIndex in 1:100 loop
      currentPose := Planning.Dubins.evaluate(plannedPath, sampleIndex / 100.0);
      displacement := currentPose.position - previousPose.position;
      assert(displacement * {cos(previousPose.heading), sin(previousPose.heading)}
          >= -1.0e-10,
        "Dubins evaluator produced backward motion");
      assert(Tests.Assertions.maxAbsVector(displacement)
          <= 0.02 * plannedPath.length,
        "Dubins evaluator was not position-continuous");
      previousPose := currentPose;
    end for;

    gamma := 0.7;
    rotation := [cos(gamma), -sin(gamma); sin(gamma), cos(gamma)];
    translation := {3.0, -2.0};
    transformedPath := Planning.Dubins.plan(
      translation + rotation * plannedPath.startPosition,
      plannedPath.startHeading + gamma,
      translation + rotation * plannedPath.goalPosition,
      plannedPath.goalHeading + gamma,
      plannedPath.turnRadius);
    assert(abs(transformedPath.length - plannedPath.length) < 2.0e-8,
      "Dubins path length was not rigid-transform invariant");

    scaledPath := Planning.Dubins.plan(
      2.5 * plannedPath.startPosition,
      plannedPath.startHeading,
      2.5 * plannedPath.goalPosition,
      plannedPath.goalHeading,
      2.5 * plannedPath.turnRadius);
    assert(abs(scaledPath.length - 2.5 * plannedPath.length) < 2.0e-8,
      "Dubins path did not scale with position and turn radius");

    (hermiteCoefficient, hermiteAccepted) :=
      Polynomials.hermiteCoefficients(
        {1.0, -0.5, 0.25}, {2.0, 0.75, -0.1}, 1.7);
    assert(hermiteAccepted and
        abs(Polynomials.evaluateDerivative(hermiteCoefficient, 0.0, 0) - 1.0)
          < tolerance and
        abs(Polynomials.evaluateDerivative(hermiteCoefficient, 0.0, 1) + 0.5)
          < tolerance and
        abs(Polynomials.evaluateDerivative(hermiteCoefficient, 0.0, 2) - 0.25)
          < tolerance and
        abs(Polynomials.evaluateDerivative(hermiteCoefficient, 1.7, 0) - 2.0)
          < tolerance and
        abs(Polynomials.evaluateDerivative(hermiteCoefficient, 1.7, 1) - 0.75)
          < tolerance and
        abs(Polynomials.evaluateDerivative(hermiteCoefficient, 1.7, 2) + 0.1)
          < tolerance,
      "Dimension-generic Hermite polynomial construction failed");

    (offsetCoefficient, offsetAccepted) :=
      Planning.DubinsPolynomial.smoothOffsets(
        path=plannedPath,
        startCurvature=0.0,
        startCurvatureDerivative=0.0,
        goalCurvature=0.0,
        goalCurvatureDerivative=0.0);
    assert(offsetAccepted,
      "Coverage path contained a zero-length segment and could not be smoothed");
    continuityResidual :=
      Planning.DubinsPolynomial.junctionContinuityResidual(
        plannedPath, offsetCoefficient);
    assert(continuityResidual < 2.0e-7,
      "Dubins-polynomial seed was not C3 at a nominal segment junction");
    seedCost := 0.3 * Planning.DubinsPolynomial.derivativeCost(
      plannedPath, offsetCoefficient, 0)
      + Planning.DubinsPolynomial.derivativeCost(
        plannedPath, offsetCoefficient, 1)
      + 50.0 * Planning.DubinsPolynomial.derivativeCost(
        plannedPath, offsetCoefficient, 3);
    (optimizedCoefficient, optimizedAccepted) :=
      Planning.DubinsPolynomial.jointOptimizeOffsets(
        plannedPath, 0.0, 0.0, 0.0, 0.0);
    optimizedCost := 0.3 * Planning.DubinsPolynomial.derivativeCost(
      plannedPath, optimizedCoefficient, 0)
      + Planning.DubinsPolynomial.derivativeCost(
        plannedPath, optimizedCoefficient, 1)
      + 50.0 * Planning.DubinsPolynomial.derivativeCost(
        plannedPath, optimizedCoefficient, 3);
    assert(optimizedAccepted and optimizedCost <= seedCost * (1.0 + 1.0e-7),
      "Joint analytical Dubins-polynomial optimization increased its objective");
    assert(Planning.DubinsPolynomial.junctionContinuityResidual(
        plannedPath, optimizedCoefficient) < 2.0e-7,
      "True-derivative repair did not restore C3 junction continuity");
    offsetCoefficient := optimizedCoefficient;
    smoothStart := Planning.DubinsPolynomial.evaluate(
      plannedPath, offsetCoefficient, 0.0);
    smoothGoal := Planning.DubinsPolynomial.evaluate(
      plannedPath, offsetCoefficient, 1.0);
    assert(Tests.Assertions.maxAbsVector(
        smoothStart.position - plannedPath.startPosition) < tolerance and
        Tests.Assertions.maxAbsVector(
          smoothGoal.position - plannedPath.goalPosition) < 2.0e-8 and
        abs(Planning.Dubins.wrapAngle(
          smoothStart.heading - plannedPath.startHeading)) < tolerance and
        abs(Planning.Dubins.wrapAngle(
          smoothGoal.heading - plannedPath.goalHeading)) < 2.0e-8 and
        abs(smoothStart.curvature) < 2.0e-8 and
        abs(smoothGoal.curvature) < 2.0e-8,
      "Dubins-polynomial endpoint pose or curvature constraints failed");
    for sampleIndex in 0:100 loop
      smoothState := Planning.DubinsPolynomial.evaluate(
        plannedPath, offsetCoefficient, sampleIndex / 100.0);
      rollAngle := Planning.DubinsPolynomial.coordinatedRollAngle(
        smoothState.curvature, 7.0);
      rollRate := Planning.DubinsPolynomial.coordinatedRollRate(
        smoothState.curvature, smoothState.curvatureDerivative, 7.0);
      assert(smoothState.metricScale > 1.0e-6 and
          abs(smoothState.firstDerivative * smoothState.firstDerivative - 1.0)
            < 2.0e-9 and
          Tests.Assertions.isFiniteVector(smoothState.position) and
          Tests.Assertions.isFiniteVector(smoothState.thirdDerivative) and
          abs(rollAngle) < 0.5 * pi and abs(rollRate) < 1.0e100,
        "Dubins-polynomial evaluation was singular or non-finite");
    end for;
    smoothingCost := Planning.DubinsPolynomial.derivativeCost(
      plannedPath, offsetCoefficient, 3);
    assert(smoothingCost >= 0.0,
      "Integrated polynomial derivative cost was negative");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Dubins planning assertions did not complete");
end PlanningTests;
