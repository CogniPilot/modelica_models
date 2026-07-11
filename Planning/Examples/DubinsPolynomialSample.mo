within Planning.Examples;
model DubinsPolynomialSample
  "Compare a shortest Dubins path with its smooth transverse-polynomial seed"
  parameter Real startPosition[2] = {0.0, 0.0};
  parameter Real startHeading = -0.4;
  parameter Real goalPosition[2] = {10.0, 7.0};
  parameter Real goalHeading = 1.3;
  parameter Real turnRadius(min=0.0) = 1.7;
  parameter Boolean allowThreeTurnPaths = true
    "Allow RLR and LRL in the classical shortest-path search";
  parameter Real startCurvature = 0.0;
  parameter Real goalCurvature = 0.0;
  parameter Planning.Dubins.Path path = Planning.Dubins.plan(
    startPosition, startHeading, goalPosition, goalHeading, turnRadius,
    allowThreeTurnPaths);
  parameter Real offsetCoefficient[3, 8] =
    Planning.DubinsPolynomial.smoothOffsetCoefficients(
      path, startCurvature, goalCurvature);
  parameter Real continuityResidual =
    Planning.DubinsPolynomial.junctionContinuityResidual(
      path, offsetCoefficient);
  output Real nominalX;
  output Real nominalY;
  output Real nominalHeading;
  output Real smoothX;
  output Real smoothY;
  output Real smoothHeading;
  output Real curvature;
  output Real curvatureDerivative;
  output Real metricScale;
  output Real nominalPathLength;
  output Real nominalPathDistance;
  output Integer nominalSegmentIndex;
protected
  Planning.Dubins.Pose nominalPose;
  Planning.DubinsPolynomial.State smoothState;
equation
  nominalPose = Planning.Dubins.evaluate(path, time);
  smoothState = Planning.DubinsPolynomial.evaluate(
    path, offsetCoefficient, time);
  nominalX = nominalPose.position[1];
  nominalY = nominalPose.position[2];
  nominalHeading = nominalPose.heading;
  smoothX = smoothState.position[1];
  smoothY = smoothState.position[2];
  smoothHeading = smoothState.heading;
  curvature = smoothState.curvature;
  curvatureDerivative = smoothState.curvatureDerivative;
  metricScale = smoothState.metricScale;
  nominalPathLength = path.length;
  nominalPathDistance = time * path.length;
  nominalSegmentIndex = smoothState.segmentIndex;
  assert(continuityResidual < 2.0e-7,
    "Sample Dubins-polynomial path failed its C3 continuity certificate");
  assert(metricScale > 1.0e-6,
    "Sample Dubins-polynomial path became locally singular or reversed");
  assert(abs(smoothState.firstDerivative * smoothState.firstDerivative - 1.0)
      < 2.0e-8,
    "Sample Dubins-polynomial unit tangent lost normalization");
  assert(abs(curvature) < 1.0e6 and abs(curvatureDerivative) < 1.0e8,
    "Sample Dubins-polynomial differential geometry became non-finite");
  when initial() then
    assert(abs(smoothX - startPosition[1]) < 2.0e-8 and
        abs(smoothY - startPosition[2]) < 2.0e-8 and
        abs(Planning.Dubins.wrapAngle(smoothHeading - startHeading)) < 2.0e-8,
      "Dubins-polynomial gallery path failed its initial boundary condition");
  end when;
  when terminal() then
    assert(abs(smoothX - goalPosition[1]) < 2.0e-8 and
        abs(smoothY - goalPosition[2]) < 2.0e-8 and
        abs(Planning.Dubins.wrapAngle(smoothHeading - goalHeading)) < 2.0e-8,
      "Dubins-polynomial gallery path failed its terminal boundary condition");
  end when;
  annotation(experiment(StartTime=0.0, StopTime=1.0, Interval=0.0025));
end DubinsPolynomialSample;
