within Planning.DubinsPolynomial;
function jointOptimizeOffsets
  "Analytical joint quadratic optimization with one fixed true-derivative repair pass"
  input Planning.Dubins.Path path;
  input Real startCurvature;
  input Real goalCurvature;
  input Real startCurvatureDerivative = 0.0;
  input Real goalCurvatureDerivative = 0.0;
  input Real derivativeWeight[4] = {0.3, 1.0, 0.0, 50.0};
  output Real offsetCoefficient[3, 8];
  output Boolean accepted;
protected
  Real segmentLength[3];
  Real nominalKappa[3];
  Real coefficientMap[8, 8];
  Real costMatrix[8, 8];
  Real endpointCost[8, 8];
  Real globalHessian[16, 16];
  Real globalGradient[16];
  Real localOffset[8];
  Real globalDerivative[16];
  Real freeHessian[8, 8];
  Real freeRightHandSide[8, 1];
  Real freeSolution[8, 1];
  Real boundary[8];
  Real startDerivative[3, 4];
  Real endDerivative[3, 4];
  Real repairedDerivative[3];
  Real normal[2];
  Real averagedNormalDerivative[3];
  Real segmentDistance;
  Planning.Dubins.Pose junctionPose;
  Planning.DubinsPolynomial.State leftState;
  Planning.DubinsPolynomial.State rightState;
  Boolean mapAccepted;
  Boolean solveAccepted;
  Boolean segmentAccepted;
  Boolean inverseAccepted;
  Integer globalRow;
  Integer globalColumn;
algorithm
  accepted := path.feasible;
  for segmentIndex in 1:3 loop
    segmentLength[segmentIndex] := path.turnRadius
      * path.normalizedSegmentLength[segmentIndex];
    nominalKappa[segmentIndex] :=
      Planning.DubinsPolynomial.nominalCurvature(
        path.pathType, segmentIndex, path.turnRadius);
    accepted := accepted and segmentLength[segmentIndex] > 1.0e-8;
  end for;

  if not accepted then
    (offsetCoefficient, accepted) := Planning.DubinsPolynomial.smoothOffsets(
      path, startCurvature, goalCurvature,
      startCurvatureDerivative, goalCurvatureDerivative);
    return;
  end if;

  globalHessian := zeros(16, 16);
  globalGradient := zeros(16);
  for segmentIndex in 1:3 loop
    (coefficientMap, mapAccepted) := Polynomials.hermiteCoefficientMap(
      4, segmentLength[segmentIndex]);
    accepted := accepted and mapAccepted;
    costMatrix := zeros(8, 8);
    for derivativeOrder in 0:3 loop
      costMatrix := costMatrix + derivativeWeight[derivativeOrder + 1]
        * Polynomials.derivativeCostMatrix(
          8, derivativeOrder, segmentLength[segmentIndex]);
    end for;
    endpointCost := transpose(coefficientMap) * costMatrix * coefficientMap;
    localOffset := zeros(8);
    localOffset[3] := -nominalKappa[segmentIndex];
    localOffset[7] := -nominalKappa[segmentIndex];
    for localRow in 1:8 loop
      globalRow := if localRow <= 4 then
        4 * (segmentIndex - 1) + localRow else
        4 * segmentIndex + localRow - 4;
      for localColumn in 1:8 loop
        globalColumn := if localColumn <= 4 then
          4 * (segmentIndex - 1) + localColumn else
          4 * segmentIndex + localColumn - 4;
        globalHessian[globalRow, globalColumn] :=
          globalHessian[globalRow, globalColumn]
          + endpointCost[localRow, localColumn];
        globalGradient[globalRow] := globalGradient[globalRow]
          + endpointCost[localRow, localColumn] * localOffset[localColumn];
      end for;
    end for;
  end for;

  globalDerivative := zeros(16);
  globalDerivative[1:4] := {0.0, 0.0,
    startCurvature, startCurvatureDerivative};
  globalDerivative[13:16] := {0.0, 0.0,
    goalCurvature, goalCurvatureDerivative};
  freeHessian := globalHessian[5:12, 5:12];
  for freeIndex in 1:8 loop
    freeRightHandSide[freeIndex, 1] := -globalGradient[freeIndex + 4]
      - sum(globalHessian[freeIndex + 4, fixedIndex]
        * globalDerivative[fixedIndex] for fixedIndex in 1:4)
      - sum(globalHessian[freeIndex + 4, fixedIndex]
        * globalDerivative[fixedIndex] for fixedIndex in 13:16);
  end for;
  (freeSolution, solveAccepted) := LinearAlgebra.solve(
    freeHessian, freeRightHandSide, 1.0e-11);
  accepted := accepted and solveAccepted;
  if not accepted then
    (offsetCoefficient, accepted) := Planning.DubinsPolynomial.smoothOffsets(
      path, startCurvature, goalCurvature,
      startCurvatureDerivative, goalCurvatureDerivative);
    return;
  end if;
  globalDerivative[5:12] := freeSolution[:, 1];

  for segmentIndex in 1:3 loop
    boundary := cat(1,
      globalDerivative[4 * (segmentIndex - 1) + 1:
        4 * (segmentIndex - 1) + 4],
      globalDerivative[4 * segmentIndex + 1:4 * segmentIndex + 4]);
    boundary[3] := boundary[3] - nominalKappa[segmentIndex];
    boundary[7] := boundary[7] - nominalKappa[segmentIndex];
    (coefficientMap, mapAccepted) := Polynomials.hermiteCoefficientMap(
      4, segmentLength[segmentIndex]);
    offsetCoefficient[segmentIndex, :] := coefficientMap * boundary;
    accepted := accepted and mapAccepted;
  end for;

  for segmentIndex in 1:3 loop
    for derivativeIndex in 1:4 loop
      startDerivative[segmentIndex, derivativeIndex] :=
        Polynomials.evaluateDerivative(
          offsetCoefficient[segmentIndex, :], 0.0, derivativeIndex - 1);
      endDerivative[segmentIndex, derivativeIndex] :=
        Polynomials.evaluateDerivative(
          offsetCoefficient[segmentIndex, :], segmentLength[segmentIndex],
          derivativeIndex - 1);
    end for;
  end for;

  for junctionIndex in 1:2 loop
    segmentDistance := segmentLength[junctionIndex];
    leftState := Planning.DubinsPolynomial.evaluateSegment(
      path.startPosition, path.startHeading, path.turnRadius, path.pathType,
      path.normalizedSegmentLength, offsetCoefficient[junctionIndex, :],
      junctionIndex, segmentDistance);
    rightState := Planning.DubinsPolynomial.evaluateSegment(
      path.startPosition, path.startHeading, path.turnRadius, path.pathType,
      path.normalizedSegmentLength, offsetCoefficient[junctionIndex + 1, :],
      junctionIndex + 1, 0.0);
    junctionPose := Planning.DubinsPolynomial.segmentStartPose(
      path.startPosition, path.startHeading, path.turnRadius, path.pathType,
      path.normalizedSegmentLength, junctionIndex + 1);
    normal := {-sin(junctionPose.heading), cos(junctionPose.heading)};
    averagedNormalDerivative := {
      normal * (0.5 * (leftState.firstDerivative + rightState.firstDerivative)),
      normal * (0.5 * (leftState.secondDerivative + rightState.secondDerivative)),
      normal * (0.5 * (leftState.thirdDerivative + rightState.thirdDerivative))};
    endDerivative[junctionIndex, 1] := 0.5
      * (leftState.offset + rightState.offset);
    startDerivative[junctionIndex + 1, 1] :=
      endDerivative[junctionIndex, 1];
    (repairedDerivative, inverseAccepted) :=
      Planning.DubinsPolynomial.invertNormalDerivatives(
        endDerivative[junctionIndex, 1], nominalKappa[junctionIndex],
        averagedNormalDerivative);
    endDerivative[junctionIndex, 2:4] := repairedDerivative;
    accepted := accepted and inverseAccepted;
    (repairedDerivative, inverseAccepted) :=
      Planning.DubinsPolynomial.invertNormalDerivatives(
        startDerivative[junctionIndex + 1, 1],
        nominalKappa[junctionIndex + 1], averagedNormalDerivative);
    startDerivative[junctionIndex + 1, 2:4] := repairedDerivative;
    accepted := accepted and inverseAccepted;
  end for;

  if accepted then
    for segmentIndex in 1:3 loop
      (offsetCoefficient[segmentIndex, :], segmentAccepted) :=
        Polynomials.hermiteCoefficients(
          startDerivative[segmentIndex, :], endDerivative[segmentIndex, :],
          segmentLength[segmentIndex]);
      accepted := accepted and segmentAccepted;
    end for;
  else
    (offsetCoefficient, accepted) := Planning.DubinsPolynomial.smoothOffsets(
      path, startCurvature, goalCurvature,
      startCurvatureDerivative, goalCurvatureDerivative);
  end if;
end jointOptimizeOffsets;
