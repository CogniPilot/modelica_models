within Planning.DubinsPolynomial;
function smoothOffsetCoefficients
  "Convenience wrapper returning the exact C3 septic seed"
  input Planning.Dubins.Path path;
  input Real startCurvature;
  input Real goalCurvature;
  input Real startCurvatureDerivative = 0.0;
  input Real goalCurvatureDerivative = 0.0;
  output Real offsetCoefficient[3, 8];
protected
  Boolean accepted;
algorithm
  (offsetCoefficient, accepted) := Planning.DubinsPolynomial.jointOptimizeOffsets(
    Planning.Dubins.Path(
      startPosition=path.startPosition,
      startHeading=path.startHeading,
      goalPosition=path.goalPosition,
      goalHeading=path.goalHeading,
      turnRadius=path.turnRadius,
      pathType=path.pathType,
      normalizedSegmentLength=path.normalizedSegmentLength,
      length=path.length,
      feasible=path.feasible),
    startCurvature,
    goalCurvature,
    startCurvatureDerivative,
    goalCurvatureDerivative);
  assert(accepted,
    "Dubins-polynomial smoothing requires a positive-length nominal path");
end smoothOffsetCoefficients;
