within Planning.DubinsPolynomial;
function smoothOffsets
  "Construct a septic C3 transverse-offset seed, skipping zero-length segments"
  input Planning.Dubins.Path path;
  input Real startCurvature;
  input Real goalCurvature;
  input Real startCurvatureDerivative = 0.0;
  input Real goalCurvatureDerivative = 0.0;
  output Real offsetCoefficient[3, 8]
    "Power-basis coefficients in local physical nominal distance";
  output Boolean accepted
    "False only when the path has no length or a polynomial solve fails";
protected
  Real nominalKappa[3];
  Real segmentLength[3];
  Boolean active[3];
  Integer previousActive[3];
  Integer nextActive[3];
  Real coefficient[8];
  Boolean segmentAccepted;
algorithm
  for segmentIndex in 1:3 loop
    nominalKappa[segmentIndex] :=
      Planning.DubinsPolynomial.nominalCurvature(
        path.pathType, segmentIndex, path.turnRadius);
    segmentLength[segmentIndex] := path.turnRadius
      * path.normalizedSegmentLength[segmentIndex];
    active[segmentIndex] := segmentLength[segmentIndex] > 1.0e-10;
  end for;

  // Nearest active neighbour of each segment, written straight-line. The
  // segment loop below must stay unrolled: it contains a multi-output call,
  // and rumoca drops loop-carried updates in any function `for` loop that
  // does (same defect class as commit 8f14de1).
  previousActive := {
    0,
    if active[1] then 1 else 0,
    if active[2] then 2 elseif active[1] then 1 else 0};
  nextActive := {
    if active[2] then 2 elseif active[3] then 3 else 0,
    if active[3] then 3 else 0,
    0};

  offsetCoefficient := zeros(3, 8);
  accepted := path.length > 1.0e-10;
  coefficient := zeros(8);
  segmentAccepted := false;

  if active[1] then
    (coefficient, segmentAccepted) := Planning.DubinsPolynomial.segmentOffset(
      1, previousActive[1], nextActive[1], nominalKappa, segmentLength,
      startCurvature, goalCurvature,
      startCurvatureDerivative, goalCurvatureDerivative);
    offsetCoefficient[1, :] := coefficient;
    accepted := accepted and segmentAccepted;
  end if;

  if active[2] then
    (coefficient, segmentAccepted) := Planning.DubinsPolynomial.segmentOffset(
      2, previousActive[2], nextActive[2], nominalKappa, segmentLength,
      startCurvature, goalCurvature,
      startCurvatureDerivative, goalCurvatureDerivative);
    offsetCoefficient[2, :] := coefficient;
    accepted := accepted and segmentAccepted;
  end if;

  if active[3] then
    (coefficient, segmentAccepted) := Planning.DubinsPolynomial.segmentOffset(
      3, previousActive[3], nextActive[3], nominalKappa, segmentLength,
      startCurvature, goalCurvature,
      startCurvatureDerivative, goalCurvatureDerivative);
    offsetCoefficient[3, :] := coefficient;
    accepted := accepted and segmentAccepted;
  end if;
end smoothOffsets;
