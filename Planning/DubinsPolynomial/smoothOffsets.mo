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
  Real desiredStartCurvature;
  Real desiredEndCurvature;
  Real desiredStartCurvatureDerivative;
  Real desiredEndCurvatureDerivative;
  Real startDerivative[4];
  Real endDerivative[4];
  Real coefficient[8];
  Boolean segmentAccepted;
  Integer previousActiveSegment;
  Integer nextActiveSegment;
algorithm
  for segmentIndex in 1:3 loop
    nominalKappa[segmentIndex] :=
      Planning.DubinsPolynomial.nominalCurvature(
        path.pathType, segmentIndex, path.turnRadius);
    segmentLength[segmentIndex] := path.turnRadius
      * path.normalizedSegmentLength[segmentIndex];
  end for;

  offsetCoefficient := zeros(3, 8);
  accepted := path.length > 1.0e-10;
  previousActiveSegment := 0;
  for segmentIndex in 1:3 loop
    if segmentLength[segmentIndex] > 1.0e-10 then
      nextActiveSegment := 0;
      for candidateIndex in segmentIndex + 1:3 loop
        if nextActiveSegment == 0
            and segmentLength[candidateIndex] > 1.0e-10 then
          nextActiveSegment := candidateIndex;
        end if;
      end for;

      desiredStartCurvature := if previousActiveSegment == 0 then
          startCurvature else 0.5 * (nominalKappa[previousActiveSegment]
            + nominalKappa[segmentIndex]);
      desiredEndCurvature := if nextActiveSegment == 0 then
          goalCurvature else 0.5 * (nominalKappa[segmentIndex]
            + nominalKappa[nextActiveSegment]);
      desiredStartCurvatureDerivative := if previousActiveSegment == 0 then
          startCurvatureDerivative else 0.0;
      desiredEndCurvatureDerivative := if nextActiveSegment == 0 then
          goalCurvatureDerivative else 0.0;
      startDerivative := {
        0.0,
        0.0,
        desiredStartCurvature - nominalKappa[segmentIndex],
        desiredStartCurvatureDerivative};
      endDerivative := {
        0.0,
        0.0,
        desiredEndCurvature - nominalKappa[segmentIndex],
        desiredEndCurvatureDerivative};
      (coefficient, segmentAccepted) := Polynomials.hermiteCoefficients(
        startDerivative, endDerivative, segmentLength[segmentIndex]);
      offsetCoefficient[segmentIndex, :] := coefficient;
      accepted := accepted and segmentAccepted;
      previousActiveSegment := segmentIndex;
    end if;
  end for;
end smoothOffsets;
