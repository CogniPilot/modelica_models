within Planning.DubinsPolynomial;
function junctionContinuityResidual
  "Maximum mismatch in position and true metric derivatives through third order"
  input Planning.Dubins.Path path;
  input Real offsetCoefficient[3, :];
  output Real residual;
protected
  Planning.DubinsPolynomial.State leftState;
  Planning.DubinsPolynomial.State rightState;
  Real segmentLength;
  Real previousSegmentLength;
  Integer previousSegment;
algorithm
  residual := 0.0;
  previousSegment := 0;
  for segmentIndex in 1:3 loop
    segmentLength := path.turnRadius
      * path.normalizedSegmentLength[segmentIndex];
    if segmentLength > 1.0e-10 then
      if previousSegment > 0 then
        previousSegmentLength := path.turnRadius
          * path.normalizedSegmentLength[previousSegment];
        leftState := Planning.DubinsPolynomial.evaluateSegment(
          path.startPosition, path.startHeading, path.turnRadius, path.pathType,
          path.normalizedSegmentLength, offsetCoefficient[previousSegment, :],
          previousSegment, previousSegmentLength);
        rightState := Planning.DubinsPolynomial.evaluateSegment(
          path.startPosition, path.startHeading, path.turnRadius, path.pathType,
          path.normalizedSegmentLength, offsetCoefficient[segmentIndex, :],
          segmentIndex, 0.0);
        residual := max(residual,
          max(abs(leftState.position - rightState.position)));
        residual := max(residual,
          max(abs(leftState.firstDerivative - rightState.firstDerivative)));
        residual := max(residual,
          max(abs(leftState.secondDerivative - rightState.secondDerivative)));
        residual := max(residual,
          max(abs(leftState.thirdDerivative - rightState.thirdDerivative)));
      end if;
      previousSegment := segmentIndex;
    end if;
  end for;
end junctionContinuityResidual;
