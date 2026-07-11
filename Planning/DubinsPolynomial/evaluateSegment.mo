within Planning.DubinsPolynomial;
function evaluateSegment
  "Evaluate one transverse polynomial using physical nominal segment distance"
  input Real startPosition[2];
  input Real startHeading;
  input Real turnRadius;
  input Planning.Dubins.PathType pathType;
  input Real normalizedSegmentLength[3];
  input Real offsetCoefficient[:] "Increasing powers of local nominal distance";
  input Integer segmentIndex(min=1, max=3);
  input Real localDistance;
  output Planning.DubinsPolynomial.State state;
protected
  Planning.Dubins.Pose segmentStart;
  Planning.Dubins.Pose nominalPose;
  Real segmentLength;
  Real distance;
  Real nominalKappa;
  Real offsetDerivative[4];
  Real tangent[2];
  Real normal[2];
  Real firstNominal[2];
  Real secondNominal[2];
  Real thirdNominal[2];
  Real tangentialFactor;
  Real firstTangential;
  Real secondNormal;
  Real thirdTangential;
  Real thirdNormal;
  Real scale;
  Real innerProduct;
  Real innerProductDerivative;
algorithm
  segmentLength := turnRadius * normalizedSegmentLength[segmentIndex];
  assert(localDistance >= -1.0e-12 and
      localDistance <= segmentLength + 1.0e-12,
    "Local Dubins-polynomial distance lies outside its segment");
  distance := min(max(localDistance, 0.0), segmentLength);
  segmentStart := Planning.DubinsPolynomial.segmentStartPose(
    startPosition, startHeading, turnRadius, pathType,
    normalizedSegmentLength, segmentIndex);
  nominalPose := Planning.Dubins.advance(
    segmentStart.position,
    segmentStart.heading,
    Planning.Dubins.segmentType(pathType, segmentIndex),
    distance / turnRadius,
    turnRadius);
  nominalKappa := Planning.DubinsPolynomial.nominalCurvature(
    pathType, segmentIndex, turnRadius);
  for derivativeOrder in 0:3 loop
    offsetDerivative[derivativeOrder + 1] :=
      Polynomials.evaluateDerivative(
        offsetCoefficient, distance, derivativeOrder);
  end for;

  tangent := {cos(nominalPose.heading), sin(nominalPose.heading)};
  normal := {-sin(nominalPose.heading), cos(nominalPose.heading)};
  tangentialFactor := 1.0 - nominalKappa * offsetDerivative[1];
  firstNominal := tangentialFactor * tangent + offsetDerivative[2] * normal;
  firstTangential := -2.0 * nominalKappa * offsetDerivative[2];
  secondNormal := nominalKappa * tangentialFactor + offsetDerivative[3];
  secondNominal := firstTangential * tangent + secondNormal * normal;
  thirdTangential := -3.0 * nominalKappa * offsetDerivative[3]
    - nominalKappa^2 * tangentialFactor;
  thirdNormal := offsetDerivative[4]
    - 3.0 * nominalKappa^2 * offsetDerivative[2];
  thirdNominal := thirdTangential * tangent + thirdNormal * normal;

  scale := sqrt(firstNominal * firstNominal);
  assert(scale > 1.0e-10,
    "Dubins-polynomial offset is singular; increase nominal radius or offset penalty");
  innerProduct := firstNominal * secondNominal;
  innerProductDerivative := secondNominal * secondNominal
    + firstNominal * thirdNominal;
  state.firstDerivative := firstNominal / scale;
  state.secondDerivative := secondNominal / scale^2
    - firstNominal * innerProduct / scale^4;
  state.thirdDerivative := thirdNominal / scale^3
    - 3.0 * secondNominal * innerProduct / scale^5
    - firstNominal * innerProductDerivative / scale^5
    + 4.0 * firstNominal * innerProduct^2 / scale^7;
  state.position := nominalPose.position + offsetDerivative[1] * normal;
  state.heading := nominalPose.heading
    + atan2(offsetDerivative[2], tangentialFactor);
  state.curvature := state.firstDerivative[1] * state.secondDerivative[2]
    - state.firstDerivative[2] * state.secondDerivative[1];
  state.curvatureDerivative :=
    state.firstDerivative[1] * state.thirdDerivative[2]
    - state.firstDerivative[2] * state.thirdDerivative[1];
  state.offset := offsetDerivative[1];
  state.offsetDerivative := offsetDerivative;
  state.metricScale := scale;
  state.segmentIndex := segmentIndex;
end evaluateSegment;
