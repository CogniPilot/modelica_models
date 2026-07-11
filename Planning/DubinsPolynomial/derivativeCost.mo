within Planning.DubinsPolynomial;
function derivativeCost
  "Integrated squared offset-derivative cost for arbitrary polynomial order"
  input Planning.Dubins.Path path;
  input Real offsetCoefficient[3, :];
  input Integer derivativeOrder(min=0);
  output Real cost;
protected
  Integer coefficientCount = size(offsetCoefficient, 2);
  Real segmentLength;
  Real costMatrix[size(offsetCoefficient, 2), size(offsetCoefficient, 2)];
algorithm
  cost := 0.0;
  for segmentIndex in 1:3 loop
    segmentLength := path.turnRadius
      * path.normalizedSegmentLength[segmentIndex];
    costMatrix := Polynomials.derivativeCostMatrix(
      coefficientCount, derivativeOrder, segmentLength);
    cost := cost + offsetCoefficient[segmentIndex, :]
      * (costMatrix * offsetCoefficient[segmentIndex, :]);
  end for;
end derivativeCost;
