within Planning.DubinsPolynomial;
function invertNormalDerivatives
  "Invert true unit-speed normal derivatives to nominal-distance offset derivatives"
  input Real offset;
  input Real nominalCurvature;
  input Real trueNormalDerivative[3]
    "Normal components of true first through third spatial derivatives";
  output Real polynomialDerivative[3]
    "First through third offset derivatives in nominal distance";
  output Boolean accepted;
protected
  Real tangentialFactor;
  Real firstDerivative;
  Real secondDerivative;
  Real scale;
  Real firstTangential;
  Real secondNormal;
  Real innerProduct;
  Real thirdTangential;
  Real innerProductDerivativeWithoutThirdNormal;
  Real thirdNormalRemainder;
  Real thirdNormal;
algorithm
  tangentialFactor := 1.0 - nominalCurvature * offset;
  accepted := tangentialFactor > 1.0e-8 and
    abs(trueNormalDerivative[1]) < 1.0 - 1.0e-10;
  if accepted then
    firstDerivative := tangentialFactor * trueNormalDerivative[1]
      / sqrt(1.0 - trueNormalDerivative[1]^2);
    scale := sqrt(tangentialFactor^2 + firstDerivative^2);
    firstTangential := -2.0 * nominalCurvature * firstDerivative;
    secondDerivative := trueNormalDerivative[2] * scale^4
      / tangentialFactor^2 - nominalCurvature * tangentialFactor
      - 2.0 * nominalCurvature * firstDerivative^2 / tangentialFactor;
    secondNormal := nominalCurvature * tangentialFactor + secondDerivative;
    innerProduct := tangentialFactor * firstTangential
      + firstDerivative * secondNormal;
    thirdTangential := -3.0 * nominalCurvature * secondDerivative
      - nominalCurvature^2 * tangentialFactor;
    innerProductDerivativeWithoutThirdNormal :=
      firstTangential^2 + secondNormal^2
      + tangentialFactor * thirdTangential;
    thirdNormalRemainder :=
      -3.0 * secondNormal * innerProduct / scale^5
      - firstDerivative * innerProductDerivativeWithoutThirdNormal / scale^5
      + 4.0 * firstDerivative * innerProduct^2 / scale^7;
    thirdNormal := (trueNormalDerivative[3] - thirdNormalRemainder)
      * scale^5 / tangentialFactor^2;
    polynomialDerivative := {
      firstDerivative,
      secondDerivative,
      thirdNormal + 3.0 * nominalCurvature^2 * firstDerivative};
  else
    polynomialDerivative := zeros(3);
  end if;
end invertNormalDerivatives;
