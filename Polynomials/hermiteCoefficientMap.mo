within Polynomials;
function hermiteCoefficientMap
  "Map endpoint derivatives to minimum-degree Hermite coefficients"
  input Integer derivativeCount(min=1);
  input Real intervalLength(min=0.0);
  output Real coefficientMap[2 * derivativeCount, 2 * derivativeCount];
  output Boolean accepted;
protected
  Integer coefficientCount = 2 * derivativeCount;
  Integer power;
  Real constraints[2 * derivativeCount, 2 * derivativeCount];
  Real identityMatrix[2 * derivativeCount, 2 * derivativeCount];
algorithm
  assert(intervalLength > 0.0, "Hermite interval length must be positive");
  constraints := zeros(coefficientCount, coefficientCount);
  identityMatrix := identity(coefficientCount);
  for derivativeIndex in 1:derivativeCount loop
    for coefficientIndex in 1:coefficientCount loop
      power := coefficientIndex - 1;
      if power >= derivativeIndex - 1 then
        constraints[derivativeIndex, coefficientIndex] :=
          Polynomials.fallingFactorial(power, derivativeIndex - 1)
          * 0.0^(power - derivativeIndex + 1);
        constraints[derivativeCount + derivativeIndex, coefficientIndex] :=
          Polynomials.fallingFactorial(power, derivativeIndex - 1)
          * intervalLength^(power - derivativeIndex + 1);
      end if;
    end for;
  end for;
  (coefficientMap, accepted) := LinearAlgebra.solve(
    constraints, identityMatrix);
end hermiteCoefficientMap;
