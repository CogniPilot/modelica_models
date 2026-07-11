within Polynomials;
function hermiteCoefficients
  "Minimum-degree polynomial satisfying endpoint derivatives of arbitrary order"
  input Real startDerivative[:]
    "Value and successive derivatives at the start";
  input Real endDerivative[size(startDerivative, 1)]
    "Value and successive derivatives at the end";
  input Real intervalLength;
  output Real coefficient[2 * size(startDerivative, 1)]
    "Increasing power order in the physical abscissa";
  output Boolean accepted;
protected
  Integer derivativeCount = size(startDerivative, 1);
  Integer coefficientCount = 2 * size(startDerivative, 1);
  Integer power;
  Real constraints[2 * size(startDerivative, 1),
    2 * size(startDerivative, 1)];
  Real boundary[2 * size(startDerivative, 1), 1];
  Real solution[2 * size(startDerivative, 1), 1];
algorithm
  assert(intervalLength > 0.0,
    "Hermite polynomial interval length must be positive");
  constraints := zeros(coefficientCount, coefficientCount);
  boundary[:, 1] := cat(1, startDerivative, endDerivative);
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
  (solution, accepted) := LinearAlgebra.solve(constraints, boundary);
  coefficient := solution[:, 1];
end hermiteCoefficients;
