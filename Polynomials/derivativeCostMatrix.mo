within Polynomials;
function derivativeCostMatrix
  "Integral quadratic cost matrix for a polynomial derivative"
  input Integer coefficientCount(min=1);
  input Integer derivativeOrder(min=0);
  input Real intervalLength(min=0.0);
  output Real cost[coefficientCount, coefficientCount];
protected
  Integer leftPower;
  Integer rightPower;
  Integer exponent;
algorithm
  cost := zeros(coefficientCount, coefficientCount);
  for row in 1:coefficientCount loop
    leftPower := row - 1;
    for column in 1:coefficientCount loop
      rightPower := column - 1;
      if leftPower >= derivativeOrder and rightPower >= derivativeOrder then
        exponent := leftPower + rightPower - 2 * derivativeOrder + 1;
        cost[row, column] :=
          Polynomials.fallingFactorial(leftPower, derivativeOrder)
          * Polynomials.fallingFactorial(rightPower, derivativeOrder)
          * intervalLength^exponent / exponent;
      end if;
    end for;
  end for;
end derivativeCostMatrix;
