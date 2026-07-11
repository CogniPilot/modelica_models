within Polynomials;
function evaluateDerivative
  "Evaluate any derivative of a power-basis polynomial"
  input Real coefficient[:] "Increasing power order";
  input Real abscissa;
  input Integer derivativeOrder(min=0) = 0;
  output Real value;
protected
  Integer power;
algorithm
  value := 0.0;
  for coefficientIndex in 1:size(coefficient, 1) loop
    power := coefficientIndex - 1;
    if power >= derivativeOrder then
      value := value + coefficient[coefficientIndex]
        * Polynomials.fallingFactorial(power, derivativeOrder)
        * abscissa^(power - derivativeOrder);
    end if;
  end for;
end evaluateDerivative;
