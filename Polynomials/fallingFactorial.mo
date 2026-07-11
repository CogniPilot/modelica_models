within Polynomials;
function fallingFactorial "n*(n-1)*...*(n-order+1)"
  input Integer n(min=0);
  input Integer order(min=0);
  output Real value;
algorithm
  value := 1.0;
  if order > n then
    value := 0.0;
  else
    for factorIndex in 0:order - 1 loop
      value := value * (n - factorIndex);
    end for;
  end if;
end fallingFactorial;
