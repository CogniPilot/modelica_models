within Optimization.Internal;
function frictionConeResidual "Largest block step a projected sweep would take"
  input Real A[:, size(A, 1)];
  input Real b[size(A, 1)];
  input Real mu[:];
  input Real lipschitz[size(mu, 1)];
  input Real f[size(A, 1)];
  output Real residual "In the units of f; zero exactly at the optimum";
protected
  Real step[3];
algorithm
  residual := 0.0;
  for c in 1:size(mu, 1) loop
    step := Optimization.Internal.contactBlockStep(A, b, f, c, mu[c], lipschitz[c]);
    for k in 1:3 loop
      residual := max(residual, abs(f[3 * (c - 1) + k] - step[k]));
    end for;
  end for;
end frictionConeResidual;
