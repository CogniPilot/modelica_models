within Tests.Assertions;
function frictionConeQPCheck "Error of a cold-start frictionConeQP solve against a known optimum and contact states"
  input Real mu[:];
  input Real A[3 * size(mu, 1), 3 * size(mu, 1)];
  input Real b[3 * size(mu, 1)];
  input Real expected[3 * size(mu, 1)];
  input Integer expectedState[size(mu, 1)];
  output Real error "Largest solution error, plus 1 per contact-state mismatch, plus 1 if not converged";
protected
  Real f[3 * size(mu, 1)];
  Integer contactState[size(mu, 1)];
  Real residual;
  Integer iterations;
  Boolean converged;
algorithm
  (f, contactState, residual, iterations, converged) := Optimization.frictionConeQP(mu, A, b);
  error := 0.0;
  for i in 1:3 * size(mu, 1) loop
    error := max(error, abs(f[i] - expected[i]));
  end for;
  for c in 1:size(mu, 1) loop
    if contactState[c] <> expectedState[c] then
      error := error + 1.0;
    end if;
  end for;
  if not converged then
    error := error + 1.0;
  end if;
end frictionConeQPCheck;
