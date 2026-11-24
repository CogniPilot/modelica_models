within Tests.Assertions;
function boxQPCheck "Error of a cold-start boxQP solve against a known optimum and active set"
  input Real H[:, size(H, 1)];
  input Real g[size(H, 1)];
  input Real lower[size(H, 1)];
  input Real upper[size(H, 1)];
  input Real expected[size(H, 1)];
  input Integer expectedActive[size(H, 1)];
  output Real error "Largest solution error, plus 1 per active-set mismatch, plus 1 if not converged";
protected
  Real x[size(H, 1)];
  Integer active[size(H, 1)];
  Real residual;
  Integer iterations;
  Boolean converged;
algorithm
  (x, active, residual, iterations, converged) := Optimization.boxQP(H, g, lower, upper);
  error := 0.0;
  for i in 1:size(H, 1) loop
    error := max(error, abs(x[i] - expected[i]));
  end for;
  for i in 1:size(H, 1) loop
    if active[i] <> expectedActive[i] then
      error := error + 1.0;
    end if;
  end for;
  if not converged then
    error := error + 1.0;
  end if;
end boxQPCheck;
