within Tests.Assertions;
function boxQPWarmStartIterations "Sweeps boxQP needs when warm-started at x0 with explicit settings"
  input Real H[:, size(H, 1)];
  input Real g[size(H, 1)];
  input Real lower[size(H, 1)];
  input Real upper[size(H, 1)];
  input Real x0[size(H, 1)];
  output Real sweeps "Iterations as a Real; a negative value means the solve did not converge";
protected
  Real x[size(H, 1)];
  Integer active[size(H, 1)];
  Real residual;
  Integer iterations;
  Boolean converged;
algorithm
  (x, active, residual, iterations, converged) := Optimization.boxQP(
    H, g, lower, upper, x0,
    Optimization.SolverSettings(maxIterations = 1000, tolerance = 1.0e-12));
  sweeps := if converged then iterations else -1.0;
end boxQPWarmStartIterations;
