within Optimization;
function boxQP "Box-constrained convex QP: minimize 1/2 x'Hx + g'x with lower <= x <= upper"
  input Real H[:, size(H, 1)] "Symmetric positive definite Hessian";
  input Real g[size(H, 1)] "Linear term";
  input Real lower[size(H, 1)] "Lower bounds (may be very negative for unbounded)";
  input Real upper[size(H, 1)] "Upper bounds (may be very positive for unbounded)";
  input Real x0[size(H, 1)] = zeros(size(H, 1)) "Optional warm start; default is a cold start";
  input Optimization.SolverSettings settings = Optimization.SolverSettings() "Iteration controls";
  output Real x[size(H, 1)] "Solution";
  output Integer active[size(H, 1)] "-1 at the lower bound, 0 free, +1 at the upper bound";
  output Real residual "Largest independent coordinate projected displacement at x, in the units of x";
  output Integer iterations "Full sweeps performed";
  output Boolean converged "True when residual <= settings.tolerance";
protected
  Real gradient;
algorithm
  for i in 1:size(H, 1) loop
    assert(H[i, i] > 0.0, "boxQP requires positive diagonal entries in H");
    assert(lower[i] <= upper[i], "boxQP requires lower <= upper");
    x[i] := min(max(x0[i], lower[i]), upper[i]);
  end for;
  residual := Optimization.Internal.boxResidual(H, g, lower, upper, x);
  iterations := 0;
  // A bounded for loop with a convergence guard instead of a while loop, so compilers that
  // reject while loops in functions can lower it; sweeps stop changing x once converged.
  for sweep in 1:settings.maxIterations loop
    if residual > settings.tolerance then
      for i in 1:size(H, 1) loop
        gradient := g[i];
        for j in 1:size(H, 1) loop
          gradient := gradient + H[i, j] * x[j];
        end for;
        x[i] := min(max(x[i] - gradient / H[i, i], lower[i]), upper[i]);
      end for;
      iterations := iterations + 1;
      residual := Optimization.Internal.boxResidual(H, g, lower, upper, x);
    end if;
  end for;
  converged := residual <= settings.tolerance;
  for i in 1:size(H, 1) loop
    active[i] := if x[i] <= lower[i] then -1 elseif x[i] >= upper[i] then 1 else 0;
  end for;
  annotation(Documentation(info = "<html>
<p>Projected Gauss-Seidel: each sweep minimizes the objective exactly over one
coordinate at a time and clamps it to its bounds, so the objective never
increases. For symmetric positive definite H the sweeps converge to the unique
optimum.</p>
<p><code>residual</code> is the largest independent coordinate projected
displacement at the returned <code>x</code>: for each coordinate separately, the
gradient divided by the diagonal of H, then clamped, all evaluated at the same x.
It is not the displacement of the next sequential sweep. It is in the units of x
and is zero exactly at the optimum, whichever method produced x.
<code>active</code> is the bound state of each coordinate. At a nondegenerate active
set (no free coordinate exactly at a bound, and nonzero multipliers on the active
bounds) a sensitivity can be obtained from a reduced solve on the free coordinates;
no global differentiability of the solution is claimed.</p>
</html>"));
end boxQP;
