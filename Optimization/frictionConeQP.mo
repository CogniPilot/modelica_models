within Optimization;
function frictionConeQP
  "Contact QP: minimize 1/2 f'Af + b'f with one elliptic friction cone per contact"
  input Real mu[:] "Friction coefficient per contact (>= 0)";
  input Real A[3 * size(mu, 1), 3 * size(mu, 1)]
    "Symmetric positive semidefinite Delassus matrix; rows (normal, tangent1, tangent2) per contact";
  input Real b[3 * size(mu, 1)] "Linear term";
  input Real f0[3 * size(mu, 1)] = zeros(3 * size(mu, 1))
    "Optional warm start; default is a cold start";
  input Optimization.SolverSettings settings = Optimization.SolverSettings() "Iteration controls";
  output Real f[3 * size(mu, 1)] "Contact impulses or forces, (normal, tangent1, tangent2) per contact";
  output Integer contactState[size(mu, 1)]
    "Geometric class of f per contact: 0 zero force, 1 cone interior, 2 cone boundary";
  output Real residual "Largest independent block projected displacement at f, in the units of f";
  output Integer iterations "Full sweeps performed";
  output Boolean converged "True when residual <= settings.tolerance";
protected
  Real lipschitz[size(mu, 1)];
  Real rowSum;
  Real step[3];
  Real slip;
  Real normal;
  Real tangent1;
  Real tangent2;
algorithm
  for c in 1:size(mu, 1) loop
    assert(mu[c] >= 0.0, "frictionConeQP requires mu >= 0");
    lipschitz[c] := 0.0;
    for k in 1:3 loop
      rowSum := 0.0;
      for j in 1:3 loop
        rowSum := rowSum + abs(A[3 * (c - 1) + k, 3 * (c - 1) + j]);
      end for;
      lipschitz[c] := max(lipschitz[c], rowSum);
    end for;
    assert(lipschitz[c] > 0.0, "frictionConeQP requires a nonzero diagonal block per contact");
    (normal, tangent1, tangent2) := Optimization.Internal.projectCone(
      f0[3 * c - 2], f0[3 * c - 1], f0[3 * c], mu[c]);
    f[3 * c - 2] := normal;
    f[3 * c - 1] := tangent1;
    f[3 * c] := tangent2;
  end for;
  residual := Optimization.Internal.frictionConeResidual(A, b, mu, lipschitz, f);
  iterations := 0;
  // Bounded for loop with a convergence guard (no while loop), as in boxQP.
  for sweep in 1:settings.maxIterations loop
    if residual > settings.tolerance then
      for c in 1:size(mu, 1) loop
        step := Optimization.Internal.contactBlockStep(A, b, f, c, mu[c], lipschitz[c]);
        f[3 * c - 2] := step[1];
        f[3 * c - 1] := step[2];
        f[3 * c] := step[3];
      end for;
      iterations := iterations + 1;
      residual := Optimization.Internal.frictionConeResidual(A, b, mu, lipschitz, f);
    end if;
  end for;
  converged := residual <= settings.tolerance;
  for c in 1:size(mu, 1) loop
    slip := sqrt(f[3 * c - 1] ^ 2 + f[3 * c] ^ 2);
    contactState[c] := if f[3 * c - 2] <= 0.0 then 0
      elseif slip < mu[c] * f[3 * c - 2] * (1.0 - 1.0e-10) then 1 else 2;
  end for;
  annotation(Documentation(info = "<html>
<p>Block projected Gauss-Seidel: for each contact in turn, take one gradient step
on its three components with step 1/L (L is the largest absolute row sum of that
contact's diagonal block, an upper bound on its largest eigenvalue), then project
exactly onto the friction cone. Block coordinate projected gradient with this
step converges for convex problems.</p>
<p><code>residual</code> is the largest independent block projected displacement at
the returned <code>f</code>: for each contact separately, one projected block step
evaluated at the same f. It is not the displacement of the next sequential sweep.
It is in the units of f (impulse or force) and is zero exactly at the optimum,
whichever method produced f.</p>
<p><code>contactState</code> classifies f geometrically: zero force, cone interior or
cone boundary. It is not a velocity-based separation, stick or slip mode. A force
within a relative 1e-10 of the cone boundary is reported as boundary; with mu = 0
every nonzero force lies on the degenerate cone boundary. At a nondegenerate
active set this classification is what a sensitivity of f needs; no global
differentiability of the solution is claimed.</p>
</html>"));
end frictionConeQP;
