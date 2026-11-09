within Optimization;
record SolverSettings "Iteration controls shared by the QP solvers"
  Optimization.SolverMethod method = Optimization.SolverMethod.ProjectedGaussSeidel
    "Iterative method";
  Integer maxIterations = 1000 "Maximum number of full sweeps";
  Real tolerance = 1.0e-12
    "Stop when the residual is at or below this value, in the units of the decision variable";
  annotation(Documentation(info = "<html>
<p><code>tolerance</code> is compared with the solver residual, which is in the
units of the decision variable. Scale it for the problem at hand; the default is
a numerical default only.</p>
</html>"));
end SolverSettings;
