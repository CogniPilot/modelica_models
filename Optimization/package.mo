within;
package Optimization "Convex optimization primitives for contact and control"
  annotation(Documentation(info = "<html>
<p>Small convex quadratic programs solved by pure Modelica algorithms, with no
external C and no Modelica Standard Library dependency.</p>
<ul>
<li><a href=\"modelica://Optimization.boxQP\">boxQP</a>: minimize
1/2 x'Hx + g'x subject to lower &lt;= x &lt;= upper.</li>
<li><a href=\"modelica://Optimization.frictionConeQP\">frictionConeQP</a>: minimize
1/2 f'Af + b'f with one elliptic friction cone per contact.</li>
</ul>
<p>Every solver returns its solution together with the active set (which bound or
cone face is active), a method-independent optimality residual, the iteration
count and a converged flag. The active set is what an exact sensitivity of the
solution needs: differentiating the optimality conditions at that active set,
instead of differentiating through the iterations. Solver settings are a record
with a method field, so further methods can be added without changing the
function signatures.</p>
<p>The residual is measured in the units of the decision variable (for example
impulse or force for contact), and <code>SolverSettings.tolerance</code> uses the
same units. Callers choose the tolerance for their own scaling; the default is
only a numerical default and is not tied to any integrator state tolerance.</p>
<p>The warm start is an optional input with a cold-start default. The functions
keep no memory between calls, so a result depends only on the inputs.</p>
</html>"));
end Optimization;
