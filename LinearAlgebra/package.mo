within;
package LinearAlgebra
  "Dimension-generic numerical linear algebra for estimation and control"
  annotation(Documentation(info = "<html>
    <h4>Choose an operation</h4>
<p>Dimension-generic functions accept arrays whose extents follow their inputs.
Use matrix products and solves instead of forming explicit inverses.</p>
<table>
<tr><th>Task</th><th>Function</th></tr>
<tr><td>General square system</td><td><a href=\"modelica://LinearAlgebra.solve\">solve</a></td></tr>
<tr><td>Positive-definite system</td><td><a href=\"modelica://LinearAlgebra.solveSPD\">solveSPD</a></td></tr>
<tr><td>Covariance correction</td><td><a href=\"modelica://LinearAlgebra.josephUpdate\">josephUpdate</a></td></tr>
<tr><td>Covariance coordinate change</td><td><a href=\"modelica://LinearAlgebra.transformCovariance\">transformCovariance</a></td></tr>
<tr><td>Factor from noise-factor columns</td><td><a href=\"modelica://LinearAlgebra.covarianceRoot\">covarianceRoot</a></td></tr>
</table>
<h4>Example</h4>
<p>With <code>Real solution[2,1]</code> and <code>Boolean accepted</code>, this
algorithm fragment solves two diagonal equations:</p>
<pre>(solution, accepted) := LinearAlgebra.solveSPD([2,0;0,4], [2;8]);</pre>
<p>The accepted result is {1,2} as a column. Always check the returned acceptance
flag before using a solve. Positive semidefinite and positive definite are
separate contracts; a rank-deficient covariance is not an invertible system.
Numerical acceptance depends on the precision executing the generated code.</p>
    </html>"));
end LinearAlgebra;
