within;
package Polynomials "Dimension-generic polynomial construction and analysis"
  annotation(uses(LinearAlgebra),
    Documentation(info = "<html>
    <h4>Endpoint-constrained trajectories</h4>
<p><a href=\"modelica://Polynomials.hermiteCoefficients\">hermiteCoefficients</a>
builds the minimum-degree polynomial matching endpoint values and derivatives.
Arrays are ordered by increasing derivative or power: {value, first derivative,
...} at an endpoint and {constant, linear, quadratic, ...} for coefficients.</p>
<h4>Example</h4>
<p>Declare <code>Real coefficient[4]</code>, <code>Boolean accepted</code> and
<code>Real midpoint</code>, then use:</p>
<pre>(coefficient, accepted) := Polynomials.hermiteCoefficients({0,0}, {1,0}, 2);
midpoint := Polynomials.evaluateDerivative(coefficient, 1, 0);</pre>
<p>This cubic goes from zero to one over a physical interval of length two,
with zero endpoint slopes; its midpoint value is one half. Check acceptance
before evaluation. Interval length must be positive.</p>
<p><a href=\"modelica://Polynomials.evaluateDerivative\">evaluateDerivative</a>
uses the physical abscissa, not a normalized phase.
<a href=\"modelica://Polynomials.derivativeCostMatrix\">derivativeCostMatrix</a>
returns Q such that c' Q c is the integrated squared derivative. Preserve the
physical interval and units when combining segment costs.</p>
    </html>"));
end Polynomials;
