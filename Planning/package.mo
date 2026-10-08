within;
package Planning "Path and trajectory planning algorithms"
  annotation(
    uses(Geodesy, LieGroups, LinearAlgebra, Polynomials),
    Documentation(info="<html>
    <p>Geometric path planning, polynomial smoothing, and time-parameterized
    trajectory construction. Packages expose explicit conventions and keep
    vehicle dynamics outside the planning layer.</p>
  <h4>Choose a planning layer</h4>
<ul>
<li><a href=\"modelica://Planning.Dubins\">Dubins</a> constructs planar,
forward-only paths with a minimum turn radius.</li>
<li><a href=\"modelica://Planning.DubinsPolynomial\">DubinsPolynomial</a>
smooths a Dubins reference using polynomial offsets.</li>
<li><a href=\"modelica://Planning.Bezier\">Bezier</a> supplies waypoint and
polynomial trajectory tools.</li>
<li><a href=\"modelica://Planning.Interfaces\">Interfaces</a> holds the shared
trajectory boundary.</li>
</ul>
<p>Geometric path distance and elapsed trajectory time are distinct variables.
A path's curvature bound does not itself impose speed, acceleration, thrust or
actuator limits. Check those limits against the vehicle and controller.</p>
<h4>Examples</h4>
<p>Start with <a href=\"modelica://Planning.Examples.DubinsFamilyGallery\">DubinsFamilyGallery</a>
for the path families and
<a href=\"modelica://Planning.Examples.DubinsAircraftFlightPlan\">DubinsAircraftFlightPlan</a>
for a complete aircraft reference. Global waypoints are projected once through
<a href=\"modelica://Geodesy\">Geodesy</a> about a fixed mission origin.</p>
    </html>"));
end Planning;
