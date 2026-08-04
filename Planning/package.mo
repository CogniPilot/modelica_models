within;
package Planning "Path and trajectory planning algorithms"
  annotation(
    uses(Geodesy, LieGroups, LinearAlgebra, Polynomials),
    Documentation(info="<html>
    <p>Geometric path planning, polynomial smoothing, and time-parameterized
    trajectory construction. Packages expose explicit conventions and keep
    vehicle dynamics outside the planning layer.</p>
  </html>"));
end Planning;
