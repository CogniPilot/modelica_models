within;
package Estimation "Aided inertial filters and delayed fusion"
  annotation(uses(Avionics, LieGroups, LinearAlgebra, MathUtilities),
    Documentation(info = "<html>
    <p><a href=\"modelica://Estimation.StrapdownINS\">StrapdownINS</a>
    provides aided inertial filters.
    <a href=\"modelica://Estimation.FusionHorizon\">FusionHorizon</a>
    provides reusable delayed delivery and current-time prediction around
    those filters. Both use the physical sensor and navigation contracts in
    <a href=\"modelica://Avionics\">Avionics</a>.</p>
    </html>"));
end Estimation;
