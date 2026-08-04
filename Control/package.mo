within;
package Control "Reusable control algorithms"
  annotation(
    uses(LieGroups, MathUtilities, Planning),
    Documentation(info="<html>
    <p>Execution-neutral feedback, guidance, and control components. Pure
    control maps are kept independent of vehicle parameterizations; named
    vehicles extend the reusable models with their physical constants and
    tuning.</p>
  </html>"));
end Control;
