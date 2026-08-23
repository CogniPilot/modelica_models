within Tests;
package RuleChecks
  "Central-difference oracles and residual drivers for the LieGroups derivative rules"
  annotation(Documentation(info="<html>
    <p>Each <code>fd*</code> function is a central difference of one LieGroups
    primitive taken in exactly the trivialization that primitive's rule
    declares: a group-valued slot is perturbed as
    <code>X*exp_map(h e)</code>, a group-valued result is read back as
    <code>log_map(Y^-1 Y_perturbed)</code>, and plain vectors are perturbed and
    read back additively. The <code>*Residuals</code> drivers sweep randomized
    points at a chosen magnitude and return the worst absolute disagreement
    between rule and difference, one entry per rule.</p>
    <p>The step is a caller argument because the two error sources move in
    opposite directions: the central difference truncates at O(h^2) times a
    third derivative and rounds at O(eps/h) times the value, so the floor sits
    near h = 1e-6 for double-precision quantities of order one.</p>
  </html>"));
end RuleChecks;
