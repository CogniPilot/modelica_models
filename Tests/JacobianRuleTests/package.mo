within Tests;
package JacobianRuleTests
  "LieGroups closed-form derivative rules against central differences"
  annotation(Documentation(info="<html>
    <p>Every rule in the LieGroups derivative-rule library is compared against a
    central difference of its own primitive, taken in the trivialization the
    rule declares, at randomized points spanning six decades of rotation
    magnitude.</p>
    <p>Magnitudes deliberately avoid |omega| = 0.1 rad. That is the radius where
    <code>exp_mixed</code> and the SO(3) Jacobians switch between their retained
    series and their closed forms, and the two sides differ by about 1.4e-7 in
    the leading coefficient. A difference whose step straddles that radius
    divides a 1.4e-7 jump by 2e-6 and reports a 7e-2 disagreement no matter how
    exact the rule is, so the difference, not the rule, is what fails there.</p>
    <p>Tolerances are difference floors, not rule accuracy. A binary64 central
    difference of a quantity of magnitude M costs about eps*M/h in rounding,
    which at h = 1e-6 is 2e-10 for M = 1 and 1e-8 for the 50 m position lever
    arms the SE_2(3) rules carry. The budgets sit above those floors with room
    to spare so that an assertion stays about the rule rather than about the
    last bits of its oracle.</p>
    <p>One model per rule family, rather than one model for all of them, so that
    a failure names the family and so that each model stays small enough to
    lower quickly.</p>
  </html>"));
end JacobianRuleTests;
