within Tests;
package JacobianRuleTests
  "LieGroups closed-form derivative rules against central differences"
  annotation(Documentation(info="<html>
    <p>Every rule in the LieGroups derivative-rule library is compared against a
    central difference of its own primitive, taken in the trivialization the
    rule declares, at randomized points spanning six decades of rotation
    magnitude.</p>
    <p>Magnitudes are dense from 1e-6 to 3 rad and deliberately cover 0.09 to
    0.11, the band where the SE(3) and SE_2(3) Jacobian building blocks switch
    between their retained series and their closed forms. That band is where a
    rule that borrowed the 0.1 rad radius carries a truncation of order
    |omega|^5/720 that no other magnitude shows, so skipping it hides exactly
    the defect it would catch.</p>
    <p>One magnitude is excluded, and only from the exp_mixed sweep:
    |omega| = 0.1 rad itself. <code>exp_mixed</code>'s own coefficients switch
    there and the two sides differ by about 1.4e-7 in the leading coefficient,
    so a difference whose 1e-6 step straddles the radius divides that jump by
    2e-6 and reports 7e-2 no matter how exact the rule is. That argument is
    about a branch inside the differenced primitive and applies at that
    magnitude alone; it says nothing about 0.06 to 0.099, and it does not apply
    to the SO(3) sweep at all, whose differences call only
    <code>exp_map</code>, <code>log_map</code>, <code>product</code>,
    <code>inverse</code> and <code>rotate</code>, none of which branches at
    0.1 rad. The SO(3) sweep therefore includes 0.1 exactly.</p>
    <p>Tolerances are difference floors, not rule accuracy, and they are stated
    per rule rather than per model. A binary64 central difference of a quantity
    of magnitude M costs about eps*M/h in rounding, which at h = 1e-6 is 2e-10
    for M = 1, 1e-9 for the 5 m vectors the rotate rules carry, 1e-8 for a 50 m
    position lever arm and 4e-8 for 200 m. Each budget is a small multiple of
    its own rule's floor, so an assertion stays about that rule rather than
    about the last bits of its oracle or about whichever neighbour happens to
    have the largest floor.</p>
    <p>One model per rule family, rather than one model for all of them, so that
    a failure names the family and so that each model stays small enough to
    lower quickly.</p>
  </html>"));
end JacobianRuleTests;
