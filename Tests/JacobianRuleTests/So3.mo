within Tests.JacobianRuleTests;
model So3 "SO(3) rules against central differences over a dense magnitude sweep"
  constant Real step = 1.0e-6
    "Central-difference step: the O(h^2) truncation and the O(eps/h) rounding
     of a binary64 central difference cross near 1e-6 for quantities of order one";
  constant Integer trials = 6 "Randomized points per magnitude";
  constant Real magnitudes[32] = {
    1.0e-6, 3.0e-6, 1.0e-5, 3.0e-5, 1.0e-4, 3.0e-4, 1.0e-3, 3.0e-3,
    0.01, 0.03, 0.05, 0.07, 0.08, 0.085, 0.09, 0.092,
    0.094, 0.096, 0.098, 0.0999, 0.1, 0.1001, 0.102, 0.105,
    0.11, 0.2, 0.3, 0.5, 1.0, 1.5, 2.0, 3.0}
    "Dense from 1e-6 to 3 rad. 1e-4 rad is the radius where exp_map and the
     rule-local Jacobians switch between series and closed form, and 0.09 to
     0.11 brackets the radius the SE(3) and SE_2(3) building blocks switch at";
  constant Real tolerance[7] = {
    1.0e-9, 2.0e-9, 1.5e-9, 1.5e-9, 1.0e-9, 5.0e-9, 2.5e-9}
    "One budget per rule: exp_map, log_map, product left, product right,
     inverse, rotate rotation, rotate vector. Each is about five times what
     that rule measures over the whole sweep, which is its own difference
     floor: eps times the magnitude of the quantity being differenced, divided
     by the step, so the rules that carry a 5 m target vector sit higher than
     the rules that carry only rotations";
  constant Real familyTolerance = 1.0e-14
    "The three identities are exact: the first measures 4.4e-16 and the other
     two measure zero, so this budget is rounding, not accuracy";
  Real sweep[7] "Worst over the whole magnitude list, one entry per rule";
  Real family[3] "J_r J_r^-1 - I, J_l - J_r^T, J_l^-1 - (J_r^-1)^T";
  Real series[7] "0.05 rad";
  Real belowBranch[7] "0.095 rad";
  Real atBranch[7] "0.0999 rad, the last magnitude below 0.1";
  Real large[7] "2.0 rad";
equation
  sweep = Tests.RuleChecks.so3RuleResidualsSweep(magnitudes, step, trials);
  family = Tests.RuleChecks.so3JacobianFamilyResiduals(magnitudes, trials);
  series = Tests.RuleChecks.so3RuleResiduals(0.05, step, trials);
  belowBranch = Tests.RuleChecks.so3RuleResiduals(0.095, step, trials);
  atBranch = Tests.RuleChecks.so3RuleResiduals(0.0999, step, trials);
  large = Tests.RuleChecks.so3RuleResiduals(2.0, step, trials);

  assert(sweep[1] < tolerance[1],
    "exp_map_jacobian disagrees with a central difference of exp_map");
  assert(sweep[2] < tolerance[2],
    "log_map_jacobian disagrees with a central difference of log_map");
  assert(sweep[3] < tolerance[3],
    "product_left_factor_jacobian disagrees with a central difference of product");
  assert(sweep[4] < tolerance[4],
    "product_right_factor_jacobian disagrees with a central difference of product");
  assert(sweep[5] < tolerance[5],
    "inverse_jacobian disagrees with a central difference of inverse");
  assert(sweep[6] < tolerance[6],
    "rotate_rotation_jacobian disagrees with a central difference of rotate");
  assert(sweep[7] < tolerance[7],
    "rotate_vector_jacobian disagrees with a central difference of rotate");

  assert(family[1] < familyTolerance,
    "right_jacobian_exact and right_jacobian_inv_exact are not inverse");
  assert(family[2] < familyTolerance,
    "left_jacobian_exact is not the transpose of right_jacobian_exact");
  assert(family[3] < familyTolerance,
    "left_jacobian_inv_exact is not the transpose of right_jacobian_inv_exact");

  // The named rows are inside the sweep and are recorded so a failure can be
  // placed. 0.095 and 0.0999 rad are where a rule that carried the SE(3)
  // building blocks' 0.1 rad series radius reports its own truncation, of
  // order |v|^5/720, as a disagreement with exp_map: 1.0e-8 and 1.3e-8, both
  // over the 1e-8 budget this model used to share between all seven rules.
  assert(Tests.Assertions.maxAbsVector(belowBranch) < 5.0e-9,
    "SO(3) rules disagree with central differences at 0.095 rad");
  assert(Tests.Assertions.maxAbsVector(atBranch) < 5.0e-9,
    "SO(3) rules disagree with central differences at 0.0999 rad");

  annotation(Documentation(info="<html>
    <p>Each vector entry is one rule: exp_map, log_map, product in its left
    factor, product in its right factor, inverse, rotate in its rotation slot,
    and rotate in its vector slot. Budgets are per rule rather than per model:
    one shared budget hides a rule sitting a hundred times worse than its
    neighbours, which is how the two-term series in <code>right_jacobian</code>
    reached 1.3e-8 at 0.0999 rad while its six neighbours measured 1e-11 to
    8e-10 and no assertion noticed.</p>
    <p>The sweep is dense and includes 0.1 rad itself. The central differences
    here call only <code>exp_map</code>, <code>log_map</code>,
    <code>product</code>, <code>inverse</code> and <code>rotate</code>, none of
    which branches at 0.1 rad, so no magnitude in this model straddles a branch
    of its own oracle. The rules and their oracle now branch at the same
    1e-4 rad, where the two sides differ by about 1e-33 rather than the 1.4e-7
    that a 0.1 rad radius produces.</p>
    <p><code>family</code> is not a difference. It is three exact identities of
    the rule-local Jacobian family, measured directly, so it stays sharp where a
    finite difference bottoms out at 1e-10.</p>
  </html>"),
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end So3;
