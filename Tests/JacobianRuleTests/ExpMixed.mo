within Tests.JacobianRuleTests;
model ExpMixed "exp_mixed rules against central differences over a dense magnitude sweep"
  constant Real step = 1.0e-6 "Central-difference step";
  constant Integer trials = 4 "Randomized points per magnitude";
  constant Real coupling = 0.05 "The dt entry of the nilpotent coupling block";
  constant Real shortLeverArm = 50.0 "Metres of position the nominal sample carries";
  constant Real longLeverArm = 200.0
    "Metres of position the long-arm sample carries. The right-increment rule
     multiplies the SO(3) right Jacobian by the accumulated position block, so
     any error in that Jacobian is amplified by this arm and the rule must be
     checked at an arm large enough to expose it";
  constant Real magnitudes[24] = {
    1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 0.01, 0.03, 0.05, 0.07,
    0.08, 0.085, 0.09, 0.092, 0.095, 0.097, 0.099, 0.0999,
    0.1001, 0.102, 0.105, 0.11, 0.3, 1.0, 1.5, 3.0}
    "Dense from 1e-6 to 3 rad, bracketing 0.1 on both sides. |omega| = 0.1
     itself is left out and only there: exp_mixed's own coefficients switch
     between series and closed form at that radius and the two sides differ by
     1.4e-7 in the leading coefficient, so a difference whose 1e-6 step
     straddles it divides that jump by 2e-6 and measures its own branch";
  // A binary64 central difference of a quantity of magnitude M rounds at about
  // eps*M/h, so these residuals have a floor proportional to the lever arm:
  // 1.1e-8 at 50 m and 4.4e-8 at 200 m for h = 1e-6. Measured over the whole
  // sweep the worst rule sits at 1.5e-8 and 6.0e-8, within a factor of 1.4 of
  // that model. Each budget is ten times the modelled floor, so it is about
  // seven times what the rules measure: enough that the assertion is about the
  // rule rather than the last bits of its oracle, and tight enough that a
  // rotation Jacobian carrying the 0.1 rad series radius, which measures
  // 6.4e-7 and 2.6e-6 at these two arms, fails it by a factor of six.
  constant Real floorPerMetre = 2.2e-16 / step "Difference floor per metre of lever arm";
  constant Real shortTolerance = 10.0 * floorPerMetre * shortLeverArm;
  constant Real longTolerance = 10.0 * floorPerMetre * longLeverArm;
  constant Real blockTolerance = 1.0e-12
    "The increment block is not a difference: it reproduces exp_mixed exactly";
  Real shortArm[4] "Worst over the magnitude list at a 50 m lever arm";
  Real longArm[4] "Worst over the magnitude list at a 200 m lever arm";
  Real branchShort[4] "0.099 rad at 50 m";
  Real branchLong[4] "0.099 rad at 200 m";
equation
  shortArm = Tests.RuleChecks.expMixedRuleResidualsSweep(
    magnitudes, shortLeverArm, coupling, step, trials);
  longArm = Tests.RuleChecks.expMixedRuleResidualsSweep(
    magnitudes, longLeverArm, coupling, step, trials);
  branchShort = Tests.RuleChecks.expMixedRuleResiduals(
    0.099, shortLeverArm, coupling, step, trials);
  branchLong = Tests.RuleChecks.expMixedRuleResiduals(
    0.099, longLeverArm, coupling, step, trials);

  assert(shortArm[1] < shortTolerance,
    "exp_mixed_state_jacobian disagrees with a central difference at a 50 m lever arm");
  assert(shortArm[2] < shortTolerance,
    "exp_mixed_left_increment_jacobian disagrees with a central difference at a 50 m lever arm");
  assert(shortArm[3] < shortTolerance,
    "exp_mixed_right_increment_jacobian disagrees with a central difference at a 50 m lever arm");
  assert(shortArm[4] < blockTolerance,
    "mixed_increment_matrix no longer reproduces exp_mixed's own increment block");

  assert(longArm[1] < longTolerance,
    "exp_mixed_state_jacobian disagrees with a central difference at a 200 m lever arm");
  assert(longArm[2] < longTolerance,
    "exp_mixed_left_increment_jacobian disagrees with a central difference at a 200 m lever arm");
  assert(longArm[3] < longTolerance,
    "exp_mixed_right_increment_jacobian disagrees with a central difference at a 200 m lever arm");
  assert(longArm[4] < blockTolerance,
    "mixed_increment_matrix no longer reproduces exp_mixed's own increment block at a 200 m lever arm");

  // 0.099 rad is inside the sweep and is recorded separately because it is the
  // magnitude at which a rotation Jacobian carrying the 0.1 rad series radius
  // reaches |omega|^5/720 and the lever arm turns that into a failure: 6.4e-7
  // at 50 m and 2.6e-6 at 200 m.
  assert(branchShort[3] < shortTolerance,
    "exp_mixed_right_increment_jacobian disagrees at 0.099 rad and a 50 m lever arm");
  assert(branchLong[3] < longTolerance,
    "exp_mixed_right_increment_jacobian disagrees at 0.099 rad and a 200 m lever arm");

  annotation(Documentation(info="<html>
    <p>Entries 1 to 3 are the rules for the initial-state, left-increment and
    right-increment slots of <code>exp_mixed</code>. Entry 4 is not a
    difference: it is the increment block <code>mixed_increment_matrix</code>
    evaluated against <code>exp_mixed</code> itself at the identity element with
    a zero right increment, where the group product reduces to the block
    exactly. It pins the factored block the rules are written in against the
    primitive they claim to differentiate, and it measures zero.</p>
    <p>Two lever arms rather than one. The right-increment rule transports the
    accumulated position block R_0 N_l + P_0 M through a cross product, so it
    multiplies the SO(3) right Jacobian by the arm; the difference floor is
    proportional to the arm as well, but only linearly, so a rotation-Jacobian
    error and the floor stay separable and the long arm is where an error shows
    first. The budgets are each ten times their own modelled floor, about seven
    times what the rules measure.</p>
  </html>"),
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end ExpMixed;
