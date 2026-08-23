within Tests.JacobianRuleTests;
model ExpMixed "exp_mixed rules against central differences over five magnitudes"
  constant Real step = 1.0e-6 "Central-difference step";
  constant Integer trials = 4 "Randomized points per magnitude";
  constant Real positionScale = 50.0 "Metres of position the sampled state carries";
  constant Real coupling = 0.05 "The dt entry of the nilpotent coupling block";
  constant Real tolerance = 1.0e-6
    "These rules carry the 50 m position lever arm, so the difference floor is
     about 1e-8; the worst measured case, 2.7e-8, is the SO(3) right Jacobian's
     own two-term series multiplied by that lever arm";
  Real tiny[4] "1e-6 rad";
  Real small[4] "1e-3 rad";
  Real series[4] "0.05 rad, inside the retained-series branch";
  Real moderate[4] "0.3 rad";
  Real large[4] "1.5 rad";
equation
  tiny = Tests.RuleChecks.expMixedRuleResiduals(
    1.0e-6, positionScale, coupling, step, trials);
  small = Tests.RuleChecks.expMixedRuleResiduals(
    1.0e-3, positionScale, coupling, step, trials);
  series = Tests.RuleChecks.expMixedRuleResiduals(
    0.05, positionScale, coupling, step, trials);
  moderate = Tests.RuleChecks.expMixedRuleResiduals(
    0.3, positionScale, coupling, step, trials);
  large = Tests.RuleChecks.expMixedRuleResiduals(
    1.5, positionScale, coupling, step, trials);

  assert(Tests.Assertions.maxAbsVector(tiny) < tolerance,
    "exp_mixed rules disagree with central differences at 1e-6 rad");
  assert(Tests.Assertions.maxAbsVector(small) < tolerance,
    "exp_mixed rules disagree with central differences at 1e-3 rad");
  assert(Tests.Assertions.maxAbsVector(series) < tolerance,
    "exp_mixed rules disagree with central differences at 0.05 rad");
  assert(Tests.Assertions.maxAbsVector(moderate) < tolerance,
    "exp_mixed rules disagree with central differences at 0.3 rad");
  assert(Tests.Assertions.maxAbsVector(large) < tolerance,
    "exp_mixed rules disagree with central differences at 1.5 rad");
  annotation(Documentation(info="<html>
    <p>Entries 1 to 3 are the rules for the initial-state, left-increment and
    right-increment slots of <code>exp_mixed</code>. Entry 4 is not a
    difference: it is the increment block <code>mixed_increment_matrix</code>
    evaluated against <code>exp_mixed</code> itself at the identity element with
    a zero right increment, where the group product reduces to the block
    exactly. It pins the factored block the rules are written in against the
    primitive they claim to differentiate, and it measures zero.</p>
  </html>"),
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end ExpMixed;
