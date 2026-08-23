within Tests.JacobianRuleTests;
model So3 "SO(3) rules against central differences over five magnitudes"
  constant Real step = 1.0e-6
    "Central-difference step: the O(h^2) truncation and the O(eps/h) rounding
     of a binary64 central difference cross near 1e-6 for quantities of order one";
  constant Integer trials = 6 "Randomized points per magnitude";
  constant Real tolerance = 1.0e-8
    "Above the 2e-10 difference floor these rules were measured at, below any
     error a wrong closed form could hide in";
  Real tiny[7] "1e-6 rad";
  Real small[7] "1e-3 rad";
  Real series[7] "0.05 rad, inside the retained-series branch";
  Real moderate[7] "0.3 rad";
  Real large[7] "2.0 rad";
equation
  tiny = Tests.RuleChecks.so3RuleResiduals(1.0e-6, step, trials);
  small = Tests.RuleChecks.so3RuleResiduals(1.0e-3, step, trials);
  series = Tests.RuleChecks.so3RuleResiduals(0.05, step, trials);
  moderate = Tests.RuleChecks.so3RuleResiduals(0.3, step, trials);
  large = Tests.RuleChecks.so3RuleResiduals(2.0, step, trials);

  assert(Tests.Assertions.maxAbsVector(tiny) < tolerance,
    "SO(3) rules disagree with central differences at 1e-6 rad");
  assert(Tests.Assertions.maxAbsVector(small) < tolerance,
    "SO(3) rules disagree with central differences at 1e-3 rad");
  assert(Tests.Assertions.maxAbsVector(series) < tolerance,
    "SO(3) rules disagree with central differences at 0.05 rad");
  assert(Tests.Assertions.maxAbsVector(moderate) < tolerance,
    "SO(3) rules disagree with central differences at 0.3 rad");
  assert(Tests.Assertions.maxAbsVector(large) < tolerance,
    "SO(3) rules disagree with central differences at 2.0 rad");
  annotation(Documentation(info="<html>
    <p>Each vector entry is one rule: exp_map, log_map, product in its left
    factor, product in its right factor, inverse, rotate in its rotation slot,
    and rotate in its vector slot.</p>
  </html>"),
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end So3;
