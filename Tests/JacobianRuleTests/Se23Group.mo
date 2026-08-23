within Tests.JacobianRuleTests;
model Se23Group "SE_2(3) product and inverse rules against central differences"
  constant Real step = 1.0e-6 "Central-difference step";
  constant Integer trials = 6 "Randomized points";
  constant Real tolerance = 1.0e-6
    "These rules carry a 50 m position lever arm, so the difference floor is
     eps*50/h, about 1e-8; measured worst case 1.2e-8";
  Real worst[3] "product left factor, product right factor, inverse";
equation
  worst = Tests.RuleChecks.se23GroupRuleResiduals(50.0, step, trials);
  assert(Tests.Assertions.maxAbsVector(worst) < tolerance,
    "SE_2(3) group rules disagree with central differences");
  annotation(Documentation(info="<html>
    <p>These three rules are what gives <code>adjoint</code> its operational
    meaning: the left-factor rule is Ad(X2^-1), the right-factor rule is the
    identity, and the inverse rule is -Ad(X). All three are exact group
    identities rather than first-order approximations, so the only error the
    difference can show is its own.</p>
  </html>"),
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end Se23Group;
