within Tests.JacobianRuleTests;
model Suite "Every derivative-rule family in one model"
  Tests.JacobianRuleTests.So3 so3;
  Tests.JacobianRuleTests.Se23Group se23Group;
  Tests.JacobianRuleTests.ExpMixed expMixed;
  annotation(
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end Suite;
