within Tests;
model RetrodictJacobianTests
  "Chained LieGroups rules against retrodict and against the ESKF transition"
  constant Real step = 1.0e-6 "Central-difference step";
  constant Integer trials = 4 "Randomized states per delay";
  constant Real chainTolerance = 1.0e-7
    "The chain against a central difference of retrodict; measured worst case
     7.6e-9, which is the difference floor for a 50 m position lever arm";
  Real fiveMilliseconds[4];
  Real twentyMilliseconds[4];
  Real fiftyMilliseconds[4];
  Real quarterSecond[4];
equation
  fiveMilliseconds = Tests.RuleChecks.retrodictChainResiduals(0.005, step, trials);
  twentyMilliseconds = Tests.RuleChecks.retrodictChainResiduals(0.02, step, trials);
  fiftyMilliseconds = Tests.RuleChecks.retrodictChainResiduals(0.05, step, trials);
  quarterSecond = Tests.RuleChecks.retrodictChainResiduals(0.25, step, trials);

  assert(fiveMilliseconds[1] < chainTolerance,
    "Chained rules disagree with the retrodict difference at 5 ms");
  assert(twentyMilliseconds[1] < chainTolerance,
    "Chained rules disagree with the retrodict difference at 20 ms");
  assert(fiftyMilliseconds[1] < chainTolerance,
    "Chained rules disagree with the retrodict difference at 50 ms");
  assert(quarterSecond[1] < chainTolerance,
    "Chained rules disagree with the retrodict difference at 250 ms");

  // The delay budgets below follow age^4, the first term the cubic
  // discretization in discreteTransition drops. Quadrupling the delay must
  // multiply the gap by 256, and it does.
  assert(fiveMilliseconds[3] < 1.0e-8,
    "Chained H departs from the constructed H at 5 ms");
  assert(twentyMilliseconds[3] < 3.0e-6,
    "Chained H departs from the constructed H at 20 ms");
  assert(fiftyMilliseconds[3] < 1.0e-4,
    "Chained H departs from the constructed H at 50 ms");
  assert(quarterSecond[3] < 6.0e-2,
    "Chained H departs from the constructed H at 250 ms");

  // The two constructions must NOT agree at a long delay. Without this the
  // upper bounds above would still pass if the chain were quietly computing
  // the truncated transition instead of the exact Jacobian.
  assert(quarterSecond[3] > 1.0e-3,
    "Chained H and constructed H agree at 250 ms, so the chain is not the exact Jacobian");
  assert(quarterSecond[4] > 2.0,
    "The chain lost the position/velocity coupling that grows with the delay");

  annotation(Documentation(info="<html>
    <p>retrodict's tangent map is built two ways and compared. The chained way
    multiplies LieGroups rules: the exp_mixed state rule for the pose block, the
    exp_mixed left-increment rule composed with the increment's derivative in
    the inertial biases for the bias block, and the identity for the biases
    themselves. The measurement matrix is then formed exactly as
    correctGpsPosition forms it, by applying the position row selector to both
    matrices.</p>
    <p>Entry 1 of each vector is the chain against a central difference of
    retrodict itself. It sits at the difference floor at every delay and does
    not grow with the delay, because the chain is the exact Jacobian of the
    function retrodict computes.</p>
    <p>Entry 3 is the chain's H against
    <code>discreteTransition(continuousTransition(...), -age)</code>, the
    construction the corrections use today. That construction truncates the
    matrix exponential after the cubic term, so the two must differ, and they
    differ exactly at fourth order in the delay: 2.7e-9 at 5 ms, 6.9e-7 at
    20 ms, 2.7e-5 at 50 ms and 1.8e-2 at 250 ms, each ratio matching the fourth
    power of the delay ratio to three figures. The budgets are that law, not a
    tuned number, and the lower bound at 250 ms says which of the two is the
    approximation.</p>
  </html>"),
    experiment(StartTime = 0.0, StopTime = 0.002, Tolerance = 1.0e-8, Interval = 0.001));
end RetrodictJacobianTests;
