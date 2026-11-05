within Tests.LieGroupTests;
model Charts "Swappable SO(3) charts: contract for each chart plus independent reference values"
  constant Real tolerance = 1.0e-12;
  constant Real delta[3] = {0.3, -0.2, 0.25};
  // Reference values at delta from an independent forward-mode dual-number implementation of the same charts,
  // q = {w,x,y,z}.
  constant Real expQ[4] = {0.97603384598087684, 0.14879976670453762, -0.099199844469691767, 0.12399980558711469};
  constant Real cayleyQ[4] = {0.97677258101100872, 0.14651588715165129, -0.097677258101100881, 0.1220965726263761};
  constant Real mrpQ[4] = {0.97622356029025781, 0.14821676702176934, -0.098811178014512904, 0.12351397251814113};
  constant Real eulerQ[4] = {0.97429514924831639, 0.15983836472546739, -0.079404126906533229, 0.13746142711066392};
  constant Real expJ[3, 3] = [0.98308034213724416, 0.1131034243166955, 0.11078632888866337;
                              -0.13291180425358035, 0.97482685049687556, 0.13935564550179683;
                              -0.086025853967557311, -0.15586262878253418, 0.97854092173504137];
  constant Real cayleyJ[3, 3] = [0.95408467501490757, 0.11926058437686345, 0.095408467501490762;
                                 -0.11926058437686345, 0.95408467501490757, 0.14311270125223613;
                                 -0.095408467501490762, -0.14311270125223613, 0.95408467501490757];
  constant Real mrpJ[3, 3] = [0.97560210499121514, 0.11472287458223515, 0.10678990985048485;
                              -0.12936834793315879, 0.96949982442833027, 0.14035245294635151;
                              -0.088483068161830311, -0.15255701407212122, 0.9722458506816285];
  constant Real eulerJ[3, 3] = [1, 0, 0.19866933079506122;
                                0, 0.95533648912560598, 0.28962947762551555;
                                0, -0.29552020666133955, 0.93629336358419923];
  Tests.LieGroupTests.ChartContract exp(redeclare package Chart = LieGroups.SO3.Charts.Exp);
  Tests.LieGroupTests.ChartContract cayley(redeclare package Chart = LieGroups.SO3.Charts.Cayley);
  Tests.LieGroupTests.ChartContract mrp(redeclare package Chart = LieGroups.SO3.Charts.Mrp);
  Tests.LieGroupTests.ChartContract euler(redeclare package Chart = LieGroups.SO3.Charts.EulerB321);
  // Large angle (|delta| = 2.55, inside every symmetric chart's domain) and a near-singular Euler point.
  Tests.LieGroupTests.ChartContract expLarge(redeclare package Chart = LieGroups.SO3.Charts.Exp, delta = {1.5, -1.6, 1.3});
  Tests.LieGroupTests.ChartContract cayleyLarge(redeclare package Chart = LieGroups.SO3.Charts.Cayley, delta = {1.5, -1.6, 1.3});
  Tests.LieGroupTests.ChartContract mrpLarge(redeclare package Chart = LieGroups.SO3.Charts.Mrp, delta = {1.5, -1.6, 1.3});
  Tests.LieGroupTests.ChartContract eulerNearSingular(redeclare package Chart = LieGroups.SO3.Charts.EulerB321, delta = {0.2, 1.565, -0.4});
equation
  assert(abs(abs(exp.q * expQ) - 1.0) < tolerance and Tests.Assertions.maxAbsMatrix(exp.J - expJ) < tolerance,
    "Exp chart disagrees with reference values");
  assert(abs(abs(cayley.q * cayleyQ) - 1.0) < tolerance and Tests.Assertions.maxAbsMatrix(cayley.J - cayleyJ) < tolerance,
    "Cayley chart disagrees with reference values");
  assert(abs(abs(mrp.q * mrpQ) - 1.0) < tolerance and Tests.Assertions.maxAbsMatrix(mrp.J - mrpJ) < tolerance,
    "Mrp chart disagrees with reference values");
  assert(abs(abs(euler.q * eulerQ) - 1.0) < tolerance and Tests.Assertions.maxAbsMatrix(euler.J - eulerJ) < tolerance,
    "EulerB321 chart disagrees with reference values");
  assert(Tests.Assertions.maxAbsMatrix(exp.ADelta - cayley.ADelta) < 1.0e-7 and
         Tests.Assertions.maxAbsMatrix(exp.ADelta - mrp.ADelta) < 1.0e-7,
    "Symmetric charts must give the same linearization at the base");
  assert(not LieGroups.SO3.Charts.Exp.inDomain({2.0, -2.2, 1.6}),
    "Exp domain must exclude |delta| > pi, where the principal log cannot round-trip");
  assert(not LieGroups.SO3.Charts.EulerB321.inDomain({0.0, 2.0, 0.0}) and
         not LieGroups.SO3.Charts.EulerB321.inDomain({0.0, 6.0, 0.0}),
    "EulerB321 domain must exclude |pitch| > pi/2, including periodic cos(pitch) > 0 cases such as pitch = 6");
  assert(not LieGroups.SO3.Charts.Mrp.inDomain({5.0, 0.0, 0.0}) and
         not LieGroups.SO3.Charts.Mrp.inDomain({4.0, 0.0, 0.0}) and
         LieGroups.SO3.Charts.Mrp.inDomain({3.9, 0.0, 0.0}),
    "Mrp domain must be the open principal ball |delta| < 4");
  assert(Tests.Assertions.maxAbsVector(
      LieGroups.SO3.Charts.Examples.bodyAngularAcceleration({1.0, 1.5, 2.0}, {0.3, -0.2, 2.0}, zeros(3))
        - {0.2, 0.4, 0.015}) < tolerance,
    "Examples.bodyAngularAcceleration disagrees with Euler's equations");
end Charts;
