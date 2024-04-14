within Lie2.Test;

model TestSO3Dcm
  import Lie2.SO3;
  type G = SO3.Group.Dcm;
  type A = SO3.Algebra;
  G.Element g1, g2;
  SO3.Group.EulerB321.Element e1;
  Real normError;
  Real eqError;
initial equation
  g1 = G.fromMatrix(SO3.Group.EulerB321.toMatrix({0, 0, 0}));
equation
  der(g1) = G.Dr(g1, {1, 2, 3});
  e1 = SO3.Group.EulerB321.fromMatrix(G.toMatrix(g1));
  g2 = G.fromMatrix(G.toMatrix(g1));
  assert(G.equal(G.product(g1, G.inv(g1)), G.one()), "inverse");
  eqError = Math.norm2(G.log(G.product(g2, G.inv(g1))));
  normError = G.normError(g1);
  when (normError > 1e-4) then
    reinit(g1, G.normalize(g1));
  end when;
  when G.shouldSwitchCoordinates(g1) then
    reinit(g1, G.switchCoordinates(g1));
  end when;
  annotation(
    experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "rungekutta", variableFilter = ".*", cpu = "()"),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
end TestSO3Dcm;