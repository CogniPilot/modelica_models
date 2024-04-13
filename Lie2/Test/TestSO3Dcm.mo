within Lie2.Test;

model TestSO3Dcm
  import Lie2.SO3;
  type G = SO3.Dcm;
  G.Element g1;
  SO3.EulerB321.Element e1;
  Real normError;
  constant Boolean test_product = true;
  constant Boolean test_matrix = true;
initial equation
  g1 = G.fromMatrix(SO3.EulerB321.toMatrix({0, Constants.pi/2, 0}));
equation
  der(g1) = G.Dr(g1, {1, 2, 3});
  e1 = SO3.EulerB321.fromMatrix(G.toMatrix(g1));
  if test_product then
    assert(G.equal(G.product(g1, G.inv(g1)), G.one()), "inverse test failed");
  end if;
  if test_matrix then
    assert(G.equal(G.fromMatrix(G.toMatrix(g1)), g1), "from/to matrix failed");
  end if;
  normError = G.normError(g1);
  when (normError > 1e-4) then
    reinit(g1, G.normalize(g1));
  end when;
annotation(
    experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-6, Interval = 0.01),
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "rungekutta", variableFilter = ".*", cpu = "()"),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
end TestSO3Dcm;