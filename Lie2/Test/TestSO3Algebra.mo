within Lie2.Test;

model TestSO3Algebra
  import Lie2.SO3;
  SO3.Algebra.Element a1 = {1, 2, 3};
  SO3.Algebra.Element a2 = {4, 5, 6};
  SO3.Algebra.Element a3;
equation
  a3 = SO3.Algebra.bracket(a1, a2);
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end TestSO3Algebra;