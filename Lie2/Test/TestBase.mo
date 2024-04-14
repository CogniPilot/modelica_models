within Lie2.Test;

model TestBase
  import Lie2;
  Lie2.Base.Group g;
  Lie2.Base.Algebra a;
equation

annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end TestBase;