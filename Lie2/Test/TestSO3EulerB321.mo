within Lie2.Test;

model TestSO3EulerB321
  extends TestSO3Dcm(redeclare type G = SO3.Group.EulerB321);
  annotation(
    experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(cpu = "()", lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "rungekutta", variableFilter = ".*"));
end TestSO3EulerB321;