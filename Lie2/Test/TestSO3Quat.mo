within Lie2.Test;

model TestSO3Quat
  extends TestSO3Dcm(redeclare type G = SO3.Quat);
  annotation(
    experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(cpu = "()", lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "rungekutta", variableFilter = ".*"));
end TestSO3Quat;