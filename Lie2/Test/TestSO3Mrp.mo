within Lie2.Test;

model TestSO3Mrp
  extends TestSO3Dcm(redeclare type G = SO3.Mrp);
  annotation(
    experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 0.01),
    __OpenModelica_simulationFlags(cpu = "()", lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "rungekutta", variableFilter = ".*"));
end TestSO3Mrp;