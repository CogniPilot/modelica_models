within CogniPilot.Vehicles.RDD2.Examples;
model Sim
  import CogniPilot.RDD2;
  inner RDD2.Components.World world;
  RDD2.Components.Vehicle rdd2;
equation
  annotation (
    experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.005),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "rungekuttaSsc", variableFilter = ".*"));
end Sim;