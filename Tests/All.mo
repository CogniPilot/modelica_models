within Tests;
model All "Complete Modelica assertion suite"
  Tests.LinearAlgebraTests linearAlgebra;
  Tests.LieGroupsTests lieGroupsSmoke;
  Tests.LieGroupTests.Suite lieGroups;
  Tests.EstimationTests estimation;
  Tests.VerificationTests verification;
  Tests.PlanningTests planning;
  Tests.BezierTests bezier;
  Tests.DynamicsTests.Suite dynamics;
  annotation(experiment(StartTime=0.0, StopTime=1.0, Tolerance=1.0e-8, Interval=0.001));
end All;
