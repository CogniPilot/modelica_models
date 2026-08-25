within Vehicles.Rdd2.Test;

model UkfGlobalWaypointMission
  "GPS waypoint mission using the StrapdownINS manifold UKF"
  parameter Real cruiseAltitude_m = 2.0;
  parameter Real boxSide_m = 4.0;
  extends Vehicles.Rdd2.WaypointVehicleSystem(
    redeclare block EstimatorModel = Estimation.StrapdownINS.UKF.Estimator,
    useGlobalWaypoints=true,
    navigationSource=1,
    estimatorInitialPositionWorldEnu_m={0.08, -0.06, 0.04},
    localRoute=[
      0.0,       0.0,       0.0;
      0.0,       0.0,       cruiseAltitude_m;
      boxSide_m, 0.0,       cruiseAltitude_m;
      boxSide_m, boxSide_m, cruiseAltitude_m;
      0.0,       boxSide_m, cruiseAltitude_m;
      0.0,       0.0,       cruiseAltitude_m;
      0.0,       0.0,       0.3;
      0.0,       0.0,       0.1]);
  annotation(experiment(StartTime=0.0, StopTime=45.0,
    Tolerance=1.0e-8, Interval=0.005));
end UkfGlobalWaypointMission;
