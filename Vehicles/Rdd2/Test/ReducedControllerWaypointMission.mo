within Vehicles.Rdd2.Test;

model ReducedControllerWaypointMission
  "Optical-flow box mission flown by the reduced-authority controller variant"
  parameter Real cruiseAltitude_m = 2.0;
  parameter Real boxSide_m = 4.0;

  extends Vehicles.Rdd2.WaypointVehicleSystem(
    redeclare block ControllerModel = Vehicles.Rdd2.ReducedRateController,
    useGlobalWaypoints = false,
    navigationSource = 2,
    estimatorInitialPositionWorldEnu_m = {0.08, -0.06, 0.04},
    localRoute = [
      0.0,       0.0,       0.0;
      0.0,       0.0,       cruiseAltitude_m;
      boxSide_m, 0.0,       cruiseAltitude_m;
      boxSide_m, boxSide_m, cruiseAltitude_m;
      0.0,       boxSide_m, cruiseAltitude_m;
      0.0,       0.0,       cruiseAltitude_m;
      0.0,       0.0,       0.3;
      0.0,       0.0,       0.1]);

  annotation(
    experiment(
      StartTime = 0.0,
      StopTime = 45.0,
      Tolerance = 1.0e-8,
      Interval = 0.005),
    Documentation(info = "<html>
      <p>Flies the same optical-flow box route as
      <code>WaypointMission</code>, but swaps the flight controller with a
      single <code>redeclare block ControllerModel</code> line. Comparing this
      mission against <code>WaypointMission</code> isolates the effect of the
      reduced-authority rate loop under an identical plant, estimator, plan, and
      noise realization, which is the controller-side twin of the ESKF-versus-UKF
      estimator comparison (<code>WaypointMission</code> versus
      <code>UkfWaypointMission</code>).</p>
    </html>"));
end ReducedControllerWaypointMission;
