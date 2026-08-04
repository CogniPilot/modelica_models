within Vehicles.Rdd2.Test;

model WaypointMission
  "Takeoff, box, and landing mission in the local East-North-Up frame"
  parameter Real cruiseAltitude_m = 2.0;
  parameter Real boxSide_m = 4.0;

  extends Vehicles.Rdd2.WaypointVehicleSystem(
    useGlobalWaypoints = false,
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
      <p>Defines only the local box route and experiment. Plant, task periods,
      signal routing, planning, guidance, rate control, and allocation belong
      to the reusable <code>Vehicles.Rdd2.WaypointVehicleSystem</code> stack.</p>
    </html>"));
end WaypointMission;
