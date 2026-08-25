within Vehicles.Rdd2.Test;

model WaypointMission
  "Optical-flow-navigated takeoff, box, and landing mission in the local East-North-Up frame"
  parameter Real cruiseAltitude_m = 2.0;
  parameter Real boxSide_m = 4.0;

  extends Vehicles.Rdd2.WaypointVehicleSystem(
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
      <p>Defines only the local box route and experiment. Plant, task periods,
      signal routing, planning, guidance, rate control, and allocation belong
      to the reusable <code>Vehicles.Rdd2.WaypointVehicleSystem</code> stack.</p>
      <p>Guidance flies on the optical-flow-aided inertial solution
      (<code>navigationSource&nbsp;=&nbsp;2</code>): position is dead-reckoned
      from the IMU with planar body-velocity aiding, matching a GPS-denied
      indoor flight over a textured floor. The small deterministic initial
      position error is a qualification witness: controller feedback must
      follow the Kalman estimate rather than coincident plant truth.</p>
    </html>"));
end WaypointMission;
