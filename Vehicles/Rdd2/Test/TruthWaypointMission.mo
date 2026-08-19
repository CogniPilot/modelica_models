within Vehicles.Rdd2.Test;

model TruthWaypointMission
  "RDD2 waypoint mission using plant truth as the controller baseline"
  extends WaypointMission(
    navigationSource = 0,
    enableSensorNoise = false);

  annotation(Documentation(info = "<html>
    <p>Flies the same local box as <code>WaypointMission</code>, but routes
    plant truth directly to controller feedback. This comparison baseline
    isolates the plant, planner, and controller from navigation-estimator
    behavior; the optical-flow and GPS qualification missions must still fly
    on their Kalman-filter estimates.</p>
  </html>"));
end TruthWaypointMission;
