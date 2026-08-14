within Vehicles.Rdd2.Test;
model GlobalWaypointMission
  "GPS-navigated RDD2 waypoint mission authored in global latitude/longitude/altitude"
  extends WaypointMission(
    useGlobalWaypoints = true,
    navigationSource = 1);
  annotation(Documentation(info="<html>
    <p>Runs <code>WaypointMission</code> in global mode: the box waypoints are
    interpreted as latitude/longitude/altitude and projected into the local
    East-North-Up frame through the mission origin before guidance. Guidance
    flies on the GPS-aided inertial solution
    (<code>navigationSource&nbsp;=&nbsp;1</code>), so the mission exercises the
    geodetic projection round-trip and GPS position/velocity fusion together.
    The flown local trajectory must match the optical-flow local-mode mission
    to within the navigation-modality tolerance.</p>
  </html>"));
end GlobalWaypointMission;
