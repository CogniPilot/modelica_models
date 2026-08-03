within Vehicles.Rdd2.Qualification;
model GlobalWaypointMission
  "RDD2 waypoint mission authored in global latitude/longitude/altitude"
  extends WaypointMission(useGlobalWaypoints = true);
  annotation(Documentation(info="<html>
    <p>Runs <code>WaypointMission</code> in global mode: the box waypoints are
    interpreted as latitude/longitude/altitude and projected into the local
    East-North-Up frame through the mission origin before guidance. The flown
    local trajectory matches the local-mode mission, confirming the geodetic
    projection round-trip.</p>
  </html>"));
end GlobalWaypointMission;
