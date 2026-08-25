within Vehicles.Rdd2.Test;
model UnfilteredVibrationGlobalWaypointMission
  "Vibration mission flown without the sensor anti-alias low-pass"
  extends VibrationGlobalWaypointMission(imuAntiAliasFilter = false);
  annotation(Documentation(info = "<html>
    <p>Companion to <code>VibrationGlobalWaypointMission</code> with the
    ICM-45686 user-interface filter removed, which is what the plant modelled
    before the filter existed. Comparing the two traces separates the
    navigation cost of the vibration itself from the cost of point sampling
    unattenuated rotor content.</p>
  </html>"));
end UnfilteredVibrationGlobalWaypointMission;
