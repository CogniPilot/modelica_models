within Vehicles.Rdd2.Test;
model VibrationGlobalWaypointMission
  "GPS-navigated RDD2 waypoint mission flown with rotor vibration on the IMU"
  parameter Real vibrationAngularAmplitude_rad_s(unit = "rad/s") = 0.05
    "Per-rotor angular-rate vibration amplitude at hover trim";
  parameter Real vibrationSpecificForceAmplitude_m_s2(unit = "m/s2") = 1.0
    "Per-rotor lateral specific-force vibration amplitude at hover trim";
  parameter Boolean imuAntiAliasFilter = true
    "Run the ICM-45686 user-interface low-pass ahead of IMU sampling";
  extends GlobalWaypointMission(plant(
    enableRotorVibration = true,
    rotorVibrationAngular_rad_s = vibrationAngularAmplitude_rad_s,
    rotorVibrationSpecificForce_m_s2 = vibrationSpecificForceAmplitude_m_s2,
    enableImuAntiAliasFilter = imuAntiAliasFilter));
  annotation(Documentation(info = "<html>
    <p>Runs <code>GlobalWaypointMission</code> with the rotor unbalance and
    blade-pass disturbance switched on. Everything else, including the
    waypoints, task periods, sensor noise seed, and estimator, is shared with
    the vibration-free mission, so the difference between the two traces is
    the navigation cost of vibration alone.</p>
  </html>"));
end VibrationGlobalWaypointMission;
