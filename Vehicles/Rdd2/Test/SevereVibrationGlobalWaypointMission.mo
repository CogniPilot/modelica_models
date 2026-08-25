within Vehicles.Rdd2.Test;
model SevereVibrationGlobalWaypointMission
  "Vibration mission flown with a worn or badly unbalanced rotor set"
  extends VibrationGlobalWaypointMission(
    vibrationAngularAmplitude_rad_s = 0.15,
    vibrationSpecificForceAmplitude_m_s2 = 3.0);
  annotation(Documentation(info = "<html>
    <p>Three times the nominal vibration amplitude, which is the level a
    worn or badly balanced rotor set reaches in flight logs. Because the
    unbalance amplitude law is quadratic in rotor speed and the rectified
    coning rate is quadratic in amplitude, this case carries roughly an
    order of magnitude more rectified content than the nominal mission and
    is the closed-loop point at which an accumulation shortfall would first
    become visible.</p>
  </html>"));
end SevereVibrationGlobalWaypointMission;
