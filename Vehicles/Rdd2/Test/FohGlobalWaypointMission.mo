within Vehicles.Rdd2.Test;
model FohGlobalWaypointMission
  "GPS-navigated waypoint mission with first-order-hold IMU composition"
  extends GlobalWaypointMission(useFirstOrderHoldImu = true);
  annotation(Documentation(info="<html>
    <p>Runs <code>GlobalWaypointMission</code> with the raw 1 kHz IMU
    intervals composed under a first-order hold: the driver-side
    preintegrator adds the coning, sculling, and scrolling cross terms of
    the truncated Magnus exponent and the matching bias-Jacobian cross
    terms. Compare against <code>GlobalWaypointMission</code>, which keeps
    the zero-order hold, to measure what the hold model changes on a full
    mission.</p>
  </html>"));
end FohGlobalWaypointMission;
