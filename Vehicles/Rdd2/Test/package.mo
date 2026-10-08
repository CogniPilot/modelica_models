within Vehicles.Rdd2;
package Test "RDD2 model-level test missions"
  annotation(Documentation(info = "<html>
    <p><a href=\"modelica://Vehicles.Rdd2.Test.TruthWaypointMission\">TruthWaypointMission</a>
    is the truth-feedback baseline. Compare it with
    <a href=\"modelica://Vehicles.Rdd2.Test.WaypointMission\">WaypointMission</a>
    for optical-flow-aided ESKF navigation and
    <a href=\"modelica://Vehicles.Rdd2.Test.GlobalWaypointMission\">GlobalWaypointMission</a>
    for GPS-aided geodetic waypoints. The corresponding UKF missions use the
    same controller and plant assumptions.</p>
    <h4>GPS-to-Mocap handoff</h4>
    <p>Paired surveyed and ideal missions share noise and coverage timing.
    The surveyed rig offset is 5 cm east and 3 cm north. Compare estimate
    minus truth at matching ticks: errors must agree before coverage and
    recover that vector during both the first 0.5 seconds and remaining
    coverage. The residual budget is 3 sqrt(2) times the per-axis 1 cm Mocap
    noise. It does not shrink with sample count because errors are correlated.
    Missing windows, mismatched ticks and non-finite errors fail.</p>
    <p>These are deterministic simulation checks, not independent-sample
    confidence intervals or a substitute for flight qualification.</p>
    </html>"));
end Test;
