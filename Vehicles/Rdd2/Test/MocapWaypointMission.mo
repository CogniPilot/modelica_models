within Vehicles.Rdd2.Test;

model MocapWaypointMission
  "Motion-capture-navigated takeoff, box, and landing inside a rig volume"
  extends WaypointMission(
    navigationSource = 3,
    fuseMocap = true,
    // The rig origin sits at the mission datum, and the survey is exact. A
    // deliberately imperfect survey belongs to the handoff mission, where two
    // sources have to agree about a frame; here there is one source and the
    // question is whether it flies.
    mocapRigOriginWorldEnu_m = {0.0, 0.0, 0.0},
    mocapRigSurveyOffsetWorldEnu_m = {0.0, 0.0, 0.0},
    mocapRigSurveyHeadingError_rad = 0.0);
  annotation(Documentation(info="<html>
    <p>Runs the same 4 m box at 2 m that every other RDD2 waypoint
    qualification flies, on motion capture instead of GPS or optical flow, so
    the four estimator rows are comparable maneuvre for maneuvre. The box is
    already indoor scale, which is why the route is inherited rather than
    shrunk.</p>
    <p>The aiding set is the indoor-rig one and is chosen rather than
    inherited: motion capture and the IMU, with GPS, optical flow, the
    magnetometer and the barometer all off. The reasoning is on
    <code>navigationSource</code>; the short form is that a 6-dof pose observes
    yaw better than a magnetometer can in a hall full of steel, and absolute
    altitude better than an indoor pressure datum.</p>
    <p>Poses arrive 20 ms old through
    <code>Vehicles.Rdd2.mocapRigToWorld</code>, so this mission flies on the
    age-alignment stanza in <code>correctMocap</code> rather than merely
    containing it.</p>
    </html>"));
end MocapWaypointMission;
