within Vehicles.Rdd2.Test;

model MocapHandoffMission
  "Fly GPS, cross into rig coverage, manoeuvre, and cross back out"
  extends WaypointMission(
    navigationSource = 1,
    useGlobalWaypoints = false,
    mocapCoverageWindow = true,
    // Coverage opens after the climb has settled and closes before the
    // descent, so both crossings happen in level cruise where a step in the
    // estimate would show as a tracking transient rather than hiding inside
    // an altitude change.
    mocapCoverageStart_s = 15.0,
    mocapCoverageEnd_s = 32.0,
    mocapRigOriginWorldEnu_m = {2.0, 2.0, 0.0},
    // THE SURVEY IS DELIBERATELY WRONG, by 5 cm east and 3 cm north. Two
    // sources that disagree about where the world's origin is disagree about
    // where the vehicle is by exactly that much, which is what makes the
    // handoff a test rather than a formality. It is small enough to sit well
    // inside the innovation gate, so the honest outcome is a smooth
    // correction and not a refusal.
    mocapRigSurveyOffsetWorldEnu_m = {0.05, 0.03, 0.0},
    mocapRigSurveyHeadingError_rad = 0.0);
  annotation(Documentation(info="<html>
    <p>The two-way handoff. Guidance flies on the GPS-aided estimate
    throughout; what changes mid-mission is which source is correcting it.
    Inside the coverage window motion capture aids and GPS is withheld, and
    outside it the reverse, so the mission crosses the boundary in both
    directions.</p>
    <p><b>Nothing re-anchors.</b> Both sources correct the same world state,
    each through its own transform into it: GPS through the
    <code>Geodesy.GeodeticOrigin</code> datum the global routes already use,
    motion capture through the surveyed rig placement. The 5 cm survey error
    therefore arrives as an innovation on the first mocap correction after the
    crossing and is corrected through the gate like any other disagreement. It
    is never absorbed by moving the state to meet the new source, which would
    hide a frame error as a state jump.</p>
    </html>"));
end MocapHandoffMission;
