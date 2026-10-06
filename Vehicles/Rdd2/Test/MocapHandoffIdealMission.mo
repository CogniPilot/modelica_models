within Vehicles.Rdd2.Test;

model MocapHandoffIdealMission
  "The same two-way handoff with a perfect rig survey"
  extends MocapHandoffMission(
    mocapRigSurveyOffsetWorldEnu_m = {0.0, 0.0, 0.0},
    mocapRigSurveyHeadingError_rad = 0.0);
  annotation(Documentation(info="<html>
    <p>The degenerate control. With an exact survey the rig-to-world
    composition is the identity, so the two sources agree about the frame and
    the only thing that changes at a crossing is which noise realization is
    driving the correction.</p>
    <p><b>The honest claim here is NOT that the handoff is bitwise smooth</b>,
    and deriving which claim is true matters more than picking the tidier one.
    The two sources carry independent noise draws and different measurement
    covariances, so the innovation stream changes at the crossing and the
    estimate does not continue bit for bit. What is bounded is the step: with
    no frame disagreement the innovation is the arriving source's own noise
    against the departing source's converged error, so the step falls to the
    measurement-noise class rather than to floating-point epsilon. Gating it at
    epsilon would be gating a claim that is false.</p>
    <p>What the pair buys is discrimination. Both runs must agree before
    coverage. Their estimate-minus-truth errors must then differ by the
    configured survey vector at entry and throughout coverage, within the
    paired mocap noise budget. A missing, reversed, or doubled survey offset
    fails this comparison even when an unrelated GPS correction produces a
    larger single-tick step. Each mission also keeps its crossing-step bound
    and phase-specific aiding and consistency checks.</p>
    </html>"));
end MocapHandoffIdealMission;
