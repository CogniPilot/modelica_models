within SLAM;

package LoopClosure
  "Keyframe selection, retrieval and geometric loop verification"
  annotation(Documentation(info = "<html>
    <h4>Retrieve, then verify</h4>
<p>Keyframes retain descriptors, calibrated geometry and measured image identity.
<a href=\"modelica://SLAM.LoopClosure.RGBDKeyframePolicy\">RGBDKeyframePolicy</a>
controls capture; <a href=\"modelica://SLAM.LoopClosure.RGBDKeyframeRetrieval\">RGBDKeyframeRetrieval</a>
finds candidates through visual words; and
<a href=\"modelica://SLAM.LoopClosure.RGBDLoopVerification\">RGBDLoopVerification</a>
checks geometry before a loop is accepted.</p>
<p>A descriptor similarity is a retrieval score, not a pose constraint. Graph
edges require measured relative geometry and its uncertainty. Preserve frame
generations and identities so a reused storage slot cannot masquerade as an
old observation. Integration into a particular application must explicitly
connect capture, retrieval, verification and graph insertion.</p>
    </html>"));
end LoopClosure;
