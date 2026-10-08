within SLAM;

package Inertial
  "Inertial propagation and Schmidt relative-pose correction"
  annotation(Documentation(info = "<html>
    <h4>Visual-inertial reference</h4>
<p>This package contains the world-additive ES15 and retained-reference Schmidt
operations used by the visual pipeline.
<a href=\"modelica://SLAM.Inertial.ModelicaInertial\">ModelicaInertial</a>
is the nominal inertial model;
<a href=\"modelica://SLAM.Inertial.SchmidtCaptureReference\">SchmidtCaptureReference</a>
and <a href=\"modelica://SLAM.Inertial.SchmidtCorrectRelativePose\">SchmidtCorrectRelativePose</a>
manage the retained reference and relative observations.</p>
<p>The navigation ESKF under
<a href=\"modelica://Estimation.StrapdownINS.ESKF\">Estimation.StrapdownINS.ESKF</a>
uses a different tangent/injection convention. Do not copy a covariance between
these filters as if its coordinate ordering and frame were interchangeable.
Retaining a reference requires its uncertainty and cross-correlation with the
current state, not just its saved pose.</p>
    </html>"));
end Inertial;
