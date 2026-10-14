within Vision;

package Features
  "Image feature detection with bounded output capacity"
  annotation(Documentation(info = "<html>
    <h4>Acquire bounded feature sets</h4>
<p><a href=\"modelica://Vision.Features.D435FastFeatures\">D435FastFeatures</a>
and <a href=\"modelica://Vision.Features.D435HarrisFeatures\">D435HarrisFeatures</a>
combine image scoring with bounded feature selection. Lower-level score and
selection functions support alternative front ends without changing downstream
geometry.</p>
<p>Choose image extents and feature capacity before translation. Preserve the
pixel convention and enabled mask when passing sparse selections to
<a href=\"modelica://Vision.Matching\">Matching</a>. A detector score describes
image structure; it is not a calibrated position covariance or proof that a
feature has usable depth.</p>
    </html>"));
end Features;
