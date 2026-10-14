within Vision;

package Registration
  "Rigid point-pair registration and geometric checks"
  annotation(Documentation(info = "<html>
    <h4>Fit a rigid transformation</h4>
<p><a href=\"modelica://Vision.Registration.FitRigidPointPairs\">FitRigidPointPairs</a>
fits source/target point pairs; the
<a href=\"modelica://Vision.Registration.RigidPointRegistration\">RigidPointRegistration</a>
model exposes the same fit with array inputs and diagnostics.
<a href=\"modelica://Vision.Registration.FitRigidPointPairsRobust\">FitRigidPointPairsRobust</a>
adds robust pair handling.</p>
<p>Supply points in declared coordinates, active-count bounds and explicit
pair masks. Check acceptance, rejection reason, rank and residual before using
the rotation and translation. A small RMS with insufficient geometric rank is
not a fully constrained pose. Registration estimates geometry; observation
uncertainty is handled by the
<a href=\"modelica://SLAM.Localization.RGBDRegistrationUncertainty\">localization layer</a>.</p>
    </html>"));
end Registration;
