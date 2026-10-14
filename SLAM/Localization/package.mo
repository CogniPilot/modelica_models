within SLAM;

package Localization
  "Calibrated relative RGB-D localization and navigation interfaces"
  annotation(Documentation(info = "<html>
    <h4>From image pairs to pose observations</h4>
<p>This layer joins feature processing and calibrated registration into visual
observations. Start with
<a href=\"modelica://SLAM.Localization.ObserveRGBDRelativeFrame\">ObserveRGBDRelativeFrame</a>
for relative-frame processing and
<a href=\"modelica://SLAM.Localization.RGBDInertialLocalizationInterface\">RGBDInertialLocalizationInterface</a>
for the persistent localization boundary.</p>
<p><a href=\"modelica://SLAM.Localization.RGBDRegistrationUncertainty\">RGBDRegistrationUncertainty</a>
propagates point noise around the accepted fit. Its covariance is conditional
on the correspondences and reference assumptions; it does not automatically
include map uncertainty or shared-IMU correlation. Check validity and rejection
outputs before forwarding an observation to a filter.</p>
<p>Optical coordinates, body FLU coordinates and world ENU coordinates are
separate frames. Preserve calibration, camera extrinsics, capture time and
reference identity when adapting an observation.</p>
    </html>"));
end Localization;
