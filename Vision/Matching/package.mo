within Vision;

package Matching
  "RGB-D descriptors, matching and patch tracking"
  annotation(Documentation(info = "<html>
    <h4>Calibrated RGB-D correspondences</h4>
<p><a href=\"modelica://Vision.Matching.DescribeRGBDFrame\">DescribeRGBDFrame</a>
builds descriptors and
<a href=\"modelica://Vision.Matching.MatchRGBDDescriptors\">MatchRGBDDescriptors</a>
finds candidate correspondences. Keep feature identity, enabled status, image
size and RGB/depth calibrations with the descriptor arrays.</p>
<p>Matching appearance does not establish a rigid transformation. Qualify depth
and geometric residuals before registration. A separate depth calibration does
not imply that the depth image is already registered to the RGB image; the
caller must supply the actual alignment and extrinsics.</p>
<p><a href=\"modelica://Vision.Matching.RGBDPatchTracking\">RGBDPatchTracking</a>
is a separate photometric primitive. Its existence does not mean the full
application selects it automatically.</p>
    </html>"));
end Matching;
