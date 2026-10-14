within SLAM;

package Mapping
  "Landmark projection, identity and persistent map storage"
  annotation(Documentation(info = "<html>
    <h4>Landmark ownership</h4>
<p>Landmarks retain identity, position and their reference/anchor relationship.
<a href=\"modelica://SLAM.Mapping.RGBDProjectLandmarks\">RGBDProjectLandmarks</a>
transforms valid optical points through camera extrinsics and body pose into
world coordinates. Its masks and validity outputs are part of the contract.</p>
<p><a href=\"modelica://SLAM.Mapping.RGBDAnchoredLandmarkMap\">RGBDAnchoredLandmarkMap</a>
and <a href=\"modelica://SLAM.Mapping.RGBDLandmarkCatalog\">RGBDLandmarkCatalog</a>
provide anchored and catalog representations. Array slots are storage, not
permanent feature identities. Preserve anchor and generation metadata across
map updates and graph corrections.</p>
<p>An estimated map is uncertain and correlated with the trajectory that built
it. Freezing its coordinates does not turn it into independent ground truth.
Keep that distinction explicit when using a reconstructed cloud as a synthetic
scene or as a navigation observation.</p>
    </html>"));
end Mapping;
