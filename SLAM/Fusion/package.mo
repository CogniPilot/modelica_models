within SLAM;

package Fusion "Visual measurements for the CogniPilot navigation estimator"
  annotation(Documentation(info = "<html>
    <p>Independent visual corrections for the right-error navigation ESKF.</p>
    <ul>
    <li><a href=\"modelica://SLAM.Fusion.correctMappedLandmarksLoose\">correctMappedLandmarksLoose</a>
    compresses RGB-D observations into a six-dimensional pose statistic and
    requires sufficient geometric rank.</li>
    <li><a href=\"modelica://SLAM.Fusion.correctMappedLandmarksTight\">correctMappedLandmarksTight</a>
    uses pixel/depth residuals directly and can retain partial constraints.</li>
    </ul>
    <p>Both paths assume a fixed known map and independent new camera noise.
    Select visible observations and the matching rows and columns of their
    full covariance before calling either function; neither consumes a
    visibility mask. Uncertain live maps and external poses sharing IMU/GPS
    inputs require additional map uncertainty and cross-correlation treatment.</p>
    <p>Use <a href=\"modelica://SLAM.Simulation.LandmarkCamera\">LandmarkCamera</a>
    for synthetic inputs and
    <a href=\"modelica://Tests.VisualNavigationReplay\">VisualNavigationReplay</a>
    for sequential GPS and camera aiding.</p>
    </html>"));
end Fusion;
