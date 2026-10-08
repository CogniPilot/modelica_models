within SLAM;

package Simulation "Landmark-level scenes without image processing"
  annotation(Documentation(info = "<html>
    <p><a href=\"modelica://SLAM.Simulation.LandmarkCamera\">LandmarkCamera</a>
    projects a fixed world scene into pixel/depth observations without image
    rendering or feature detection.
    <a href=\"modelica://SLAM.Simulation.landmarkGrid\">landmarkGrid</a>
    constructs a simple scene with stable row identities.</p>
    <p>A real RGB-D reconstruction may seed a frozen scene after transformation
    into an explicit world frame. Preserve its calibration, initialization pose
    and feature identities. Reconstructed coordinates are not independent
    ground truth: keep simulated truth separate from uncertain estimator maps.</p>
    </html>"));
end Simulation;
