within;

package SLAM "Visual inertial localization, mapping and loop closure"
  annotation(uses(Vision, Estimation, LinearAlgebra, RigidBody, Control),
    Documentation(info = "<html>
    <p>Visual localization, mapping and loop closure. Image acquisition,
    matching, registration and calibration are in
    <a href=\"modelica://Vision\">Vision</a>.</p>
    <ul>
    <li><a href=\"modelica://SLAM.Localization\">Localization</a>: visual observations and registration uncertainty.</li>
    <li><a href=\"modelica://SLAM.Mapping\">Mapping</a>: anchored landmarks and spatial indexing.</li>
    <li><a href=\"modelica://SLAM.LoopClosure\">LoopClosure</a>: keyframes, retrieval and verification.</li>
    <li><a href=\"modelica://SLAM.PoseGraph\">PoseGraph</a>: measured factors and graph optimization.</li>
    <li><a href=\"modelica://SLAM.Fusion\">Fusion</a>: visual aiding for the navigation ESKF.</li>
    <li><a href=\"modelica://SLAM.Simulation\">Simulation</a>: fixed scenes and synthetic RGB-D observations.</li>
    <li><a href=\"modelica://SLAM.Examples\">Examples</a>: application and sensor examples.</li>
    </ul>
    <p><a href=\"modelica://SLAM.Inertial\">Inertial</a> is a world-additive
    ES15/Schmidt reference. It uses different tangent and injection conventions
    from <a href=\"modelica://Estimation.StrapdownINS.ESKF\">the navigation
    ESKF</a>; they require an explicit adapter to exchange internal states.
    Full graph execution must be checked against the selected compiler.</p>
    </html>"));
end SLAM;
