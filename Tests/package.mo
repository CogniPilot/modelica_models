within;
package Tests "Assertion-based tests for the reusable Modelica libraries"
  annotation(uses(
    Avionics,
    Control,
    Estimation,
    Geodesy,
    LieGroups,
    LinearAlgebra,
    Planning,
    Polynomials,
    RigidBody,
    Vehicles),
    Documentation(info = "<html>
    <h4>Executable examples and checks</h4>
<p>These Modelica classes exercise numerical primitives, geometry, filter
interfaces and replay boundaries. Assertions state the expected invariant;
replay classes expose dynamic inputs for generated-code validation.</p>
<ul>
<li><a href=\"modelica://Tests.PreintegrationReplay\">PreintegrationReplay</a>:
IMU increment composition.</li>
<li><a href=\"modelica://Tests.CorrelatedGpsTests\">CorrelatedGpsTests</a>:
correlated held-input GPS correction.</li>
<li><a href=\"modelica://Tests.SyntheticLandmarkReplay\">SyntheticLandmarkReplay</a>:
camera projection and visibility.</li>
<li><a href=\"modelica://Tests.VisualNavigationReplay\">VisualNavigationReplay</a>:
sequential loose/tight visual aiding.</li>
</ul>
<p>Root-level test classes are not a replacement for vehicle missions. Those
live under each vehicle's Test package and combine plant, controller and
sensor behavior. A numerical assertion, a compiler export check and a flight
qualification answer different questions.</p>
    </html>"));
end Tests;
