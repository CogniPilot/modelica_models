within;

package Vision "Calibrated visual acquisition, matching and registration"
  annotation(uses(LinearAlgebra), Documentation(info = "<html>
    <p>Visual front-end building blocks:
    <a href=\"modelica://Vision.Features\">Features</a> detects and selects
    features; <a href=\"modelica://Vision.Matching\">Matching</a> supplies
    descriptors and tracking;
    <a href=\"modelica://Vision.Registration\">Registration</a> estimates
    calibrated point alignment; and
    <a href=\"modelica://Vision.Sensors\">Sensors</a> supplies camera profiles.
    Localization, mapping and visual navigation are in
    <a href=\"modelica://SLAM\">SLAM</a>.</p>
    </html>"));
end Vision;
