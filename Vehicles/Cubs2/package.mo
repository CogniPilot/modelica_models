within Vehicles;
package Cubs2 "CUBS2 vehicle configuration and flight-control models"
  annotation(Documentation(info = "<html>
    <p><a href=\"modelica://Vehicles.Cubs2.OuterLoop\">OuterLoop</a> is the
    deployable controller. Closed-loop tests use
    <a href=\"modelica://Vehicles.Cubs2.OnboardStabilizerSurrogate\">OnboardStabilizerSurrogate</a>
    for the proprietary onboard stabilizer. They qualify the outer loop under
    that simulation assumption, not the unavailable stabilizer implementation.</p>
    </html>"));
end Cubs2;
