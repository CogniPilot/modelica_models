within ;
package Vehicles "Reusable vehicle models and named vehicle configurations"
  annotation(uses(
    Avionics,
    Control,
    Geodesy,
    LieGroups,
    MathUtilities,
    Planning,
    RigidBody),
    Documentation(info = "<html>
    <h4>Build a vehicle simulation</h4>
<p><a href=\"modelica://Vehicles.Templates\">Templates</a> contains reusable
parameterized plants. Named packages supply physical constants, controllers,
avionics boundaries and mission compositions:</p>
<ul>
<li><a href=\"modelica://Vehicles.Cubs2\">Cubs2</a>: fixed-wing outer-loop
control with an explicit onboard-stabilizer surrogate in simulation.</li>
<li><a href=\"modelica://Vehicles.Rdd2\">Rdd2</a>: multirotor dynamics, control
and aided-navigation missions.</li>
<li><a href=\"modelica://Vehicles.Interfaces\">Interfaces</a>: conversions
between transport-independent avionics records and plant signals.</li>
</ul>
<p>Choose a plant and its physical parameters, connect sensor models and a
navigation estimator, then connect planning, control and allocation. Compare
an aided mission with its truth-feedback baseline before attributing a flight
failure to a filter. Simulation qualification applies to the declared plant
and sensor assumptions.</p>
    </html>"));
end Vehicles;
