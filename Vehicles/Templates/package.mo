within Vehicles;
package Templates "Parameterized vehicle dynamics templates"
  annotation(Documentation(info = "<html>
    <h4>Reusable plants</h4>
<p>Parameterized vehicle dynamics separate physical equations from named-airframe
constants. <a href=\"modelica://Vehicles.Templates.FixedWingPlant\">FixedWingPlant</a>
models fixed-wing motion; inspect the members below for the available plant
interfaces. Use a named configuration under
<a href=\"modelica://Vehicles.Cubs2\">Cubs2</a> or
<a href=\"modelica://Vehicles.Rdd2\">Rdd2</a> when reproducing a vehicle mission.</p>
<p>Supply mass, inertia, aerodynamic or propulsion parameters in the declared
units. A controller's tuning and a plant's physical parameters are different
parts of a simulation; changing one is not calibration of the other.</p>
    </html>"));
end Templates;
