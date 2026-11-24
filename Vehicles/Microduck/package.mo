within Vehicles;
package Microduck "Microduck walking robot on the MultiBodyLie articulated-body library"
  annotation(Documentation(info = "<html>
    <h4>Microduck biped</h4>
<p><a href=\"modelica://Vehicles.Microduck.Robot\">Robot</a> is the named
assembly: fifteen bodies from
<a href=\"modelica://Vehicles.Microduck.Components\">Components</a> joined by
revolute servos whose reflected rotor inertia is the joint's
<code>armature</code>, with sensor, camera and range-sensor mounts. Ground
contact is <a href=\"modelica://MultiBodyLie.Contact\">MultiBodyLie.Contact</a>:
four sole spheres per foot and one sphere per body, against the plane z = 0
with gravity along -z.</p>
<p><a href=\"modelica://Vehicles.Microduck.Stand\">Stand</a> raises the leg
servo gains for standing; <a href=\"modelica://Vehicles.Microduck.JawLocked\">JawLocked</a>
replaces the jaw servo with a rigid joint and folds the jaw inertia into the
head. <a href=\"modelica://Vehicles.Microduck.PolicyInterface\">PolicyInterface</a>
maps walking-policy observations and actions to servo commands.
<a href=\"modelica://Vehicles.Microduck.Mechanics\">Mechanics</a> holds the
articulation helpers used to place the robot on the ground at start.</p>
    </html>"));
end Microduck;
