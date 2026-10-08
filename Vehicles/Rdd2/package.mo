within Vehicles;
package Rdd2 "RDD2 vehicle configuration and flight-control models"
  record RotorGeometry
    "RDD2 motor locations and reaction-torque directions in command order"
    parameter Real armLength_m(unit = "m") = 0.25 annotation(Evaluate = true);
    Real effectiveMomentArm_m(unit = "m") = armLength_m / sqrt(2.0)
      "Quad-X roll/pitch lever arm";
    parameter Real rotorTorqueRatio_m(unit = "m") = 0.016 annotation(Evaluate = true);
    Real positionBodyFlu_m[4, 3] = [
       effectiveMomentArm_m, -effectiveMomentArm_m, 0.0;
      -effectiveMomentArm_m, -effectiveMomentArm_m, 0.0;
      -effectiveMomentArm_m,  effectiveMomentArm_m, 0.0;
       effectiveMomentArm_m,  effectiveMomentArm_m, 0.0];
    Real yawMomentPerThrust_m[4] =
      rotorTorqueRatio_m * {-1.0, 1.0, -1.0, 1.0};
  end RotorGeometry;
  annotation(Documentation(info = "<html>
    <h4>RDD2 multirotor</h4>
<p>This package binds reusable dynamics and control to the RDD2 vehicle.
<a href=\"modelica://Vehicles.Rdd2.RotorGeometry\">RotorGeometry</a> defines the
motor order, positions in body FLU and yaw torque signs. Keep that order
consistent through allocation and actuator connections.</p>
<p>Start with <a href=\"modelica://Vehicles.Rdd2.Test\">Test</a> to compare truth,
optical-flow-aided and GPS-aided waypoint missions. The alternate
<a href=\"modelica://Vehicles.Rdd2.LogLinearController\">LogLinearController</a>
parameterizes the shared controller rather than duplicating its mathematics.
Use mission sensor configuration, source timestamps and the local origin
consistently when changing navigation options.</p>
    </html>"));
end Rdd2;
