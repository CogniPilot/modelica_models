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
end Rdd2;
