within LieGroups.SO3.Euler;
function to_Quat "Euler angles to scalar-first Hamilton quaternion"
  input Real angles[3];
  input LieGroups.SO3.Euler.Axis sequence[3];
  input Boolean bodyFixed = true;
  output Real q[4];
algorithm
  q := LieGroups.SO3.Quat.from_DCM(
    LieGroups.SO3.Euler.to_Matrix(angles, sequence, bodyFixed));
end to_Quat;
