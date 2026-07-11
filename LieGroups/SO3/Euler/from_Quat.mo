within LieGroups.SO3.Euler;
function from_Quat "Scalar-first Hamilton quaternion to Euler angles"
  input Real q[4];
  input LieGroups.SO3.Euler.Axis sequence[3];
  input Boolean bodyFixed = true;
  output Real angles[3];
algorithm
  angles := LieGroups.SO3.Euler.from_Matrix(
    LieGroups.SO3.Quat.to_DCM(q), sequence, bodyFixed);
end from_Quat;
