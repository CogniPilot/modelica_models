within LieGroups.SO3.EulerB321;
function to_DCM "B321 Euler coordinates to DCM"
  input Real element[3];
  output Real R[3, 3];
algorithm
  R := LieGroups.SO3.Euler.to_Matrix(element,
    {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true);
end to_DCM;
