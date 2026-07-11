within LieGroups.SO3.EulerB321;
function exp_map "so(3) exponential in B321 Euler coordinates"
  input Real tangent[3];
  output Real element[3];
algorithm
  element := LieGroups.SO3.Euler.from_Matrix(
    LieGroups.SO3.Dcm.exp_map(tangent),
    {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true);
end exp_map;
