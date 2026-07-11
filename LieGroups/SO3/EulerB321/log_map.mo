within LieGroups.SO3.EulerB321;
function log_map "B321 Euler coordinates to so(3) tangent"
  input Real element[3];
  output Real tangent[3];
algorithm
  tangent := LieGroups.SO3.Dcm.log_map(
    LieGroups.SO3.Euler.to_Matrix(element,
      {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true));
end log_map;
