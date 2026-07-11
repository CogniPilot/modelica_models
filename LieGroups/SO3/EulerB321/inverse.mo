within LieGroups.SO3.EulerB321;
function inverse "B321 Euler-coordinate group inverse"
  input Real element[3];
  output Real result[3];
protected
  Real R[3, 3];
algorithm
  R := LieGroups.SO3.Euler.to_Matrix(element,
    {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true);
  result := LieGroups.SO3.Euler.from_Matrix(transpose(R),
    {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true);
end inverse;
