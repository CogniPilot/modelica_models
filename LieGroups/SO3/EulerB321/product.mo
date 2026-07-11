within LieGroups.SO3.EulerB321;
function product "B321 Euler-coordinate group product"
  input Real left[3];
  input Real right[3];
  output Real result[3];
algorithm
  result := LieGroups.SO3.Euler.from_Matrix(
    LieGroups.SO3.Euler.to_Matrix(left,
      {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true)
    * LieGroups.SO3.Euler.to_Matrix(right,
      {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true),
    {LieGroups.SO3.Euler.Axis.z, LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.x}, true);
end product;
