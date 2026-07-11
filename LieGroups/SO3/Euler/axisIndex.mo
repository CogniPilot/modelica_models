within LieGroups.SO3.Euler;
function axisIndex "Convert an axis enumeration to a matrix index"
  input LieGroups.SO3.Euler.Axis axis;
  output Integer index;
algorithm
  index := if axis == LieGroups.SO3.Euler.Axis.x then 1
    elseif axis == LieGroups.SO3.Euler.Axis.y then 2 else 3;
end axisIndex;
