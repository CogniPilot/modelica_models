within LieGroups.SO3.Euler;
function rotationMatrix "Elementary active rotation about a coordinate axis"
  input LieGroups.SO3.Euler.Axis axis;
  input Real angle;
  output Real R[3, 3];
protected
  Real c;
  Real s;
algorithm
  c := cos(angle);
  s := sin(angle);
  if axis == LieGroups.SO3.Euler.Axis.x then
    R := [1.0, 0.0, 0.0; 0.0, c, -s; 0.0, s, c];
  elseif axis == LieGroups.SO3.Euler.Axis.y then
    R := [c, 0.0, s; 0.0, 1.0, 0.0; -s, 0.0, c];
  else
    R := [c, -s, 0.0; s, c, 0.0; 0.0, 0.0, 1.0];
  end if;
end rotationMatrix;
