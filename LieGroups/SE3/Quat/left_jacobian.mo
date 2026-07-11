within LieGroups.SE3.Quat;
function left_jacobian "Left Jacobian of SE(3)"
  input Real tangent[6];
  output Real J[6, 6];
protected
  Real rotationJ[3, 3];
algorithm
  rotationJ := LieGroups.SO3.Quat.left_jacobian(tangent[4:6]);
  J := zeros(6, 6);
  J[1:3, 1:3] := rotationJ;
  J[1:3, 4:6] := LieGroups.SE3.Quat.left_Q(tangent[1:3], tangent[4:6]);
  J[4:6, 4:6] := rotationJ;
end left_jacobian;
