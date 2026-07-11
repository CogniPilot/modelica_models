within LieGroups.SE3.Quat;
function left_jacobian_inv "Inverse left Jacobian of SE(3)"
  input Real tangent[6];
  output Real inverseJ[6, 6];
protected
  Real inverseRotationJ[3, 3];
  Real Q[3, 3];
algorithm
  inverseRotationJ := LieGroups.SO3.Quat.left_jacobian_inv(tangent[4:6]);
  Q := LieGroups.SE3.Quat.left_Q(tangent[1:3], tangent[4:6]);
  inverseJ := zeros(6, 6);
  inverseJ[1:3, 1:3] := inverseRotationJ;
  inverseJ[1:3, 4:6] := -inverseRotationJ * Q * inverseRotationJ;
  inverseJ[4:6, 4:6] := inverseRotationJ;
end left_jacobian_inv;
