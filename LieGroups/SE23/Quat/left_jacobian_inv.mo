within LieGroups.SE23.Quat;
function left_jacobian_inv "Inverse left Jacobian of SE_2(3)"
  input Real tangent[9];
  output Real inverseJ[9, 9];
protected
  Real inverseRotationJ[3, 3];
  Real positionQ[3, 3];
  Real velocityQ[3, 3];
algorithm
  inverseRotationJ := LieGroups.SO3.Quat.left_jacobian_inv(tangent[7:9]);
  positionQ := LieGroups.SE3.Quat.left_Q(tangent[1:3], tangent[7:9]);
  velocityQ := LieGroups.SE3.Quat.left_Q(tangent[4:6], tangent[7:9]);
  inverseJ := zeros(9, 9);
  inverseJ[1:3, 1:3] := inverseRotationJ;
  inverseJ[1:3, 7:9] := -inverseRotationJ * positionQ * inverseRotationJ;
  inverseJ[4:6, 4:6] := inverseRotationJ;
  inverseJ[4:6, 7:9] := -inverseRotationJ * velocityQ * inverseRotationJ;
  inverseJ[7:9, 7:9] := inverseRotationJ;
end left_jacobian_inv;
