within LieGroups.SE3.Quat;
function right_jacobian_inv "Inverse right Jacobian of SE(3)"
  input Real tangent[6];
  output Real inverseJ[6, 6];
algorithm
  inverseJ := LieGroups.SE3.Quat.left_jacobian_inv(-tangent);
end right_jacobian_inv;
