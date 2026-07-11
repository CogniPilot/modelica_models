within LieGroups.SE23.Quat;
function right_jacobian_inv "Inverse right Jacobian of SE_2(3)"
  input Real tangent[9];
  output Real inverseJ[9, 9];
algorithm
  inverseJ := LieGroups.SE23.Quat.left_jacobian_inv(-tangent);
end right_jacobian_inv;
