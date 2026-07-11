within LieGroups.SE23.Quat;
function right_jacobian "Right Jacobian of SE_2(3)"
  input Real tangent[9];
  output Real J[9, 9];
algorithm
  J := LieGroups.SE23.Quat.left_jacobian(-tangent);
end right_jacobian;
