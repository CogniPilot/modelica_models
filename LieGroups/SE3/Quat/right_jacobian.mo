within LieGroups.SE3.Quat;
function right_jacobian "Right Jacobian of SE(3)"
  input Real tangent[6];
  output Real J[6, 6];
algorithm
  J := LieGroups.SE3.Quat.left_jacobian(-tangent);
end right_jacobian;
