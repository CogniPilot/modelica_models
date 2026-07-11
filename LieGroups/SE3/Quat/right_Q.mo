within LieGroups.SE3.Quat;
function right_Q "Right Q block of the SE(3) Jacobian"
  input Real rho[3];
  input Real omega[3];
  output Real Q[3, 3];
algorithm
  Q := LieGroups.SE3.Quat.left_Q(-rho, -omega);
end right_Q;
