within LieGroups.SE3.Quat;
function log_map "Logarithmic map: SE(3) -> se(3)"
  input Real X[7] "Group element {px,py,pz, qw,qx,qy,qz}";
  output Real xi[6] "Lie algebra {vx,vy,vz, omega_x,omega_y,omega_z}";
protected
  Real omega[3];
  Real J_inv[3,3];
algorithm
  // omega = log_SO3(q)
  omega := LieGroups.SO3.Quat.log_map(X[4:7]);
  xi[4:6] := omega;

  // v = J_l^{-1}(omega) * p
  J_inv := LieGroups.SO3.Quat.left_jacobian_inv(omega);
  xi[1:3] := J_inv * X[1:3];
end log_map;
