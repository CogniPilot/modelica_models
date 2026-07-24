within LieGroups.SE23.Quat;
function log_map "Logarithmic map: SE_2(3) -> se_2(3)"
  input Real X[10] "Group element {p, v, q}";
  output Real xi[9] "Lie algebra {vb, ab, omega}";
protected
  Real omega[3];
  Real J_inv[3,3];
algorithm
  // omega = log_SO3(q)
  omega := LieGroups.SO3.Quat.log_map(X[7:10]);
  xi[7:9] := omega;

  J_inv := LieGroups.SO3.Quat.left_jacobian_inv(omega);
  // vb = J_l^{-1} * p
  xi[1:3] := J_inv * X[1:3];
  // ab = J_l^{-1} * v
  xi[4:6] := J_inv * X[4:6];
end log_map;
