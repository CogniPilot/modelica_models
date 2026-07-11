within LieGroups.SE23.Quat;
function log_map "Logarithmic map: SE_2(3) -> se_2(3)"
  input Real X[10] "Group element {p, v, q}";
  output Real xi[9] "Lie algebra {vb, ab, omega}";
protected
  Real omega[3];
  Real theta_sq;
  Real J_inv[3,3];
  constant Real eps = 1e-8;
algorithm
  // omega = log_SO3(q)
  omega := LieGroups.SO3.Quat.log_map(X[7:10]);
  xi[7:9] := omega;

  theta_sq := omega[1]^2 + omega[2]^2 + omega[3]^2;

  if theta_sq < eps then
    // Near identity: V_inv ~ I
    // vb = p, ab = v
    xi[1:3] := X[1:3];
    xi[4:6] := X[4:6];
  else
    J_inv := LieGroups.SO3.Quat.left_jacobian_inv(omega);
    // vb = J_l^{-1} * p
    xi[1:3] := J_inv * X[1:3];
    // ab = J_l^{-1} * v
    xi[4:6] := J_inv * X[4:6];
  end if;
end log_map;
