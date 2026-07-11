within LieGroups.SE3.Quat;
function exp_map "Exponential map: se(3) -> SE(3)"
  input Real xi[6] "Lie algebra {vx,vy,vz, omega_x,omega_y,omega_z}";
  output Real X[7] "Group element {px,py,pz, qw,qx,qy,qz}";
protected
  Real v[3] "Translational component";
  Real omega[3] "Rotational component";
  Real J[3,3] "SO(3) left Jacobian";
  Real q[4] "Rotation quaternion";
algorithm
  v := xi[1:3];
  omega := xi[4:6];

  // p = J_l(omega) * v
  J := LieGroups.SO3.Quat.left_jacobian(omega);
  X[1:3] := J * v;

  // q = exp_SO3(omega)
  q := LieGroups.SO3.Quat.exp_map(omega);
  X[4:7] := q;
end exp_map;
