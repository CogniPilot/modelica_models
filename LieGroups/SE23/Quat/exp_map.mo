within LieGroups.SE23.Quat;
function exp_map "Exponential map: se_2(3) -> SE_2(3)"
  input Real xi[9] "Lie algebra {vb, ab, omega}";
  output Real X[10] "Group element {p, v, q}";
protected
  Real J[3,3] "SO(3) left Jacobian";
  Real q[4] "Rotation exponential";
algorithm
  J := LieGroups.SO3.Quat.left_jacobian(xi[7:9]);
  q := LieGroups.SO3.Quat.exp_map(xi[7:9]);

  X[1:3] := J * xi[1:3];
  X[4:6] := J * xi[4:6];
  X[7:10] := q;
end exp_map;
