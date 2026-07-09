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

  for i in 1:3 loop
    X[i] :=
      J[i,1]*xi[1] + J[i,2]*xi[2] + J[i,3]*xi[3];
    X[i + 3] :=
      J[i,1]*xi[4] + J[i,2]*xi[5] + J[i,3]*xi[6];
  end for;

  for i in 1:4 loop
    X[i + 6] := q[i];
  end for;
end exp_map;
