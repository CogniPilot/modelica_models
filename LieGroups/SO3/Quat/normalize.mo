within LieGroups.SO3.Quat;
function normalize "Normalize quaternion to unit length"
  input Real q[4];
  output Real q_n[4];
protected
  Real n;
algorithm
  n := max(sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2), 1e-10);
  q_n[1] := q[1] / n;
  q_n[2] := q[2] / n;
  q_n[3] := q[3] / n;
  q_n[4] := q[4] / n;
end normalize;
