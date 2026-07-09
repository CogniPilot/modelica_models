within LieGroups.SO3.Mrp;
function shadow "MRP shadow transformation: r_s = -r / ||r||^2"
  input Real r[3] "MRP parameters";
  output Real r_s[3] "Shadow MRP parameters";
protected
  Real n_sq;
  constant Real eps = 1e-10;
algorithm
  n_sq := max(r[1]^2 + r[2]^2 + r[3]^2, eps);
  for i in 1:3 loop
    r_s[i] := -r[i] / n_sq;
  end for;
end shadow;
