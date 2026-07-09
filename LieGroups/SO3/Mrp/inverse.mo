within LieGroups.SO3.Mrp;
function inverse "MRP inverse: -r"
  input Real r[3];
  output Real r_inv[3];
algorithm
  for i in 1:3 loop
    r_inv[i] := -r[i];
  end for;
end inverse;
