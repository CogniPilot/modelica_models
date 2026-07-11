within LieGroups.DirectProduct.SE2R3R3;
function product "Direct-product group product"
  input Real x[9];
  input Real y[9];
  output Real result[9];
algorithm
  result[1:3] := LieGroups.SE2.product(x[1:3], y[1:3]);
  result[4:6] := LieGroups.Rn.product(x[4:6], y[4:6]);
  result[7:9] := LieGroups.Rn.product(x[7:9], y[7:9]);
end product;
