within LieGroups.DirectProduct.SE2R3R3;
function inverse "Direct-product group inverse"
  input Real x[9];
  output Real result[9];
algorithm
  result[1:3] := LieGroups.SE2.inverse(x[1:3]);
  result[4:6] := LieGroups.Rn.inverse(x[4:6]);
  result[7:9] := LieGroups.Rn.inverse(x[7:9]);
end inverse;
