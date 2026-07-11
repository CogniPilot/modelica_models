within LieGroups.DirectProduct.SE2R3R3;
function exp_map "Direct-product exponential map"
  input Real tangent[9];
  output Real element[9];
algorithm
  element[1:3] := LieGroups.SE2.exp_map(tangent[1:3]);
  element[4:6] := tangent[4:6];
  element[7:9] := tangent[7:9];
end exp_map;
