within LieGroups.DirectProduct.SE2R3R3;
function log_map "Direct-product logarithmic map"
  input Real element[9];
  output Real tangent[9];
algorithm
  tangent[1:3] := LieGroups.SE2.log_map(element[1:3]);
  tangent[4:6] := element[4:6];
  tangent[7:9] := element[7:9];
end log_map;
