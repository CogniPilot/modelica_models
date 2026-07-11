within LieGroups.Rn;
function exp_map "R^n exponential map"
  input Real tangent[:];
  output Real element[size(tangent, 1)];
algorithm
  element := tangent;
end exp_map;
