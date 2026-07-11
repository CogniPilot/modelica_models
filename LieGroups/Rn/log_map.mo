within LieGroups.Rn;
function log_map "R^n logarithmic map"
  input Real element[:];
  output Real tangent[size(element, 1)];
algorithm
  tangent := element;
end log_map;
