within LieGroups.Rn;
function inverse "R^n group inverse"
  input Real x[:];
  output Real result[size(x, 1)];
algorithm
  result := -x;
end inverse;
