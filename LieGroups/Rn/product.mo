within LieGroups.Rn;
function product "R^n group product"
  input Real x[:];
  input Real y[size(x, 1)];
  output Real result[size(x, 1)];
algorithm
  result := x + y;
end product;
